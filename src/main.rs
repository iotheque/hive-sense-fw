#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::*,
    dma_descriptors, embassy,
    gpio::{self, Io},
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    rtc_cntl::{get_reset_reason, get_wakeup_cause, sleep::TimerWakeupSource, Rtc, SocResetReason},
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    system::SystemControl,
    timer::timg::TimerGroup,
    Cpu,
};
use esp_println::println;
use loadcell::{hx711, LoadCell};
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::lorawan_radio::LorawanRadio;
use lora_phy::sx126x::{self, Sx126x, Sx126xVariant, TcxoCtrlVoltage};
use lora_phy::LoRa;
use lorawan_device::{
    async_device::{region, Device, EmbassyTimer, JoinMode},
    AppEui, AppKey, DevEui,
};
use lorawan_device::{
    async_device::{JoinResponse, SendResponse},
    default_crypto::DefaultFactory as Crypto,
    mac::Session,
};

const LORAWAN_REGION: region::Region = region::Region::EU868;
const MAX_TX_POWER: u8 = 14;

#[ram(rtc_fast)]
static mut BOOT_CNT: u8 = 0;
#[ram(rtc_fast)]
static mut SEED: u32 = 0;
#[ram(rtc_fast)]
static mut IS_JOIN: bool = false;
#[ram(rtc_fast)]
static mut SAVED_SESSION: Option<Session> = None;

#[main]
async fn main(_spawner: Spawner) {
    // Configure peripherals
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rtc = Rtc::new(peripherals.LPWR, None);
    let mut rng = Rng::new(peripherals.RNG);
    embassy::init(&clocks, timg0);
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;
    let (mut descriptors, mut rx_descriptors) = dma_descriptors!(32000);

    // FIXME: remove it, it is used no see logs after power on (time to open serial port)
    let mut delay = esp_hal::delay::Delay::new(&clocks);
    delay.delay_millis(10000u32);

    println!("SW version 1.0.0");
    // Print wake or reset reason
    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    println!("Reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    println!("Wake reason: {:?}", wake_reason);

    unsafe {
        println!("BOOT_CNT {:x?}", BOOT_CNT);
        BOOT_CNT += 1;
        println!("IS_JOIN: {:?}", IS_JOIN);
        println!("SAVED_SESSION: {:?}", SAVED_SESSION);
        println!("SEED: {:x?}", SEED);
    }

    // Configure SPI
    let sclk = io.pins.gpio10;
    let miso = io.pins.gpio6;
    let mosi = io.pins.gpio7;
    let nss = io.pins.gpio8.into_open_drain_output();
    let reset = io.pins.gpio5.into_open_drain_output();
    let dio1 = io.pins.gpio3.into_pull_down_input().degrade();
    let busy = io.pins.gpio4.into_pull_down_input().degrade();

    // HX711 Pins configuration
    let hx711_dt = io.pins.gpio21.into_floating_input();
    let hx711_sck = io.pins.gpio20.into_push_pull_output();
    let mut hx711_enable = io.pins.gpio0.into_push_pull_output();
    let _ = hx711_enable.set_high();
    // HX711 configuration
    let mut load_sensor = hx711::HX711::new(hx711_sck, hx711_dt, delay);
    //load_sensor.tare(16);
    //set the sensitivity/scale
    load_sensor.set_scale(1.0);

    let mut spi_bus = Spi::new(peripherals.SPI2, 200u32.kHz(), SpiMode::Mode0, &clocks)
        .with_pins(Some(sclk), Some(mosi), Some(miso), gpio::NO_PIN)
        .with_dma(dma_channel.configure_for_async(
            false,
            &mut descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ));
    let spi = ExclusiveDevice::new(&mut spi_bus, nss, Delay);

    // Configure Sx1262 chip
    let config = sx126x::Config {
        chip: Sx126xVariant::Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: false,
        use_dio2_as_rfswitch: true,
        rx_boost: false,
    };
    let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();
    let lora = LoRa::new(Sx126x::new(spi, iv, config), true, Delay)
        .await
        .unwrap();
    let radio: LorawanRadio<_, _, MAX_TX_POWER> = lora.into();
    let region: region::Configuration = region::Configuration::new(LORAWAN_REGION);

    // Get HX711 data
    let mut hx711_value: [u8; 4] = [0, 0, 0, 0];
    // Wait HX711 to be ready
    for _ in 1..=10 {
        if load_sensor.is_ready() {
            let reading = load_sensor.read_scaled();
            match reading {
                Ok(x) => {
                    let raw_value: u32 = x as u32;
                    hx711_value[0] = ((raw_value >> 24) & 0xFF) as u8;
                    hx711_value[1] = ((raw_value >> 16) & 0xFF) as u8;
                    hx711_value[2] = ((raw_value >> 8) & 0xFF) as u8;
                    hx711_value[3] = (raw_value & 0xFF) as u8;
                },
                Err(_) => println!("Error reading HX711"),
            }
            println!("HX711 reading = {:?}", reading);
            break;
        }
        delay.delay_millis(100u32);
        println!("Wait for HX711 available");
    }

    let mut is_join: bool = false;
    unsafe {
        if IS_JOIN {
            is_join = true;
        }
    }
    if !is_join {
        println!("Ask to join lora network");
        let seed = rng.random();
        let mut device: Device<_, Crypto, _, _> =
            Device::new_with_seed(region, radio, EmbassyTimer::new(), seed.into());
        let resp = device
            .join(&JoinMode::OTAA {
                deveui: DevEui::from([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
                appeui: AppEui::from([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
                appkey: AppKey::from([
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00,
                ]),
            })
            .await;
        if let Ok(JoinResponse::JoinSuccess) = resp {
            println!("LoRaWAN network joined");
            let send_status = device
                .send(&hx711_value, 1, false)
                .await
                .unwrap();
            match send_status {
                SendResponse::RxComplete => {
                    println!("LoRaWAN send succes");
                    // Save state in RTC RAM
                    unsafe {
                        SEED = seed;
                        IS_JOIN = true;
                        SAVED_SESSION = device.get_session().clone().cloned();
                    }
                }
                _ => {
                    println!("LoRaWAN send error, reset session");
                    unsafe {
                        IS_JOIN = false;
                    }
                }
            }

            delay.delay_millis(1000u32);
        } else {
            // Save state in RTC RAM
            unsafe {
                IS_JOIN = false;
            }
            println!("CAN NOT join LoRaWAN network");
        }
    } else {
        println!("We are already joined use saved session");
        unsafe {
            let seed: u32 = SEED;
            if let Some(saved_session) = SAVED_SESSION.clone() {
                let mut device: Device<_, Crypto, _, _> = Device::new_with_seed_and_session(
                    region,
                    radio,
                    EmbassyTimer::new(),
                    seed.into(),
                    Some(saved_session),
                );
                let send_status = device
                    .send(&hx711_value, 1, false)
                    .await
                    .unwrap();

                match send_status {
                    SendResponse::RxComplete => {
                        println!("LoRaWAN send succes");
                        SAVED_SESSION = device.get_session().clone().cloned();
                    }
                    _ => {
                        println!("LoRaWAN send error, reset session");
                        IS_JOIN = false;
                    }
                }
            }
        }
    }

    delay.delay_millis(5000u32);
    println!("Go to sleep");
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(10));
    delay.delay_millis(100u32);
    rtc.sleep_deep(&[&timer], &mut delay);
}
