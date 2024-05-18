#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Delay;
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{AdcConfig, Attenuation, ADC},
    clock::{ClockControl, Clocks},
    dma::*,
    dma_descriptors, embassy,
    gpio::{self, Floating, GpioPin, Input, Output, IO},
    peripherals::{Peripherals, ADC2, RNG, SPI2},
    prelude::*,
    rtc_cntl::{get_reset_reason, get_wakeup_cause, sleep::TimerWakeupSource, Rtc, SocResetReason},
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    timer::TimerGroup,
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

/// Lora Max TX power
const MAX_TX_POWER: u8 = 14;

/// The following variables are stored in RTC RAM to keep their values
/// after deep sleep
#[ram(rtc_fast)]
static mut BOOT_CNT: u8 = 0;
#[ram(rtc_fast)]
static mut SEED: u32 = 0;
#[ram(rtc_fast)]
static mut IS_JOIN: bool = false;
#[ram(rtc_fast)]
static mut SAVED_SESSION: Option<Session> = None;

// All GPIO needed for SPI
struct SpiGpio {
    sclk: GpioPin<gpio::Unknown, 10>,
    miso: GpioPin<gpio::Unknown, 6>,
    mosi: GpioPin<gpio::Unknown, 7>,
    nss: GpioPin<Output<gpio::OpenDrain>, 8>,
    reset: GpioPin<Output<gpio::OpenDrain>, 5>,
    dio1: GpioPin<Input<gpio::PullDown>, 3>,
    busy: GpioPin<Input<gpio::PullDown>, 4>,
}

/// Join lorawan network then send a message if join is success
/// The session is saved in RTC RAM to be able to restore it after
/// deep sleep
async fn send_lorawan_msg(
    spi2: SPI2,
    dma: Dma<'_>,
    clocks: &Clocks<'_>,
    spi_gpio: SpiGpio,
    data: &mut [u8; 12],
) {
    let dma_channel = dma.channel0;
    let (mut descriptors, mut rx_descriptors) = dma_descriptors!(32000);

    let mut spi_bus = Spi::new(spi2, 200u32.kHz(), SpiMode::Mode0, clocks)
        .with_pins(
            Some(spi_gpio.sclk),
            Some(spi_gpio.mosi),
            Some(spi_gpio.miso),
            gpio::NO_PIN,
        )
        .with_dma(dma_channel.configure_for_async(
            false,
            &mut descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ));
    let spi = ExclusiveDevice::new(&mut spi_bus, spi_gpio.nss, Delay);

    // Configure Sx1262 chip
    let config = sx126x::Config {
        chip: Sx126xVariant::Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: false,
        use_dio2_as_rfswitch: true,
        rx_boost: false,
    };
    let iv = GenericSx126xInterfaceVariant::new(
        spi_gpio.reset,
        spi_gpio.dio1.degrade(),
        spi_gpio.busy.degrade(),
        None,
        None,
    )
    .unwrap();
    let lora = LoRa::new(Sx126x::new(spi, iv, config), true, Delay)
        .await
        .unwrap();
    let radio: LorawanRadio<_, _, MAX_TX_POWER> = lora.into();
    let region: region::Configuration = region::Configuration::new(region::Region::EU868);

    let mut is_join: bool = false;
    unsafe {
        if IS_JOIN {
            is_join = true;
        }
    }

    if !is_join {
        println!("Ask to join lora network");
        let rng_reg = unsafe { &*RNG::PTR };
        let seed = rng_reg.data().read().bits();
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
            let send_status = device.send(data, 1, false).await.unwrap();
            match send_status {
                SendResponse::RxComplete => {
                    println!("LoRaWAN send succes");
                    unsafe {
                        SEED = seed;
                        IS_JOIN = true;
                        SAVED_SESSION = device.get_session().cloned();
                    }
                }
                _ => {
                    println!("LoRaWAN send error, reset session");
                    unsafe {
                        IS_JOIN = false;
                    }
                }
            }
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
                let send_status = device.send(data, 1, false).await.unwrap();

                match send_status {
                    SendResponse::RxComplete => {
                        println!("LoRaWAN send succes");
                        SAVED_SESSION = device.get_session().cloned();
                    }
                    _ => {
                        println!("LoRaWAN send error, reset session");
                        IS_JOIN = false;
                    }
                }
            }
        }
    }
}

/// Read vbatt from ADC2 GPIO1 and returns the value in mV
/// Hardware gain is 0.5
fn read_vbat(
    mut adc2_pin: esp_hal::analog::adc::AdcPin<
        GpioPin<gpio::Analog, 1>,
        esp_hal::peripherals::ADC2,
    >,
    mut adc2: ADC<ADC2>,
) -> u16 {
    // Read vbat
    let raw_value: u16 = nb::block!(adc2.read_oneshot(&mut adc2_pin)).unwrap();
    raw_value * 2
}

/// Read hx7111
fn hx7111_read_value(
    hx711_dt: GpioPin<Input<Floating>, 21>,
    hx711_sck: GpioPin<Output<esp_hal::gpio::PushPull>, 20>,
    delay: esp_hal::delay::Delay,
) -> u32 {
    let mut hx7111_value: u32 = 0;
    let mut load_sensor = hx711::HX711::new(hx711_sck, hx711_dt, delay);
    load_sensor.set_scale(1.0);
    //set the sensitivity/scale
    // load_sensor.tare(16);
    // Wait HX711 to be ready
    for _ in 1..=10 {
        if load_sensor.is_ready() {
            let reading = load_sensor.read_scaled();
            match reading {
                Ok(x) => {
                    hx7111_value = x as u32;
                }
                Err(_) => println!("Error reading HX711"),
            }
            println!("HX711 reading = {:?}", reading);
            break;
        }
        delay.delay_millis(100u32);
        println!("Wait for HX711 available");
    }

    hx7111_value
}

#[main]
async fn main(_spawner: Spawner) {
    // Configure peripherals
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rtc = Rtc::new(peripherals.LPWR, None);
    embassy::init(&clocks, timg0);
    let dma: Dma = Dma::new(peripherals.DMA);
    let mut delay = esp_hal::delay::Delay::new(&clocks);

    println!("SW version 1.0.0");
    // Print wake or reset reason
    // TODO send the reset reason to Lorax
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

    // Enable HX711 and ADC power
    let mut power_supply_enable = io.pins.gpio0.into_push_pull_output();
    power_supply_enable.set_high();

    // Read HX711 value
    let raw_value: u32 = hx7111_read_value(
        io.pins.gpio21.into_floating_input(),
        io.pins.gpio20.into_push_pull_output(),
        delay,
    );

    // Read vbat measure
    let analog_pin = io.pins.gpio1.into_analog();
    let mut adc2_config = AdcConfig::new();
    let adc2_pin: esp_hal::analog::adc::AdcPin<
        GpioPin<gpio::Analog, 1>,
        esp_hal::peripherals::ADC2,
    > = adc2_config.enable_pin(analog_pin, Attenuation::Attenuation11dB);

    let adc2: ADC<ADC2> = ADC::<ADC2>::new(peripherals.ADC2, adc2_config);
    let vbat: u16 = read_vbat(adc2_pin, adc2);
    println!("ADC reading = {} mV", vbat);

    // Configure GPIO for SPI
    let sclk: GpioPin<gpio::Unknown, 10> = io.pins.gpio10;
    let miso: GpioPin<gpio::Unknown, 6> = io.pins.gpio6;
    let mosi: GpioPin<gpio::Unknown, 7> = io.pins.gpio7;
    let nss: GpioPin<Output<gpio::OpenDrain>, 8> = io.pins.gpio8.into_open_drain_output();
    let reset: GpioPin<Output<gpio::OpenDrain>, 5> = io.pins.gpio5.into_open_drain_output();
    let dio1: GpioPin<Input<gpio::PullDown>, 3> = io.pins.gpio3.into_pull_down_input();
    let busy: GpioPin<Input<gpio::PullDown>, 4> = io.pins.gpio4.into_pull_down_input();

    // Build Loraframe
    let mut lora_frame: [u8; 12] = [0; 12];
    lora_frame[1] = ((vbat - 3000) / 5) as u8;
    lora_frame[2] = ((raw_value >> 24) & 0xFF) as u8;
    lora_frame[3] = ((raw_value >> 16) & 0xFF) as u8;
    lora_frame[4] = ((raw_value >> 8) & 0xFF) as u8;

    let spi_gpio = SpiGpio {
        sclk,
        miso,
        mosi,
        nss,
        reset,
        dio1,
        busy,
    };

    // Send messsage on Lora network
    send_lorawan_msg(peripherals.SPI2, dma, &clocks, spi_gpio, &mut lora_frame).await;

    println!("End of cycle, go to sleep");
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(300));
    delay.delay_millis(100u32);
    rtc.sleep_deep(&[&timer], &mut delay);
}
