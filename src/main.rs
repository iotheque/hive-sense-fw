#![no_std]
#![no_main]
mod cli;
mod consts;
use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_storage::ReadStorage;
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::{ClockControl, Clocks},
    dma::*,
    dma_descriptors,
    gpio::{self, AnyInput, AnyOutput, GpioPin, Input, Io, Level, Output, Pull},
    peripherals::{Peripherals, SPI2, WIFI},
    prelude::*,
    rng::Rng,
    rtc_cntl::{get_reset_reason, get_wakeup_cause, sleep::TimerWakeupSource, Rtc, SocResetReason},
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    system::SystemControl,
    timer::{systimer::SystemTimer, timg::TimerGroup},
    Cpu,
};
use esp_println::println;
use esp_storage::FlashStorage;
use esp_wifi::{
    initialize,
    wifi::{AccessPointInfo, WifiError, WifiStaDevice},
    EspWifiInitFor,
};
use loadcell::{hx711, LoadCell};
use lora_phy::lorawan_radio::LorawanRadio;
use lora_phy::sx126x::{self, Sx126x, TcxoCtrlVoltage};
use lora_phy::LoRa;
use lora_phy::{iv::GenericSx126xInterfaceVariant, sx126x::Sx1262};
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

/// Lora data fame size in bytes
const LORA_FRAME_SIZE_BYTES: usize = 24;

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
    sclk: GpioPin<10>,
    miso: GpioPin<6>,
    mosi: GpioPin<7>,
    nss: GpioPin<8>,
    reset: GpioPin<5>,
    dio1: GpioPin<3>,
    busy: GpioPin<4>,
}

/// Join lorawan network then send a message if join is success
/// The session is saved in RTC RAM to be able to restore it after
/// deep sleep
async fn send_lorawan_msg(
    spi2: SPI2,
    dma: Dma<'_>,
    mut rng: Rng,
    clocks: &Clocks<'_>,
    spi_gpio: SpiGpio,
    data: &mut [u8; LORA_FRAME_SIZE_BYTES],
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
    let spi = ExclusiveDevice::new(&mut spi_bus, Output::new(spi_gpio.nss, Level::High), Delay);

    // Configure Sx1262 chip
    let config = sx126x::Config {
        chip: Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: false,
        rx_boost: false,
    };

    let io_reset = AnyOutput::new(spi_gpio.reset, Level::High);
    let io_dio1 = AnyInput::new(spi_gpio.dio1, Pull::Down);
    let io_busy = AnyInput::new(spi_gpio.busy, Pull::Down);

    let iv = GenericSx126xInterfaceVariant::new(io_reset, io_dio1, io_busy, None, None).unwrap();
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
        let mut flash = FlashStorage::new();
        let mut dev_eui = [0u8; 8];
        flash
            .read(consts::NVS_DEV_EUI_ADDRESS, &mut dev_eui)
            .unwrap();
        let mut app_eui = [0u8; 8];
        flash
            .read(consts::NVS_APP_EUI_ADDRESS, &mut app_eui)
            .unwrap();
        let mut app_key = [0u8; 16];
        flash
            .read(consts::NVS_APP_KEY_ADDRESS, &mut app_key)
            .unwrap();

        let seed = rng.random();
        let mut device: Device<_, Crypto, _, _> =
            Device::new_with_seed(region, radio, EmbassyTimer::new(), seed.into());
        let resp = device
            .join(&JoinMode::OTAA {
                deveui: DevEui::from(dev_eui),
                appeui: AppEui::from(app_eui),
                appkey: AppKey::from(app_key),
            })
            .await;
        if let Ok(JoinResponse::JoinSuccess) = resp {
            println!("LoRaWAN network joined");
            let send_status = device.send(data, 1, false).await.unwrap();
            match send_status {
                SendResponse::RxComplete | SendResponse::DownlinkReceived(0) => {
                    println!("LoRaWAN send succes");
                    unsafe {
                        SEED = seed;
                        IS_JOIN = true;
                        SAVED_SESSION = device.get_session().cloned();
                    }
                }
                _ => {
                    println!("LoRaWAN send error, reset session : {:?}", send_status);
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
                println!("send_status {:?}", send_status);

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
    mut adc2_pin: esp_hal::analog::adc::AdcPin<GpioPin<1>, esp_hal::peripherals::ADC2>,
    mut adc2: Adc<esp_hal::peripherals::ADC2>,
) -> u16 {
    // Read vbat
    let raw_value: u16 = nb::block!(adc2.read_oneshot(&mut adc2_pin)).unwrap();
    raw_value * 2
}

/// Read hx7111
fn hx7111_read_value(
    hx711_dt: GpioPin<21>,
    hx711_sck: GpioPin<20>,
    delay: esp_hal::delay::Delay,
) -> u32 {
    let mut hx7111_value: u32 = 0;
    let io_hx711_dt = Input::new(hx711_dt, Pull::None);
    let io_hx711_sck = Output::new(hx711_sck, Level::Low);

    let mut load_sensor = hx711::HX711::new(io_hx711_sck, io_hx711_dt, delay);
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

/// Wifi scan
fn scan_wifi(init: esp_wifi::EspWifiInitialization, wifi: WIFI, out: &mut [u8]) {
    let (_, mut controller) = esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();
    controller.start().unwrap();
    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
    println!("Result is {:?}", res);
    match res {
        Ok((access_points, count)) => {
            println!("Number of access points found: {}", count);
            for (i, ap) in access_points.iter().enumerate().take(2) {
                println!("SSID: {}", ap.ssid);
                println!(
                    "BSSID: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                    ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3], ap.bssid[4], ap.bssid[5]
                );
                out[i] = ap.bssid[0];
                out[i + 1] = ap.bssid[1];
                out[i + 2] = ap.bssid[2];
                out[i + 3] = ap.bssid[3];
                out[i + 4] = ap.bssid[4];
                out[i + 5] = ap.bssid[5];
            }
        }
        Err(e) => {
            println!("Failed to scan WiFi: {:?}", e);
        }
    }
}

#[main]
async fn main(spawner: Spawner) {
    // Configure peripherals
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let timg0 = TimerGroup::new_async(peripherals.TIMG0, &clocks);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let mut rtc = Rtc::new(peripherals.LPWR, None);
    let rng: Rng = Rng::new(peripherals.RNG);
    esp_hal_embassy::init(&clocks, timg0);
    let wifi = peripherals.WIFI;
    let dma: Dma = Dma::new(peripherals.DMA);
    let mut delay = esp_hal::delay::Delay::new(&clocks);

    println!("SW version {:?}", env!("CARGO_PKG_VERSION"));
    // Print wake or reset reason
    // TODO send the reset reason to Lora
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
    Output::new(io.pins.gpio0, Level::High);

    // Read HX711 value
    let hx711_raw_value: u32 = hx7111_read_value(io.pins.gpio21, io.pins.gpio20, delay);

    // Read vbat measure
    let analog_pin = io.pins.gpio1;
    let mut adc2_config = AdcConfig::new();
    let adc2_pin = adc2_config.enable_pin(analog_pin, Attenuation::Attenuation11dB);
    let adc2: Adc<esp_hal::peripherals::ADC2> = Adc::new(peripherals.ADC2, adc2_config);
    let vbat: u16 = read_vbat(adc2_pin, adc2);
    println!("ADC reading = {} mV", vbat);

    // Wifi Init
    let wifi_timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Wifi,
        wifi_timer,
        rng,
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    // Configure GPIO for SPI
    let sclk: GpioPin<10> = io.pins.gpio10;
    let miso: GpioPin<6> = io.pins.gpio6;
    let mosi: GpioPin<7> = io.pins.gpio7;
    let nss: GpioPin<8> = io.pins.gpio8;
    let reset: GpioPin<5> = io.pins.gpio5;
    let dio1: GpioPin<3> = io.pins.gpio3;
    let busy: GpioPin<4> = io.pins.gpio4;

    // Build Loraframe
    let mut lora_frame: [u8; LORA_FRAME_SIZE_BYTES] = [0; LORA_FRAME_SIZE_BYTES];
    lora_frame[1] = ((vbat - 3000) / 5) as u8;
    lora_frame[2] = ((hx711_raw_value >> 24) & 0xFF) as u8;
    lora_frame[3] = ((hx711_raw_value >> 16) & 0xFF) as u8;
    lora_frame[4] = ((hx711_raw_value >> 8) & 0xFF) as u8;

    // Scan wifi and add to the two strongest signals to Loraframe
    scan_wifi(init, wifi, &mut lora_frame[6..]);

    let spi_gpio = SpiGpio {
        sclk,
        miso,
        mosi,
        nss,
        reset,
        dio1,
        busy,
    };

    // Start the CLI task
    spawner.spawn(cli::cli_run(peripherals.USB_DEVICE)).ok();

    // Check if OTAA has been setup
    let mut flash = FlashStorage::new();
    let mut app_key = [0u8; 16];
    let mut otaa_is_set = false;
    flash
        .read(consts::NVS_APP_KEY_ADDRESS, &mut app_key)
        .unwrap();
    for &byte in app_key.iter() {
        // Check if all bytes are to default value (255 for a flash)
        if byte != 255 {
            otaa_is_set = true;
        }
    }
    if !otaa_is_set {
        println!("LoraWan cretentials has not beeen set, please use cli to set them");
        loop {
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    send_lorawan_msg(
        peripherals.SPI2,
        dma,
        rng,
        &clocks,
        spi_gpio,
        &mut lora_frame,
    )
    .await;

    println!("End of cycle, go to sleep");
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(10));
    Timer::after(Duration::from_millis(100)).await;
    rtc.sleep_deep(&[&timer], &mut delay);
}
