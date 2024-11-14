#![no_std]
#![no_main]
mod cli;
mod consts;
mod lora;
mod sensors;
mod wifi;
use consts::LORA_FRAME_SIZE_BYTES;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use embedded_storage::ReadStorage;
use esp_backtrace as _;
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    clock::ClockControl,
    dma::*,
    gpio::{GpioPin, Io, Level, Output},
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    rtc_cntl::{get_reset_reason, get_wakeup_cause, sleep::TimerWakeupSource, Rtc, SocResetReason},
    system::SystemControl,
    timer::{systimer::SystemTimer, timg::TimerGroup},
    Cpu,
};
use esp_storage::FlashStorage;
use esp_wifi::{initialize, EspWifiInitFor};
use lorawan_device::mac::Session;

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

    // Create the logger
    esp_println::logger::init_logger_from_env();

    log::info!("SW version {:?}", env!("CARGO_PKG_VERSION"));
    // TODO send the reset reason to Lora
    let reason = get_reset_reason(Cpu::ProCpu).unwrap_or(SocResetReason::ChipPowerOn);
    log::info!("Reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    log::info!("Wake reason: {:?}", wake_reason);

    unsafe {
        log::info!("BOOT_CNT {:x?}", BOOT_CNT);
        BOOT_CNT += 1;
        log::debug!("IS_JOIN: {:?}", IS_JOIN);
    }

    // Enable HX711 and ADC power
    Output::new(io.pins.gpio0, Level::High);

    // Read HX711 value
    let hx711_raw_value: u32 = sensors::hx7111_read_value(io.pins.gpio21, io.pins.gpio20, delay);

    // Read vbat measure
    let analog_pin = io.pins.gpio1;
    let mut adc2_config = AdcConfig::new();
    let adc2_pin = adc2_config.enable_pin(analog_pin, Attenuation::Attenuation11dB);
    let adc2: Adc<esp_hal::peripherals::ADC2> = Adc::new(peripherals.ADC2, adc2_config);
    let vbat: u16 = sensors::read_vbat(adc2_pin, adc2);
    log::info!("ADC reading = {} mV", vbat);

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
    wifi::scan_wifi(init, wifi, &mut lora_frame[6..]);

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
        log::info!("LoraWan credentials has not beeen set, please use cli to set them");
        loop {
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    lora::send_lorawan_msg(
        peripherals.SPI2,
        dma,
        rng,
        &clocks,
        spi_gpio,
        &mut lora_frame,
    )
    .await;

    log::info!("End of cycle, go to sleep");
    let mut wake_period_raw = [0u8; 2];
    flash
        .read(consts::NVS_WAKEUP_PERIOD_ADDRESS, &mut wake_period_raw)
        .unwrap();
    let wake_period_s: u16 = u16::from_le_bytes(wake_period_raw);

    log::info!("Next wakeup in {:?} s", wake_period_s);
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(wake_period_s.into()));
    Timer::after(Duration::from_millis(100)).await;
    rtc.sleep_deep(&[&timer], &mut delay);
}
