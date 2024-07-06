#![no_std]
#![no_main]
mod cli;
mod consts;
mod lora;
mod sensors;
mod wifi;
use consts::BSSIDS_TOTAL_SIZE;
use cycle::{cycle_end, cycle_start};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use embedded_storage::ReadStorage;
use esp_backtrace as _;
use esp_hal::{
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
use lora::{lorawan_build_msg, lorawan_otaa_is_configured};
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

// Signals used to synchonize embassy tasks and get data
static WIFI_END_SIGN: Signal<CriticalSectionRawMutex, [u8; BSSIDS_TOTAL_SIZE]> =
    Signal::<CriticalSectionRawMutex, [u8; BSSIDS_TOTAL_SIZE]>::new();
static VBATT_END_SIGN: Signal<CriticalSectionRawMutex, u16> =
    Signal::<CriticalSectionRawMutex, u16>::new();
static HX711_END_SIGN: Signal<CriticalSectionRawMutex, u32> =
    Signal::<CriticalSectionRawMutex, u32>::new();

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

    // Wifi Init peripheral
    let wifi_timer = SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let wifi_handler = initialize(
        EspWifiInitFor::Wifi,
        wifi_timer,
        rng,
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    // Start the CLI task
    spawner.spawn(cli::cli_run(peripherals.USB_DEVICE)).ok();

    // Start tasks that gather all data
    spawner
        .spawn(wifi::scan_wifi(wifi_handler, wifi, &WIFI_END_SIGN))
        .ok();
    spawner
        .spawn(sensors::read_vbat(
            io.pins.gpio1,
            peripherals.ADC2,
            &VBATT_END_SIGN,
        ))
        .ok();
    spawner
        .spawn(sensors::hx7111_read_value(
            io.pins.gpio21,
            io.pins.gpio20,
            delay,
            &HX711_END_SIGN,
        ))
        .ok();

    // Wait for all needed data
    let wifi_data: [u8; BSSIDS_TOTAL_SIZE] = WIFI_END_SIGN.wait().await;
    let vbat: u16 = VBATT_END_SIGN.wait().await;
    let hx711_raw_value: u32 = HX711_END_SIGN.wait().await;

    // Build Loraframe
    let mut lora_frame = lorawan_build_msg(vbat, hx711_raw_value, wifi_data);

    // Configure GPIO for SPI
    let spi_gpio = SpiGpio {
        sclk: io.pins.gpio10,
        miso: io.pins.gpio6,
        mosi: io.pins.gpio7,
        nss: io.pins.gpio8,
        reset: io.pins.gpio5,
        dio1: io.pins.gpio3,
        busy: io.pins.gpio4,
    };

    // Check if OTAA has been setup
    if !lorawan_otaa_is_configured() {
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
    let mut flash = FlashStorage::new();
    flash
        .read(consts::NVS_WAKEUP_PERIOD_ADDRESS, &mut wake_period_raw)
        .unwrap();
    let wake_period_s: u16 = u16::from_le_bytes(wake_period_raw);

    log::info!("Next wakeup in {:?} s", wake_period_s);
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(wake_period_s.into()));
    Timer::after(Duration::from_millis(100)).await;
    rtc.sleep_deep(&[&timer], &mut delay);
}
