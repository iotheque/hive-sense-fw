#![no_std]
#![no_main]
mod cli;
mod consts;
mod cycle;
mod lora;
mod sensors;
mod wifi;
use consts::BSSIDS_TOTAL_SIZE;
use cycle::{cycle_end, cycle_start};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::*,
    gpio::{AnyInput, AnyOutput, GpioPin, Io, Level, Output, Pull},
    peripherals::Peripherals,
    prelude::*,
    rng::Rng,
    rtc_cntl::Rtc,
    system::SystemControl,
    timer::{timg::TimerGroup, ErasedTimer, OneShotTimer, PeriodicTimer},
};
use esp_wifi::{initialize, EspWifiInitFor};
use lora::{lorawan_build_msg, lorawan_otaa_is_configured};
use lorawan_device::mac::Session;

// When you are okay with using a nightly compiler it's better to use https://docs.rs/static_cell/2.1.0/static_cell/macro.make_static.html
macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

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
    sclk: GpioPin<7>,
    miso: GpioPin<6>,
    mosi: GpioPin<5>,
    nss: GpioPin<16>,
    reset: GpioPin<8>,
    dio1: GpioPin<18>,
    busy: GpioPin<15>,
}

// Signals used to synchonize embassy tasks and get data
static WIFI_END_SIGN: Signal<CriticalSectionRawMutex, [u8; BSSIDS_TOTAL_SIZE]> =
    Signal::<CriticalSectionRawMutex, [u8; BSSIDS_TOTAL_SIZE]>::new();
static VBATT_END_SIGN: Signal<CriticalSectionRawMutex, u16> =
    Signal::<CriticalSectionRawMutex, u16>::new();
static HX711_END_SIGN_1: Signal<CriticalSectionRawMutex, u32> =
    Signal::<CriticalSectionRawMutex, u32>::new();
static HX711_END_SIGN_2: Signal<CriticalSectionRawMutex, u32> =
    Signal::<CriticalSectionRawMutex, u32>::new();

#[main]
async fn main(spawner: Spawner) {
    // Configure peripherals
    let peripherals = Peripherals::take();
    let system = SystemControl::new(peripherals.SYSTEM);
    let clocks = ClockControl::max(system.clock_control).freeze();
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks, None);
    let timer0 = OneShotTimer::new(timg0.timer0.into());
    let timers = [timer0];
    let timers = mk_static!([OneShotTimer<ErasedTimer>; 1], timers);
    esp_hal_embassy::init(&clocks, timers);
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);
    let rtc = Rtc::new(peripherals.LPWR, None);
    let rng: Rng = Rng::new(peripherals.RNG);
    let wifi = peripherals.WIFI;
    let dma: Dma = Dma::new(peripherals.DMA);
    let delay = esp_hal::delay::Delay::new(&clocks);

    // Create the logger
    esp_println::logger::init_logger_from_env();

    cycle_start();

    // Enable HX711 and ADC power
    Output::new(io.pins.gpio2, Level::High);

    // Wifi Init peripheral
    let wifi_timer = PeriodicTimer::new(
        esp_hal::timer::timg::TimerGroup::new(peripherals.TIMG1, &clocks, None)
            .timer1
            .into(),
    );
    let wifi_handler = initialize(
        EspWifiInitFor::Wifi,
        wifi_timer,
        rng,
        peripherals.RADIO_CLK,
        &clocks,
    )
    .unwrap();

    // Start tasks that gather all data
    spawner
        .spawn(wifi::scan_wifi(wifi_handler, wifi, &WIFI_END_SIGN))
        .ok();
    spawner
        .spawn(sensors::read_vbat(
            io.pins.gpio1,
            peripherals.ADC1,
            &VBATT_END_SIGN,
        ))
        .ok();
    // HX711 on J6
    let hx711_dt_1 = AnyInput::new(io.pins.gpio39, Pull::None);
    let io_hx711_sck_1 = AnyOutput::new(io.pins.gpio38, Level::Low);
    spawner
        .spawn(sensors::hx7111_read_value_1(
            hx711_dt_1,
            io_hx711_sck_1,
            delay,
            &HX711_END_SIGN_1,
        ))
        .ok();
    // HX711 on J5
    let hx711_dt_2 = AnyInput::new(io.pins.gpio43, Pull::None);
    let io_hx711_sck_2 = AnyOutput::new(io.pins.gpio44, Level::Low);
    spawner
        .spawn(sensors::hx7111_read_value_2(
            hx711_dt_2,
            io_hx711_sck_2,
            delay,
            &HX711_END_SIGN_2,
        ))
        .ok();

    // Check if OTAA has been setup
    if !lorawan_otaa_is_configured() {
        // Start the CLI task
        spawner.spawn(cli::cli_run(peripherals.USB_DEVICE)).ok();

        log::info!("LoraWan credentials has not beeen set, please use cli to set them");
        loop {
            Timer::after(Duration::from_millis(100)).await;
        }
    }

    // Wait for all needed data
    let wifi_data: [u8; BSSIDS_TOTAL_SIZE] = WIFI_END_SIGN.wait().await;
    let vbat: u16 = VBATT_END_SIGN.wait().await;
    let hx711_raw_value_1: u32 = HX711_END_SIGN_1.wait().await;
    let hx711_raw_value_2: u32 = HX711_END_SIGN_2.wait().await;

    log::info!("hx711_raw_value_1 {:?}", hx711_raw_value_1);
    log::info!("hx711_raw_value_2 {:?}", hx711_raw_value_2);

    // Build Loraframe
    let mut lora_frame = lorawan_build_msg(vbat, hx711_raw_value_1, wifi_data);

    // Configure GPIO for SPI
    let spi_gpio = SpiGpio {
        sclk: io.pins.gpio7,
        miso: io.pins.gpio6,
        mosi: io.pins.gpio5,
        nss: io.pins.gpio16,
        reset: io.pins.gpio8,
        dio1: io.pins.gpio18,
        busy: io.pins.gpio15,
    };

    lora::send_lorawan_msg(
        peripherals.SPI2,
        dma,
        rng,
        &clocks,
        spi_gpio,
        &mut lora_frame,
    )
    .await;

    cycle_end(rtc).await;
}
