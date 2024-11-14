use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    gpio::{GpioPin, Input, Level, Output, Pull},
    prelude::nb,
};
use loadcell::{hx711, LoadCell};

#[embassy_executor::task]
pub async fn hx7111_read_value(
    hx711_dt: GpioPin<39>,
    hx711_sck: GpioPin<38>,
    delay: esp_hal::delay::Delay,
    signal: &'static Signal<CriticalSectionRawMutex, u32>,
) {
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
            match load_sensor.read_scaled() {
                Ok(x) => {
                    hx7111_value = x as u32;
                    log::info!("HX711 reading = {:?}", x);
                    break;
                }
                Err(e) => log::error!("Error reading HX711: {:?}", e),
            }
        }
        delay.delay_millis(100u32);
        log::debug!("Wait for HX711 available");
    }

    signal.signal(hx7111_value);
}

#[embassy_executor::task]
pub async fn read_vbat(
    analog_pin: GpioPin<1>,
    adc1_periph: esp_hal::peripherals::ADC1,
    signal: &'static Signal<CriticalSectionRawMutex, u16>,
) {
    type AdcCal = esp_hal::analog::adc::AdcCalBasic<esp_hal::peripherals::ADC1>;
    let mut adc1_config = AdcConfig::new();
    let mut adc1_pin =
        adc1_config.enable_pin_with_cal::<_, AdcCal>(analog_pin, Attenuation::Attenuation11dB);
    let mut adc1 = Adc::new(adc1_periph, adc1_config);

    // Read vbatt from ADC2 GPIO1 and returns the value in mV
    // Hardware gain is 56/156
    let mut raw_value: u32 = 0;
    for _ in 0..=10 {
        let read = nb::block!(adc1.read_oneshot(&mut adc1_pin)).unwrap();
        raw_value += read as u32;
        Timer::after(Duration::from_millis(50)).await;
    }

    let vbat = (raw_value * 156 / 56 / 10) as u16;
    log::info!("Battery voltage = {} mV", vbat);
    signal.signal(vbat);
}
