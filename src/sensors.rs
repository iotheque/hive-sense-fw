use esp_hal::{
    analog::adc::Adc,
    gpio::{GpioPin, Input, Level, Output, Pull},
    prelude::nb,
};
use loadcell::{hx711, LoadCell};

/// Read hx7111
pub fn hx7111_read_value(
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

    hx7111_value
}

/// Read vbatt from ADC2 GPIO1 and returns the value in mV
/// Hardware gain is 0.5
pub fn read_vbat(
    mut adc2_pin: esp_hal::analog::adc::AdcPin<GpioPin<1>, esp_hal::peripherals::ADC2>,
    mut adc2: Adc<esp_hal::peripherals::ADC2>,
) -> u16 {
    // Read vbat
    let raw_value: u16 = nb::block!(adc2.read_oneshot(&mut adc2_pin)).unwrap();
    raw_value * 2
}
