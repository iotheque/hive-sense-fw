use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use esp_hal::{
    analog::adc::{Adc, AdcConfig, Attenuation},
    gpio::{AnyInput, AnyOutput, GpioPin},
    prelude::nb,
};
use loadcell::{hx711, LoadCell};

const HX711_TARE: f32 = 198000.0;

async fn hx7111_read_value(
    hx711_dt: AnyInput<'static>,
    hx711_sck: AnyOutput<'static>,
    delay: esp_hal::delay::Delay,
    signal: &'static Signal<CriticalSectionRawMutex, u32>,
) {
    let mut hx7111_value: u32 = 0;
    let mut load_sensor = hx711::HX711::new(hx711_sck, hx711_dt, delay);
    load_sensor.set_scale(1.0);
    // Wait HX711 to be ready
    // Try 20 reads, get the third successful read
    let mut hx711_read_cnt: u8 = 0;
    for _ in 1..=20 {
        if load_sensor.is_ready() {
            match load_sensor.read_scaled() {
                Ok(x) => {
                    hx711_read_cnt += 1;
                    if hx711_read_cnt > 3 {
                        log::info!("HX711 raw reading = {:?}", x);
                        if x > HX711_TARE {
                            hx7111_value = (x - HX711_TARE) as u32;
                        } else {
                            hx7111_value = 0;
                        }
                        log::info!("HX711 reading = {:?}", hx7111_value);
                        break;
                    }
                }
                Err(e) => log::error!("Error reading HX711: {:?}", e),
            }
        }
        Timer::after(Duration::from_millis(50)).await;
        log::info!("Wait for HX711 available");
    }
    signal.signal(hx7111_value);
}

#[embassy_executor::task]
pub async fn hx7111_read_value_1(
    hx711_dt: AnyInput<'static>,
    hx711_sck: AnyOutput<'static>,
    delay: esp_hal::delay::Delay,
    signal: &'static Signal<CriticalSectionRawMutex, u32>,
) {
    log::info!("HX7111 number 1");
    hx7111_read_value(hx711_dt, hx711_sck, delay, signal).await;
}

#[embassy_executor::task]
pub async fn hx7111_read_value_2(
    hx711_dt: AnyInput<'static>,
    hx711_sck: AnyOutput<'static>,
    delay: esp_hal::delay::Delay,
    signal: &'static Signal<CriticalSectionRawMutex, u32>,
) {
    log::info!("HX7111 number 2");
    hx7111_read_value(hx711_dt, hx711_sck, delay, signal).await;
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
