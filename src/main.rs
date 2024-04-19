#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use esp_backtrace as _;
use esp_hal::{
    clock::ClockControl,
    dma::*,
    dma_descriptors, embassy,
    gpio::{self, IO},
    peripherals::Peripherals,
    prelude::*,
    spi::{
        master::{prelude::*, Spi},
        SpiMode,
    },
    timer::TimerGroup,
    Rng,
};
use lora_phy::iv::GenericSx126xInterfaceVariant;
use lora_phy::lorawan_radio::LorawanRadio;
use lora_phy::sx1261_2::{self, Sx126xVariant, TcxoCtrlVoltage, SX1261_2};
use lora_phy::LoRa;
use lorawan_device::{async_device::JoinResponse, default_crypto::DefaultFactory as Crypto};
use lorawan_device::{
    async_device::{region, Device, EmbassyTimer, JoinMode},
    AppEui, AppKey, DevEui,
};

// warning: set these appropriately for the region
const LORAWAN_REGION: region::Region = region::Region::EU868;
const MAX_TX_POWER: u8 = 0;

#[main]
async fn main(_spawner: Spawner) -> ! {
    // Configure peripherals
    let peripherals = Peripherals::take();
    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();
    let timg0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timg0);
    let dma = Dma::new(peripherals.DMA);
    let dma_channel = dma.channel0;
    let (mut descriptors, mut rx_descriptors) = dma_descriptors!(32000);

    // Configure SPI
    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    let sclk = io.pins.gpio10;
    let miso = io.pins.gpio6;
    let mosi = io.pins.gpio7;
    let nss = io.pins.gpio8.into_open_drain_output();
    let reset = io.pins.gpio5.into_open_drain_output();
    let dio1 = io.pins.gpio3.into_pull_down_input().degrade();
    let busy = io.pins.gpio4.into_pull_down_input().degrade();

    let mut spi_bus = Spi::new(peripherals.SPI2, 200u32.kHz(), SpiMode::Mode0, &clocks)
        .with_pins(
            Some(sclk),
            Some(mosi),
            Some(miso),
            gpio::NO_PIN,
        )
        .with_dma(dma_channel.configure(
            false,
            &mut descriptors,
            &mut rx_descriptors,
            DmaPriority::Priority0,
        ));
    let spi = ExclusiveDevice::new(&mut spi_bus, nss, Delay);

    // Configure Sx1262 chip
    let config = sx1261_2::Config {
        chip: Sx126xVariant::Sx1262,
        tcxo_ctrl: Some(TcxoCtrlVoltage::Ctrl1V7),
        use_dcdc: false,
        use_dio2_as_rfswitch: true,
    };
    let iv = GenericSx126xInterfaceVariant::new(reset, dio1, busy, None, None).unwrap();
    let lora = LoRa::new(SX1261_2::new(spi, iv, config), true, Delay)
        .await
        .unwrap();
    let radio: LorawanRadio<_, _, MAX_TX_POWER> = lora.into();
    let region: region::Configuration = region::Configuration::new(LORAWAN_REGION);
    let mut device: Device<_, Crypto, _, _> = Device::new(
        region,
        radio,
        EmbassyTimer::new(),
        Rng::new(peripherals.RNG),
    );

    // Join to Lora Network
    let resp = device
        .join(&JoinMode::OTAA {
            deveui: DevEui::from([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            appeui: AppEui::from([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]),
            appkey: AppKey::from([
                0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00,
            ]),
        })
        .await;
    if let Ok(JoinResponse::JoinSuccess) = resp {
        esp_println::println!("LoRaWAN network joined, send message");
        let data_test: [u8; 5] = [1, 2, 3, 4, 5];
        let send_status = device.send(&data_test, 1, true).await.unwrap();
        esp_println::println!("Send data status: {:?}", send_status);
    } else {
        esp_println::println!("CAN NOT join LoRaWAN network");
    }

}
