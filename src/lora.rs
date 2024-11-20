use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use embedded_storage::ReadStorage;
use esp_hal::{
    clock::Clocks,
    dma::{self, Dma, DmaPriority},
    dma_descriptors,
    gpio::{self, AnyInput, AnyOutput, Level, Output, Pull},
    peripherals::SPI2,
    prelude::*,
    rng::Rng,
    spi::{
        master::{dma::SpiDma, prelude::*, Spi},
        FullDuplexMode, SpiMode,
    },
    Async,
};
use esp_storage::FlashStorage;
use lora_phy::{
    iv::GenericSx126xInterfaceVariant,
    lorawan_radio::LorawanRadio,
    sx126x::{self, Sx1262, Sx126x, TcxoCtrlVoltage},
    LoRa,
};
use lorawan_device::{
    async_device::{Device, EmbassyTimer, JoinResponse, SendResponse},
    default_crypto::DefaultFactory as Crypto,
    region, AppEui, AppKey, DevEui, JoinMode,
};
use static_cell::StaticCell;

use crate::{
    consts::{
        self, BSSIDS_TOTAL_SIZE, LORA_FRAME_SIZE_BYTES, MAX_TX_POWER, NVS_APP_EUI_ADDRESS,
        NVS_APP_KEY_ADDRESS, NVS_DEV_EUI_ADDRESS,
    },
    SpiGpio, IS_JOIN, SAVED_SESSION, SEED,
};

/// Join lorawan network then send a message if join is success
/// The session is saved in RTC RAM to be able to restore it after
/// deep sleep
pub async fn send_lorawan_msg(
    spi2: SPI2,
    dma: Dma<'_>,
    mut rng: Rng,
    clocks: &Clocks<'_>,
    spi_gpio: SpiGpio,
    data: &mut [u8; LORA_FRAME_SIZE_BYTES],
) {
    let dma_channel = dma.channel0;
    let (descriptors, rx_descriptors) = dma_descriptors!(32000);

    let spi = Spi::new(spi2, 200u32.kHz(), SpiMode::Mode0, clocks)
        .with_pins(
            Some(spi_gpio.sclk),
            Some(spi_gpio.mosi),
            Some(spi_gpio.miso),
            gpio::NO_PIN,
        )
        .with_dma(
            dma_channel.configure_for_async(false, DmaPriority::Priority0),
            descriptors,
            rx_descriptors,
        );

    static SPI_BUS: StaticCell<
        Mutex<NoopRawMutex, SpiDma<'_, SPI2, dma::Channel0, FullDuplexMode, Async>>,
    > = StaticCell::new();
    let spi_bus = Mutex::new(spi);
    let spi_bus = SPI_BUS.init(spi_bus);
    let cs = Output::new(spi_gpio.nss, Level::High);
    let spi_dev1 = SpiDevice::new(spi_bus, cs);

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
    let lora = LoRa::new(Sx126x::new(spi_dev1, iv, config), true, Delay)
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
        log::info!("Ask to join lora network");
        let mut flash = FlashStorage::new();
        let mut dev_eui = [0u8; 8];
        flash.read(NVS_DEV_EUI_ADDRESS, &mut dev_eui).unwrap();
        let mut app_eui = [0u8; 8];
        flash.read(NVS_APP_EUI_ADDRESS, &mut app_eui).unwrap();
        let mut app_key = [0u8; 16];
        flash.read(NVS_APP_KEY_ADDRESS, &mut app_key).unwrap();

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
            log::info!("LoRaWAN network joined");
            let send_status = device.send(data, 1, false).await.unwrap();
            match send_status {
                SendResponse::RxComplete | SendResponse::DownlinkReceived(0) => {
                    log::info!("LoRaWAN send succes");
                    unsafe {
                        SEED = seed;
                        IS_JOIN = true;
                        SAVED_SESSION = device.get_session().cloned();
                    }
                }
                _ => {
                    log::error!("LoRaWAN send error, reset session : {:?}", send_status);
                    unsafe { IS_JOIN = false };
                }
            }
        } else {
            // Save state in RTC RAM
            unsafe { IS_JOIN = false };
            log::info!("CAN NOT join LoRaWAN network {:?}", resp);
        }
    } else {
        log::info!("We are already joined use saved session");
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
                log::debug!("send_status {:?}", send_status);

                match send_status {
                    SendResponse::RxComplete => {
                        log::info!("LoRaWAN send succes");
                        SAVED_SESSION = device.get_session().cloned();
                    }
                    _ => {
                        log::error!("LoRaWAN send error, reset session");
                        IS_JOIN = false;
                    }
                }
            }
        }
    }
}

pub fn lorawan_build_msg(
    vbat: u16,
    mut hx711_raw_value: u32,
    wifi_data: [u8; BSSIDS_TOTAL_SIZE],
) -> [u8; LORA_FRAME_SIZE_BYTES] {
    let mut lora_frame: [u8; LORA_FRAME_SIZE_BYTES] = [0; LORA_FRAME_SIZE_BYTES];
    if vbat <= 3000 {
        lora_frame[1] = 0;
    } else {
        lora_frame[1] = ((vbat - 3000) / 5) as u8;
    }
    if hx711_raw_value > 0xFFFFFF {
        hx711_raw_value = 0xFFFFFF;
    }
    lora_frame[2] = ((hx711_raw_value >> 16) & 0xFF) as u8;
    lora_frame[3] = ((hx711_raw_value >> 8) & 0xFF) as u8;
    lora_frame[4] = ((hx711_raw_value) & 0xFF) as u8;
    lora_frame[5..].copy_from_slice(&wifi_data);

    log::info!("LoraWan Frame is {:?}", lora_frame);

    lora_frame
}

pub fn lorawan_otaa_is_configured() -> bool {
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
    otaa_is_set
}
