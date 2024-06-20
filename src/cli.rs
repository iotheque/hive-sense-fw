//mod consts;
use core::convert::Infallible;

use crate::consts::{NVS_APP_EUI_ADDRESS, NVS_APP_KEY_ADDRESS, NVS_DEV_EUI_ADDRESS};
use embassy_time::{Duration, Timer};
use embedded_cli::cli::CliBuilder;
use embedded_cli::Command;
use embedded_storage::{ReadStorage, Storage};
use esp_hal::{peripherals::USB_DEVICE, reset::software_reset, usb_serial_jtag::UsbSerialJtag};
use esp_storage::FlashStorage;
use ufmt::uwrite;

#[derive(Command)]
enum Base<'a> {
    /// Access to LoraWan DevEui info
    DevEui {
        #[command(subcommand)]
        command: EuiCommand<'a>,
    },

    /// Access to LoraWan AppEui info
    AppEui {
        #[command(subcommand)]
        command: EuiCommand<'a>,
    },

    /// Access to LoraWan AppKey info
    AppKey {
        #[command(subcommand)]
        command: AppKeyCommand<'a>,
    },

    /// Reset the system
    Reset,
}

#[derive(Debug, Command)]
enum EuiCommand<'a> {
    /// Get current EUI value
    Get,

    /// Set EUI value
    Set {
        /// EUI value
        value: &'a str,
    },
}

#[derive(Debug, Command)]
enum AppKeyCommand<'a> {
    /// Get current AppKey value
    Get,

    /// Set AppKey value
    Set {
        /// AppKey value
        value: &'a str,
    },
}

/// Wrapper around usart so we can impl embedded_io::Write
/// which is required for cli
struct Writer(esp_hal::usb_serial_jtag::UsbSerialJtagTx<'static, esp_hal::Blocking>);

impl embedded_io::ErrorType for Writer {
    type Error = Infallible;
}

impl embedded_io::Write for Writer {
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        let size = self.0.write(buf).unwrap();
        Ok(size)
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

fn eui_to_hex_bytes(eui: &str) -> [u8; 8] {
    let mut bytes = [0u8; 8];

    for i in 0..8 {
        let byte_str = &eui[2 * i..2 * i + 2];
        bytes[i] = u8::from_str_radix(byte_str, 16).unwrap();
    }
    bytes
}

fn otaa_to_hex_bytes(otaa: &str) -> [u8; 16] {
    let mut bytes = [0u8; 16];

    for i in 0..16 {
        let byte_str = &otaa[2 * i..2 * i + 2];
        bytes[i] = u8::from_str_radix(byte_str, 16).unwrap();
    }
    bytes
}

#[embassy_executor::task]
pub async fn cli_run(usb_periph: USB_DEVICE) {
    let usb_serial: UsbSerialJtag<esp_hal::Blocking> = UsbSerialJtag::new(usb_periph, None);
    let (tx, mut rx) = usb_serial.split();
    let writer = Writer(tx);

    let mut flash = FlashStorage::new();

    // create static buffers for use in cli (so we're not using stack memory)
    // History buffer is 1 byte longer so max command fits in it (it requires extra byte at end)
    // SAFETY: buffers are passed to cli and are used by cli only
    let (command_buffer, history_buffer) = unsafe {
        static mut COMMAND_BUFFER: [u8; 64] = [0; 64];
        static mut HISTORY_BUFFER: [u8; 64] = [0; 64];
        (COMMAND_BUFFER.as_mut(), HISTORY_BUFFER.as_mut())
    };
    let mut cli = CliBuilder::default()
        .writer(writer)
        .command_buffer(command_buffer)
        .history_buffer(history_buffer)
        .build()
        .ok()
        .unwrap();

    loop {
        // Read next byte
        let result: Result<u8, esp_hal::prelude::nb::Error<Infallible>> = rx.read_byte();

        if let Ok(value) = result {
            let _ = cli.process_byte::<Base, _>(
                value,
                &mut Base::processor(|cli, command| {
                    match command {
                        Base::DevEui { command } => {
                            let mut dev_eui = [0u8; 8];
                            match command {
                                EuiCommand::Get => {
                                    flash.read(NVS_DEV_EUI_ADDRESS, &mut dev_eui).unwrap();
                                    uwrite!(cli.writer(), "Current DevEui is {:?}", &dev_eui[..8])?;
                                }
                                EuiCommand::Set { value } => {
                                    flash
                                        .write(NVS_DEV_EUI_ADDRESS, &eui_to_hex_bytes(value)[..8])
                                        .unwrap();
                                    uwrite!(
                                        cli.writer(),
                                        "Set DevEui is {:?}",
                                        eui_to_hex_bytes(value)
                                    )?;
                                }
                            }
                        }
                        Base::AppEui { command } => {
                            let mut app_eui = [0u8; 8];
                            match command {
                                EuiCommand::Get => {
                                    flash.read(NVS_APP_EUI_ADDRESS, &mut app_eui).unwrap();
                                    uwrite!(cli.writer(), "Current AppEui is {:?}", &app_eui[..8])?;
                                }
                                EuiCommand::Set { value } => {
                                    flash
                                        .write(NVS_APP_EUI_ADDRESS, &eui_to_hex_bytes(value)[..8])
                                        .unwrap();
                                    uwrite!(
                                        cli.writer(),
                                        "Set AppEui is {:?}",
                                        eui_to_hex_bytes(value)
                                    )?;
                                }
                            }
                        }
                        Base::AppKey { command } => {
                            let mut app_key = [0u8; 16];
                            match command {
                                AppKeyCommand::Get => {
                                    flash.read(NVS_APP_KEY_ADDRESS, &mut app_key).unwrap();
                                    uwrite!(
                                        cli.writer(),
                                        "Current AppKey is {:?}",
                                        &app_key[..16]
                                    )?;
                                }
                                AppKeyCommand::Set { value } => {
                                    flash
                                        .write(NVS_APP_KEY_ADDRESS, &otaa_to_hex_bytes(value)[..16])
                                        .unwrap();
                                    uwrite!(
                                        cli.writer(),
                                        "Set AppKey is {:?}",
                                        otaa_to_hex_bytes(value)
                                    )?;
                                }
                            }
                        }
                        Base::Reset => {
                            uwrite!(cli.writer(), "Reset now !")?;
                            software_reset();
                        }
                    }
                    Ok(())
                }),
            );
        }
        // Wait 10ms between each byte read, bytes are buffered by peripheral
        Timer::after(Duration::from_millis(10)).await;
    }
}
