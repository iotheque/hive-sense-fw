use core::convert::Infallible;

use crate::consts::{
    NVS_APP_EUI_ADDRESS, NVS_APP_KEY_ADDRESS, NVS_DEV_EUI_ADDRESS, NVS_WAKEUP_PERIOD_ADDRESS,
};
use embassy_time::{Duration, Timer};
use embedded_cli::cli::{CliBuilder, CliHandle};
use embedded_cli::Command;
use embedded_storage::{ReadStorage, Storage};
use esp_hal::{
    efuse::Efuse, peripherals::USB_DEVICE, reset::software_reset, usb_serial_jtag::UsbSerialJtag,
};
use esp_storage::FlashStorage;
use ufmt::uwrite;

const EUI_SIZE_BYTES: usize = 8;
const EUI_CHAR_NB: usize = 2 * EUI_SIZE_BYTES; // Each byte is sent as two characters
const OTAA_SIZE_BYTES: usize = 16;
const OTAA_CHAR_NB: usize = 2 * OTAA_SIZE_BYTES; // Each byte is sent as two characters

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

    /// Configure wakeup period in seconds
    WakeUp {
        #[command(subcommand)]
        command: WakeUpCommand,
    },

    /// Get the device MAC address
    GetMac,

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

#[derive(Debug, Command)]
enum WakeUpCommand {
    /// Get current WakeUp value
    Get,

    /// Set WakeUp value
    Set {
        /// WakeUp value in seconds
        value: u16,
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

fn eui_to_hex_bytes(eui: &str) -> [u8; EUI_SIZE_BYTES] {
    let mut bytes = [0u8; EUI_SIZE_BYTES];

    for i in 0..EUI_SIZE_BYTES {
        let byte_str = &eui[2 * i..2 * i + 2];
        bytes[i] = u8::from_str_radix(byte_str, 16).unwrap();
    }
    bytes
}

fn otaa_to_hex_bytes(otaa: &str) -> [u8; OTAA_SIZE_BYTES] {
    let mut bytes = [0u8; OTAA_SIZE_BYTES];

    for i in 0..OTAA_SIZE_BYTES {
        let byte_str = &otaa[2 * i..2 * i + 2];
        bytes[i] = u8::from_str_radix(byte_str, 16).unwrap();
    }
    bytes
}

fn handle_eui_command(
    cli: &mut CliHandle<'_, Writer, Infallible>,
    flash: &mut FlashStorage,
    address: u32,
    command: EuiCommand,
) -> Result<(), Infallible> {
    let mut eui = [0u8; EUI_SIZE_BYTES];
    match command {
        EuiCommand::Get => {
            flash.read(address, &mut eui).unwrap();
            uwrite!(cli.writer(), "Current EUI is {:?}", &eui[..EUI_SIZE_BYTES])?;
        }
        EuiCommand::Set { value } => {
            if value.len() != EUI_CHAR_NB {
                uwrite!(
                    cli.writer(),
                    "Invalid size {:?}, expected {:?} ",
                    value.len(),
                    EUI_CHAR_NB
                )?;
            } else {
                flash
                    .write(address, &eui_to_hex_bytes(value)[..EUI_SIZE_BYTES])
                    .unwrap();
                uwrite!(cli.writer(), "Set EUI is {:?}", eui_to_hex_bytes(value))?;
            }
        }
    }
    Ok(())
}

fn handle_app_key_command(
    cli: &mut CliHandle<'_, Writer, Infallible>,
    flash: &mut FlashStorage,
    address: u32,
    command: AppKeyCommand,
) -> Result<(), Infallible> {
    let mut app_key = [0u8; OTAA_SIZE_BYTES];
    match command {
        AppKeyCommand::Get => {
            flash.read(address, &mut app_key).unwrap();
            uwrite!(cli.writer(), "Current AppKey is {:?}", &app_key[..16])?;
        }
        AppKeyCommand::Set { value } => {
            if value.len() != OTAA_CHAR_NB {
                uwrite!(
                    cli.writer(),
                    "Invalid size {:?}, expected {:?} ",
                    value.len(),
                    OTAA_CHAR_NB
                )?;
            } else {
                flash
                    .write(address, &otaa_to_hex_bytes(value)[..OTAA_SIZE_BYTES])
                    .unwrap();
                uwrite!(cli.writer(), "Set AppKey is {:?}", otaa_to_hex_bytes(value))?;
            }
        }
    }
    Ok(())
}

fn handle_wakeup_command(
    cli: &mut CliHandle<'_, Writer, Infallible>,
    flash: &mut FlashStorage,
    address: u32,
    command: WakeUpCommand,
) -> Result<(), Infallible> {
    let mut raw_value = [0u8; 2];
    match command {
        WakeUpCommand::Get => {
            flash.read(address, &mut raw_value).unwrap();
            uwrite!(
                cli.writer(),
                "Current WakeUp is {:?}",
                u16::from_be_bytes(raw_value)
            )?;
        }
        WakeUpCommand::Set { value } => {
            flash.write(address, &value.to_be_bytes()).unwrap();
            uwrite!(cli.writer(), "Set WakeUp is {:?}", value)?;
        }
    }
    Ok(())
}

fn handle_get_mac_command(cli: &mut CliHandle<'_, Writer, Infallible>) -> Result<(), Infallible> {
    let mac: [u8; 6] = Efuse::get_mac_address();

    uwrite!(cli.writer(), "Chip Mac address: ").unwrap();
    for &element in &mac {
        uwrite!(cli.writer(), "{:x}", element).unwrap();
    }

    Ok(())
}

fn handle_reset_command(cli: &mut CliHandle<'_, Writer, Infallible>) -> Result<(), Infallible> {
    uwrite!(cli.writer(), "Reset now !")?;
    software_reset();
    Ok(())
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
                    let _ = match command {
                        Base::DevEui { command } => {
                            handle_eui_command(cli, &mut flash, NVS_DEV_EUI_ADDRESS, command)
                        }
                        Base::AppEui { command } => {
                            handle_eui_command(cli, &mut flash, NVS_APP_EUI_ADDRESS, command)
                        }
                        Base::AppKey { command } => {
                            handle_app_key_command(cli, &mut flash, NVS_APP_KEY_ADDRESS, command)
                        }
                        Base::WakeUp { command } => handle_wakeup_command(
                            cli,
                            &mut flash,
                            NVS_WAKEUP_PERIOD_ADDRESS,
                            command,
                        ),
                        Base::GetMac => handle_get_mac_command(cli),
                        Base::Reset => handle_reset_command(cli),
                    };
                    Ok(())
                }),
            );
        }
        // Wait 1ms between each byte read, bytes are buffered by peripheral
        Timer::after(Duration::from_millis(1)).await;
    }
}
