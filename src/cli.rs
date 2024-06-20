use core::convert::Infallible;

use embassy_time::{Duration, Timer};
use embedded_cli::cli::CliBuilder;
use embedded_cli::Command;
use esp_hal::{
    peripherals::USB_DEVICE,
    usb_serial_jtag::UsbSerialJtag,
};
use ufmt::uwrite;

#[derive(Command)]
enum Base<'a> {
    /// Say hello to World or someone else
    Hello {
        /// To whom to say hello (World by default)
        name: Option<&'a str>,
    },

    /// Stop CLI and exit
    Exit,
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

#[embassy_executor::task]
pub async fn cli_run(usb_periph: USB_DEVICE) {
    let usb_serial: UsbSerialJtag<esp_hal::Blocking> = UsbSerialJtag::new(usb_periph, None);
    let (tx, mut rx) = usb_serial.split();
    let writer = Writer(tx);

    // create static buffers for use in cli (so we're not using stack memory)
    // History buffer is 1 byte longer so max command fits in it (it requires extra byte at end)
    // SAFETY: buffers are passed to cli and are used by cli only
    let (command_buffer, history_buffer) = unsafe {
        static mut COMMAND_BUFFER: [u8; 32] = [0; 32];
        static mut HISTORY_BUFFER: [u8; 32] = [0; 32];
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
                        Base::Hello { name } => {
                            uwrite!(cli.writer(), "Hello, {}", name.unwrap_or("World"))?;
                        }
                        Base::Exit => {
                            uwrite!(cli.writer(), "Cli can't shutdown now")?;
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
