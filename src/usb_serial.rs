use defmt;
use embassy_futures::select::{select, Either};
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::{bind_interrupts, Peri};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{with_timeout, Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use heapless::String;

use crate::constants::{buffers, timing};

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

/// Message types that can be sent to the USB serial output
#[derive(Clone, Debug)]
pub enum SerialMessage {
    /// Simple text message
    Text(&'static str),
    /// Formatted ADC reading with channel and value
    AdcReading { channel: u8, value: f32 },
    /// Custom formatted message with heapless string
    Formatted(String<{ buffers::MESSAGE_BUFFER_SIZE }>),
}

/// Global channel for sending messages to USB serial from anywhere in the code
pub static SERIAL_CHANNEL: Channel<
    ThreadModeRawMutex,
    SerialMessage,
    { buffers::SERIAL_CHANNEL_DEPTH },
> = Channel::new();

/// Get a sender to write messages to USB serial from other modules
pub fn get_serial_sender(
) -> Sender<'static, ThreadModeRawMutex, SerialMessage, { buffers::SERIAL_CHANNEL_DEPTH }> {
    SERIAL_CHANNEL.sender()
}

/// Send a simple text message to USB serial (convenience function)
pub async fn send_message(message: &'static str) {
    let sender = get_serial_sender();
    let _ = sender.send(SerialMessage::Text(message)).await;
}

/// Send an ADC reading to USB serial (convenience function)
pub async fn send_adc_reading(channel: u8, value: f32) {
    let sender = get_serial_sender();
    let _ = sender
        .send(SerialMessage::AdcReading { channel, value })
        .await;
}

/// Send a formatted message to USB serial (convenience function)
pub async fn send_formatted_message(message: String<{ buffers::MESSAGE_BUFFER_SIZE }>) {
    let sender = get_serial_sender();
    let _ = sender.send(SerialMessage::Formatted(message)).await;
}

pub fn send_sync_formatted_message(message: String<{ buffers::MESSAGE_BUFFER_SIZE }>) {
    let sender = get_serial_sender();
    let _ = sender.try_send(SerialMessage::Formatted(message));
}

/// Macro for send_sync_formatted_message
#[macro_export]
macro_rules! log_info {
    ($($arg:tt)*) => {
        let mut msg: heapless::String<{ crate::constants::buffers::MESSAGE_BUFFER_SIZE }> = heapless::String::new();
        let _ = core::fmt::write(
            &mut msg,
            format_args!($($arg)*),
        );
        crate::usb_serial::send_sync_formatted_message(msg);
    };
}

#[embassy_executor::task]
pub async fn usb_serial_task(usb: Peri<'static, USB>) -> ! {
    // Create the driver
    let driver = Driver::new(usb, UsbIrqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Raspberry Pi");
    config.product = Some("Pico 2 W USB Serial");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();
    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Get receiver for messages from other modules
    let message_receiver = SERIAL_CHANNEL.receiver();

    // Bidirectional communication handler
    let comm_fut = async {
        loop {
            class.wait_connection().await;
            defmt::info!("USB Serial connected");

            // Send initial greeting and setup terminal
            send_welcome_message(&mut class).await;

            let mut read_buf = [0u8; buffers::USB_PACKET_SIZE]; // USB packet receive buffer
            let mut line_buffer = [0u8; buffers::LINE_BUFFER_SIZE]; // Buffer for accumulating characters until Enter
            let mut line_pos = 0usize; // Current position in the line buffer
            let mut heartbeat_counter = 0u32;

            loop {
                // Use select to handle reading, messages from other modules, and periodic heartbeat
                let result = select3(
                    // Read incoming USB data with 100ms timeout
                    with_timeout(Duration::from_millis(100), class.read_packet(&mut read_buf)),
                    // Receive messages from other modules
                    message_receiver.receive(),
                    // Send heartbeat every 10 seconds to show device is alive
                    Timer::after(Duration::from_millis(timing::USB_HEARTBEAT_MS)),
                )
                .await;

                match result {
                    Either3::First(read_result) => {
                        match read_result {
                            Ok(Ok(n)) => {
                                if n > 0 {
                                    // Process each received byte individually
                                    for &byte in &read_buf[..n] {
                                        match byte {
                                            // Line terminators: Carriage Return (0x0D) and Line Feed (0x0A)
                                            // These indicate the user pressed Enter and wants to execute the command
                                            b'\r' | b'\n' => {
                                                if line_pos > 0 {
                                                    // Echo the newline to move to next line
                                                    let _ = class.write_packet(b"\r\n").await;

                                                    // Process the complete command
                                                    let command = &line_buffer[..line_pos];
                                                    process_command(&mut class, command).await;
                                                    send_prompt(&mut class).await;

                                                    // Reset buffer for next command
                                                    line_pos = 0;
                                                } else {
                                                    // Empty line - just show a new prompt
                                                    let _ = class.write_packet(b"\r\n").await;
                                                    send_prompt(&mut class).await;
                                                }
                                            }

                                            // Backspace (0x08) and DEL (0x7F) - used for editing the current line
                                            // Most terminals send one of these when the user presses backspace
                                            b'\x08' | b'\x7f' => {
                                                if line_pos > 0 {
                                                    // Remove character from buffer
                                                    line_pos -= 1;

                                                    // Send backspace sequence to terminal:
                                                    // \x08 - move cursor back one position
                                                    // ' '  - overwrite the character with a space
                                                    // \x08 - move cursor back again to final position
                                                    let _ = class.write_packet(b"\x08 \x08").await;
                                                }
                                                // If nothing to delete, do nothing (don't beep or show error)
                                            }

                                            // Ctrl+C (0x03) - cancel current line input
                                            // This is the ASCII ETX (End of Text) character
                                            b'\x03' => {
                                                if line_pos > 0 {
                                                    // Show ^C and start new line
                                                    let _ = class.write_packet(b"\r\n^C\r\n").await;
                                                    line_pos = 0;
                                                } else {
                                                    // Just show ^C if no input
                                                    let _ = class.write_packet(b"^C\r\n").await;
                                                }
                                                send_prompt(&mut class).await;
                                            }

                                            // Printable ASCII characters (space 0x20 through tilde 0x7E)
                                            // These are the characters the user can actually type and see
                                            0x20..=0x7E => {
                                                if line_pos < line_buffer.len() - 1 {
                                                    // Add character to buffer
                                                    line_buffer[line_pos] = byte;
                                                    line_pos += 1;

                                                    // Echo the character back to the terminal so user sees what they typed
                                                    let _ = class.write_packet(&[byte]).await;
                                                } else {
                                                    // Buffer full - send bell character (0x07) to make terminal beep
                                                    // This alerts the user that their input is too long
                                                    let _ = class.write_packet(b"\x07").await;
                                                }
                                            }

                                            // All other bytes (control characters, extended ASCII, etc.)
                                            // We ignore these to prevent terminal confusion
                                            _ => {
                                                // Silently ignore other control characters like:
                                                // - ESC sequences, arrow keys, function keys
                                                // - High-bit characters from different encodings
                                                // - Other control codes we don't handle
                                            }
                                        }
                                    }
                                }
                            }
                            Ok(Err(_)) => {
                                // USB read error, connection might be lost
                                break;
                            }
                            Err(_) => {
                                // Timeout occurred, continue to other cases
                            }
                        }
                    }
                    Either3::Second(message) => {
                        // Message received from another module - send it to terminal
                        send_module_message(&mut class, message).await;
                    }
                    Either3::Third(_) => {
                        // Heartbeat timeout - send periodic status
                        heartbeat_counter += 1;
                        send_heartbeat(&mut class, heartbeat_counter, &line_buffer, line_pos).await;
                    }
                }
            }
        }
    };

    // Run everything concurrently.
    embassy_futures::join::join(usb_fut, comm_fut).await;

    core::unreachable!()
}

/// Helper for 3-way select since embassy doesn't have select3
async fn select3<A, B, C>(a: A, b: B, c: C) -> Either3<A::Output, B::Output, C::Output>
where
    A: core::future::Future,
    B: core::future::Future,
    C: core::future::Future,
{
    match select(a, select(b, c)).await {
        Either::First(a_result) => Either3::First(a_result),
        Either::Second(Either::First(b_result)) => Either3::Second(b_result),
        Either::Second(Either::Second(c_result)) => Either3::Third(c_result),
    }
}

/// 3-way Either enum
enum Either3<A, B, C> {
    First(A),
    Second(B),
    Third(C),
}

/// Send a message from another module to the terminal
pub async fn send_module_message<'a>(
    class: &mut CdcAcmClass<'a, Driver<'a, USB>>,
    message: SerialMessage,
) {
    match message {
        SerialMessage::Text(text) => {
            let _ = class.write_packet(b"\r\n[MSG] ").await;
            let _ = class.write_packet(text.as_bytes()).await;
            let _ = class.write_packet(b"\r\n").await;
        }
        SerialMessage::AdcReading { channel, value } => {
            let mut msg: String<{ buffers::MESSAGE_BUFFER_SIZE }> = String::new();
            let _ = core::fmt::write(
                &mut msg,
                format_args!("\r\n[ADC] Channel {}: {} V\r\n", channel, value),
            );
            let _ = class.write_packet(msg.as_bytes()).await;
        }
        SerialMessage::Formatted(formatted_msg) => {
            let _ = class.write_packet(b"\r\n").await;
            let _ = class.write_packet(formatted_msg.as_bytes()).await;
            let _ = class.write_packet(b"\r\n").await;
        }
    }
}

/// Send welcome message when terminal connects
async fn send_welcome_message<'a>(class: &mut CdcAcmClass<'a, Driver<'a, USB>>) {
    let _ = class
        .write_packet(b"\r\nPico 2 W USB Serial Ready\r\n")
        .await;
    let _ = class
        .write_packet(b"Commands: led_on, led_off, status, echo <text>, help\r\n")
        .await;
    let _ = class
        .write_packet(b"Type your command and press Enter\r\n")
        .await;
    send_prompt(class).await;
}

/// Send command prompt
async fn send_prompt<'a>(class: &mut CdcAcmClass<'a, Driver<'a, USB>>) {
    let _ = class.write_packet(b"> ").await;
}

/// Send periodic heartbeat message
pub async fn send_heartbeat<'a>(
    class: &mut CdcAcmClass<'a, Driver<'a, USB>>,
    counter: u32,
    line_buffer: &[u8],
    line_pos: usize,
) {
    let mut status_msg: String<64> = String::new();
    let _ = core::fmt::write(
        &mut status_msg,
        format_args!("\r\n[Heartbeat #{}]\r\n", counter),
    );

    if class.write_packet(status_msg.as_bytes()).await.is_err() {
        return; // Connection lost
    }

    send_prompt(class).await;

    // Restore any partial command that was being typed when heartbeat interrupted
    if line_pos > 0 {
        let _ = class.write_packet(&line_buffer[..line_pos]).await;
    }
}

/// Process a complete command line
pub async fn process_command<'a>(class: &mut CdcAcmClass<'a, Driver<'a, USB>>, data: &[u8]) {
    if data.is_empty() {
        return;
    }

    // Log received command for debugging
    defmt::info!("Received command: {:?}", data);

    // Parse and execute commands
    if data == b"status" {
        let _ = class
            .write_packet(b"Pico 2 W Status: Running, WiFi initialized\r\n")
            .await;
    } else if data == b"led_on" {
        let _ = class.write_packet(b"LED turned ON\r\n").await;
        // TODO: Add actual LED control here
        defmt::info!("LED ON command received");
    } else if data == b"led_off" {
        let _ = class.write_packet(b"LED turned OFF\r\n").await;
        // TODO: Add actual LED control here
        defmt::info!("LED OFF command received");
    } else if data.starts_with(b"echo ") {
        let echo_text = &data[5..]; // Skip "echo " prefix
        let _ = class.write_packet(b"Echo: ").await;
        let _ = class.write_packet(echo_text).await;
        let _ = class.write_packet(b"\r\n").await;
    } else if data == b"help" {
        send_help(class).await;
    } else if data == b"clear" {
        // ANSI escape sequences:
        // \x1B[2J - clear entire screen (ESC[2J)
        // \x1B[H  - move cursor to home position (1,1) (ESC[H)
        let _ = class.write_packet(b"\x1B[2J\x1B[H").await;
        let _ = class.write_packet(b"Pico 2 W USB Serial Ready\r\n").await;
    } else {
        let _ = class
            .write_packet(b"Unknown command. Type 'help' for available commands.\r\n")
            .await;
    }
}

/// Send help text
async fn send_help<'a>(class: &mut CdcAcmClass<'a, Driver<'a, USB>>) {
    let _ = class.write_packet(b"Available commands:\r\n").await;
    let _ = class
        .write_packet(b"  status      - Show device status\r\n")
        .await;
    let _ = class.write_packet(b"  led_on      - Turn LED on\r\n").await;
    let _ = class
        .write_packet(b"  led_off     - Turn LED off\r\n")
        .await;
    let _ = class
        .write_packet(b"  echo <text> - Echo back text\r\n")
        .await;
    let _ = class
        .write_packet(b"  help        - Show this help\r\n")
        .await;
    let _ = class
        .write_packet(b"  clear       - Clear screen\r\n")
        .await;
}
