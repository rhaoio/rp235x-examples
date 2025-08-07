#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either3};
use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use embassy_rp::Peri;
use embassy_time::{with_timeout, Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::{Builder, Config};
use heapless::String;
use rp_pico2_examples::constants::{buffers, timing};
use rp_pico2_examples::usb_serial::{
    process_command, send_heartbeat, send_module_message, SerialMessage, SERIAL_CHANNEL,
};

use {defmt_rtt as _, panic_probe as _};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"USB Serial Communication Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"Standalone USB CDC-ACM serial communication example on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct UsbIrqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

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

#[embassy_executor::task]
async fn message_sender_task() -> ! {
    let sender = SERIAL_CHANNEL.sender();
    let mut counter = 0u32;

    // Wait for USB to be ready
    Timer::after(Duration::from_secs(3)).await;

    loop {
        counter += 1;

        match counter % 4 {
            0 => {
                let _ = sender.try_send(SerialMessage::Text("System status: Running\r\n"));
            }
            1 => {
                let _ = sender.try_send(SerialMessage::AdcReading {
                    channel: 0,
                    value: 2.5 + (counter % 10) as f32 * 0.1,
                });
            }
            2 => {
                let mut msg = String::<128>::new();
                let _ = core::fmt::write(&mut msg, format_args!("Counter: {}\r\n", counter));
                let _ = sender.try_send(SerialMessage::Formatted(msg));
            }
            3 => {
                let _ = sender.try_send(SerialMessage::Text("Sensors: All operational\r\n"));
            }
            _ => {}
        }

        Timer::after(Duration::from_secs(2)).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("USB Serial Communication Example Starting");

    // Start USB Serial task (copied from main.rs pattern)
    unwrap!(spawner.spawn(usb_serial_task(p.USB)));

    // Start message sender task
    unwrap!(spawner.spawn(message_sender_task()));

    // Main loop
    loop {
        info!("USB Serial example running...");
        Timer::after(Duration::from_secs(10)).await;
    }
}
