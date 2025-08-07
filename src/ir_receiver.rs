use defmt::Format;
use embassy_rp::gpio::{Input, Pull};
use embassy_rp::Peri;
use embassy_time::{Duration, Instant, Timer};
use heapless::Vec;

use crate::constants::buffers;

use crate::usb_serial::send_sync_formatted_message;
use core::fmt::Write;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIN_14, PIN_15, PWM_SLICE7};
use embassy_rp::pwm::{self, Pwm, SetDutyCycle};

/// Constants for IR receiver timings and configuration.
mod ir_timings {
    /// The duration in microseconds to poll for changes in the IR pin state.
    /// A small delay to prevent busy-waiting and consuming too much CPU.
    pub const LOOP_POLL_DELAY_US: u64 = 10;

    // --- NEC Protocol Timings (in microseconds) ---
    // These values define the expected pulse and space durations for the NEC IR protocol.
    // They include generous tolerances to account for variations in remote controls and sensors.

    /// Minimum duration of the initial start pulse to be considered valid.
    pub const NEC_START_PULSE_MIN_US: u64 = 8000; // 8ms
    /// Maximum duration of the initial start pulse to be considered valid.
    pub const NEC_START_PULSE_MAX_US: u64 = 10000; // 10ms

    /// Minimum duration of a data bit "mark" (LOW pulse).
    pub const NEC_MARK_MIN_US: u32 = 400;
    /// Maximum duration of a data bit "mark" (LOW pulse).
    pub const NEC_MARK_MAX_US: u32 = 800; // nominal 562µs

    /// Minimum duration of a "space" (HIGH pulse) for a logical '0'.
    pub const NEC_ZERO_SPACE_MIN_US: u32 = 400;
    /// Maximum duration of a "space" (HIGH pulse) for a logical '0'.
    pub const NEC_ZERO_SPACE_MAX_US: u32 = 800; // nominal 562µs

    /// Minimum duration of a "space" (HIGH pulse) for a logical '1'.
    pub const NEC_ONE_SPACE_MIN_US: u32 = 1400;
    /// Maximum duration of a "space" (HIGH pulse) for a logical '1'.
    pub const NEC_ONE_SPACE_MAX_US: u32 = 2000; // nominal 1687µs

    /// Minimum duration of a "space" (HIGH pulse) for a repeat code.
    pub const NEC_REPEAT_SPACE_MIN_US: u32 = 2000;
    /// Maximum duration of a "space" (HIGH pulse) for a repeat code.
    pub const NEC_REPEAT_SPACE_MAX_US: u32 = 2500; // nominal 2250µs

    // --- Receiver Configuration ---

    /// Maximum number of pulses to store. NEC protocol uses a max of 68 pulses
    /// (1 start bit + 32 data bits * 2 pulses per bit + 1 stop bit).
    pub const NEC_PULSE_BUFFER_SIZE: usize = 68;

    /// The minimum duration for a pulse to be considered valid data and not just noise.
    pub const PULSE_NOISE_FILTER_US: u32 = 200;

    /// Timeout in milliseconds while waiting for the initial low pulse of a transmission.
    pub const WAIT_FOR_START_TIMEOUT_MS: u64 = 100;

    /// Timeout in milliseconds for measuring the duration of the start pulse.
    pub const START_PULSE_MEASURE_TIMEOUT_MS: u64 = 15;

    /// Timeout in milliseconds for capturing the entire pulse train of a transmission.
    pub const CAPTURE_PULSES_TIMEOUT_MS: u64 = 100;

    /// The minimum number of decoded bits to consider a message valid.
    /// Some remotes might only send 16 or 24 bits, so this provides flexibility.
    pub const MIN_DECODED_BITS: usize = 16;
}

/// Represents the decoded message from the IR receiver.
/// Can be a full command or a repeating signal for a held button.
#[derive(Debug)]
pub enum IrMessage {
    Command(IrKey),
    Repeat,
}

/// IR remote control key mappings based on the provided table
#[derive(Debug, Clone, Copy, PartialEq, Format)]
pub enum IrKey {
    Power,
    Menu,
    Test,
    Plus,
    Return,
    Left,
    Play,
    Forward,
    Info,
    Minus,
    C,
    Num1,
    Num2,
    Num3,
    Num4,
    Num5,
    Num6,
    Num7,
    Num8,
    Num9,
    Unknown(u32),
}

impl IrKey {
    /// Convert a received IR code to an IrKey enum
    pub fn from_code(code: u32) -> Self {
        match code {
            0xFFA25D => IrKey::Power,
            0xFFE21D => IrKey::Menu,
            0xFF22DD => IrKey::Test,
            0xFF02FD => IrKey::Plus,
            0xFFC23D => IrKey::Return,
            0xFFE01F => IrKey::Left,
            0xFFA857 => IrKey::Play,
            0xFF906F => IrKey::Forward,
            0xFF6897 => IrKey::Info,
            0xFF9867 => IrKey::Minus,
            0xFFB04F => IrKey::C,
            0xFF30CF => IrKey::Num1,
            0xFF18E7 => IrKey::Num2,
            0xFF7A85 => IrKey::Num3,
            0xFF10EF => IrKey::Num4,
            0xFF38C7 => IrKey::Num5,
            0xFF5AA5 => IrKey::Num6,
            0xFF42BD => IrKey::Num7,
            0xFF4AB5 => IrKey::Num8,
            0xFF52AD => IrKey::Num9,
            _ => IrKey::Unknown(code),
        }
    }

    /// Get a human-readable name for the key
    pub fn name(&self) -> &'static str {
        match self {
            IrKey::Power => "POWER",
            IrKey::Menu => "MENU",
            IrKey::Test => "TEST",
            IrKey::Plus => "PLUS",
            IrKey::Return => "RETURN",
            IrKey::Left => "LEFT",
            IrKey::Play => "PLAY",
            IrKey::Forward => "FORWARD",
            IrKey::Info => "INFO",
            IrKey::Minus => "MINUS",
            IrKey::C => "C",
            IrKey::Num1 => "1",
            IrKey::Num2 => "2",
            IrKey::Num3 => "3",
            IrKey::Num4 => "4",
            IrKey::Num5 => "5",
            IrKey::Num6 => "6",
            IrKey::Num7 => "7",
            IrKey::Num8 => "8",
            IrKey::Num9 => "9",
            IrKey::Unknown(code) => "UNKNOWN",
        }
    }
}

/// IR receiver for NEC protocol (common for many IR remotes)
pub struct IrReceiver<'a> {
    pin: Input<'a>,
    pulses: Vec<u32, { ir_timings::NEC_PULSE_BUFFER_SIZE }>,
}

impl<'a> IrReceiver<'a> {
    /// Create a new IR receiver
    pub fn new(pin: Input<'a>) -> Self {
        Self {
            pin,
            pulses: Vec::new(),
        }
    }

    /// Wait for and decode an IR signal into a message (command or repeat).
    pub async fn read_message(&mut self) -> Option<IrMessage> {
        // Clear previous pulses
        self.pulses.clear();

        // Wait for the start of a transmission (long low pulse)
        if !self.wait_for_start().await {
            return None;
        }

        // Capture the pulse train
        if !self.capture_pulses().await {
            return None;
        }

        // Decode the pulses into a command
        self.decode_pulses()
    }

    /// Wait for the start of an IR transmission
    async fn wait_for_start(&mut self) -> bool {
        // Wait for line to go low (start of transmission)
        let timeout = Instant::now() + Duration::from_millis(ir_timings::WAIT_FOR_START_TIMEOUT_MS);
        while self.pin.is_high() && Instant::now() < timeout {
            Timer::after(Duration::from_micros(ir_timings::LOOP_POLL_DELAY_US)).await;
        }

        if Instant::now() >= timeout {
            return false;
        }

        // Measure the start pulse (should be ~9ms for NEC protocol)
        let start_time = Instant::now();
        let timeout =
            start_time + Duration::from_millis(ir_timings::START_PULSE_MEASURE_TIMEOUT_MS);

        while self.pin.is_low() && Instant::now() < timeout {
            Timer::after(Duration::from_micros(ir_timings::LOOP_POLL_DELAY_US)).await;
        }

        let pulse_duration = Instant::now() - start_time;

        // Check if it's a valid start pulse (8-10ms)
        let micros = pulse_duration.as_micros();
        micros >= ir_timings::NEC_START_PULSE_MIN_US && micros <= ir_timings::NEC_START_PULSE_MAX_US
    }

    /// Capture the entire pulse train
    async fn capture_pulses(&mut self) -> bool {
        let mut last_state = self.pin.is_high();
        let mut pulse_start = Instant::now();
        let timeout = pulse_start + Duration::from_millis(ir_timings::CAPTURE_PULSES_TIMEOUT_MS);

        while Instant::now() < timeout && self.pulses.len() < ir_timings::NEC_PULSE_BUFFER_SIZE {
            let current_state = self.pin.is_high();

            if current_state != last_state {
                let pulse_duration = (Instant::now() - pulse_start).as_micros() as u32;

                // Only store pulses longer than 200µs to filter noise
                if pulse_duration > ir_timings::PULSE_NOISE_FILTER_US {
                    if self.pulses.push(pulse_duration).is_err() {
                        break; // Vector is full
                    }
                }

                pulse_start = Instant::now();
                last_state = current_state;
            }

            Timer::after(Duration::from_micros(ir_timings::LOOP_POLL_DELAY_US)).await;
        }

        // We need at least 32 pulses for a valid NEC command (16 bits * 2 edges each)
        self.pulses.len() >= ir_timings::MIN_DECODED_BITS * 2
    }

    /// Decode the captured pulses into an IR command
    fn decode_pulses(&self) -> Option<IrMessage> {
        // First, check for a repeat code. It's a very short signal.
        // After the 9ms start burst, it's just a ~2.25ms space and a ~560µs mark.
        if self.pulses.len() >= 2 && self.pulses.len() <= 4 {
            let space_duration = self.pulses[0];
            let mark_duration = self.pulses[1];
            if (ir_timings::NEC_REPEAT_SPACE_MIN_US..=ir_timings::NEC_REPEAT_SPACE_MAX_US)
                .contains(&space_duration)
                && (ir_timings::NEC_MARK_MIN_US..=ir_timings::NEC_MARK_MAX_US)
                    .contains(&mark_duration)
            {
                return Some(IrMessage::Repeat);
            }
        }

        // A valid NEC transmission has at least 16 bits of data (address + command),
        // which means 32 data pulses (16 marks, 16 spaces) plus the header space.
        if self.pulses.len() < (ir_timings::MIN_DECODED_BITS * 2) + 1 {
            return None;
        }

        // The first pulse is the 4.5ms space after the start burst. We skip it.
        // The rest are pairs of [mark (low), space (high)].
        let data_pulses = &self.pulses[1..];

        let mut command: u32 = 0;
        let mut bit_count = 0;

        // Process pairs of pulses (mark and space)
        for chunk in data_pulses.chunks(2) {
            if bit_count >= 32 {
                // NEC is a 32-bit protocol
                break;
            }

            let mark_duration = chunk[0]; // LOW pulse
            let space_duration = chunk[1]; // HIGH pulse

            // For NEC protocol:
            // - A ~562µs LOW mark is expected for every bit.
            // - Logical '0': followed by a ~562µs HIGH space.
            // - Logical '1': followed by a ~1687µs HIGH space.
            // We'll be more lenient with timing due to variations.

            if (ir_timings::NEC_MARK_MIN_US..=ir_timings::NEC_MARK_MAX_US).contains(&mark_duration)
            {
                if (ir_timings::NEC_ZERO_SPACE_MIN_US..=ir_timings::NEC_ZERO_SPACE_MAX_US)
                    .contains(&space_duration)
                {
                    // Logical 0
                    command <<= 1;
                    bit_count += 1;
                } else if (ir_timings::NEC_ONE_SPACE_MIN_US..=ir_timings::NEC_ONE_SPACE_MAX_US)
                    .contains(&space_duration)
                {
                    // Logical 1
                    command = (command << 1) | 1;
                    bit_count += 1;
                } else {
                    // Invalid space timing, stop decoding
                    break;
                }
            } else {
                // Invalid mark timing, stop decoding
                break;
            }
        }

        if bit_count >= ir_timings::MIN_DECODED_BITS {
            // NEC protocol sends address, inverted address, command, and inverted command.
            // Many remotes just send an 8-bit address and 8-bit command.
            // The full 32-bit `command` variable will contain all of this.
            // We can extract what we need or use the whole thing.
            // Here, we check the last 16 bits (command and inverted command)
            // or use the whole code for matching.
            let address = (command >> 24) & 0xFF;
            let ir_command = (command >> 8) & 0xFF;

            // Our IrKey map uses the full 32-bit code (or at least, the significant parts)
            Some(IrMessage::Command(IrKey::from_code(command)))
        } else {
            // This is a catch-all for signals that are not valid commands.
            // We log it to help debug unknown remote signals, like non-standard repeat codes.
            let mut msg: heapless::String<{ buffers::MESSAGE_BUFFER_SIZE }> =
                heapless::String::new();
            let _ = write!(
                msg,
                "Unknown signal received. Pulses: {} -> [",
                self.pulses.len()
            );
            // Log the first few pulses to help identify the pattern
            for (i, p) in self.pulses.iter().take(4).enumerate() {
                if i > 0 {
                    let _ = write!(msg, ", ");
                }
                let _ = write!(msg, "{}", p);
            }
            let _ = write!(msg, "]");
            send_sync_formatted_message(msg);
            None
        }
    }
}

/// Task to continuously read IR signals and process them
#[embassy_executor::task]
pub async fn ir_receiver_task(
    pin: Peri<'static, embassy_rp::peripherals::PIN_16>,
    led_pin: Peri<'static, PIN_14>,
    buzzer_pin: Peri<'static, PIN_15>,
    pwm_slice: Peri<'static, PWM_SLICE7>,
) -> ! {
    /// Duration after which a press is considered "long".
    const LONG_PRESS_DURATION: Duration = Duration::from_millis(500);
    /// If no signal (command or repeat) is received for this duration, the key is considered released.
    const RELEASE_TIMEOUT: Duration = Duration::from_millis(150);

    let mut msg: heapless::String<{ buffers::MESSAGE_BUFFER_SIZE }> = heapless::String::new();
    let _ = write!(msg, "Starting IR receiver task on GPIO 16");
    send_sync_formatted_message(msg);

    let ir_pin = Input::new(pin, Pull::Up);
    let mut ir_receiver = IrReceiver::new(ir_pin);

    // --- LED PWM Setup ---
    let mut led_pwm_config = pwm::Config::default();
    led_pwm_config.top = 255; // 8-bit resolution is simple and effective
    let mut led_pwm = Pwm::new_output_a(pwm_slice, led_pin, led_pwm_config);
    let mut led_brightness_level: u8 = 0; // 0-3 levels

    // --- Buzzer Setup ---
    let mut buzzer = Output::new(buzzer_pin, Level::Low);

    let mut current_press: Option<(IrKey, Instant)> = None;
    let mut is_long_press_reported = false;

    loop {
        let message = ir_receiver.read_message().await;

        match message {
            Some(IrMessage::Command(key)) => {
                let mut msg: heapless::String<{ buffers::MESSAGE_BUFFER_SIZE }> =
                    heapless::String::new();
                let _ = write!(msg, "Key Pressed: {}", key.name());
                send_sync_formatted_message(msg);

                // Handle special keys for LED and Buzzer
                match key {
                    IrKey::Plus => {
                        led_brightness_level = (led_brightness_level + 1).min(3);
                        let duty = match led_brightness_level {
                            0 => 0,
                            1 => 85,
                            2 => 170,
                            _ => 255,
                        };
                        let _ = led_pwm.set_duty_cycle(duty);
                        let mut msg: heapless::String<{ buffers::MESSAGE_BUFFER_SIZE }> =
                            heapless::String::new();
                        let _ = write!(msg, "LED Brightness: {}", led_brightness_level);
                        send_sync_formatted_message(msg);
                    }
                    IrKey::Minus => {
                        led_brightness_level = led_brightness_level.saturating_sub(1);
                        let duty = match led_brightness_level {
                            0 => 0,
                            1 => 85,
                            2 => 170,
                            _ => 255,
                        };

                        let _ = led_pwm.set_duty_cycle(duty);
                        let mut msg: heapless::String<{ buffers::MESSAGE_BUFFER_SIZE }> =
                            heapless::String::new();
                        let _ = write!(msg, "LED Brightness: {}", led_brightness_level);
                        send_sync_formatted_message(msg);
                    }
                    IrKey::Play => {
                        let mut msg: heapless::String<{ buffers::MESSAGE_BUFFER_SIZE }> =
                            heapless::String::new();
                        let _ = write!(msg, "Buzzer On!");
                        send_sync_formatted_message(msg);
                        buzzer.set_high();
                        Timer::after(Duration::from_millis(100)).await;
                        buzzer.set_low();
                    }
                    _ => {}
                }

                current_press = Some((key, Instant::now()));
                is_long_press_reported = false;
            }
            Some(IrMessage::Repeat) => {
                if let Some((key, press_start_time)) = &mut current_press {
                    let now = Instant::now();
                    if !is_long_press_reported && (now - *press_start_time) >= LONG_PRESS_DURATION {
                        let mut msg: heapless::String<{ buffers::MESSAGE_BUFFER_SIZE }> =
                            heapless::String::new();
                        let _ = write!(msg, "Long Press Detected: {}", key.name());
                        send_sync_formatted_message(msg);
                        is_long_press_reported = true;
                    }
                    // Update the timestamp to reset the release timer
                    *press_start_time = now;
                }
            }
            None => {
                if let Some((key, last_seen_time)) = current_press {
                    if (Instant::now() - last_seen_time) >= RELEASE_TIMEOUT {
                        if !is_long_press_reported {
                            let mut msg: heapless::String<{ buffers::MESSAGE_BUFFER_SIZE }> =
                                heapless::String::new();
                            let _ = write!(msg, "Short Press Detected: {}", key.name());
                            send_sync_formatted_message(msg);
                        }

                        let mut msg: heapless::String<{ buffers::MESSAGE_BUFFER_SIZE }> =
                            heapless::String::new();
                        let _ = write!(msg, "Key Released: {}", key.name());
                        send_sync_formatted_message(msg);

                        // Reset state
                        current_press = None;
                        is_long_press_reported = false;
                    }
                }
            }
        }
        // Small delay to keep the loop from being too tight when no signal is present.
        Timer::after(Duration::from_millis(10)).await;
    }
}
