use defmt::*;
use embassy_rp::{
    adc::{Adc, Config as AdcConfig},
    gpio::Pull,
    peripherals::{ADC, PIN_26, PIN_27, PIN_28},
    Peri,
};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Channel, Receiver, Sender};
use embassy_time::{Duration, Timer};
use heapless::String;

use crate::adc_reader::AdcIrqs;
use crate::constants::{adc, joystick as joy_const};
use crate::usb_serial::{send_formatted_message, send_message};

// Channel for sending joystick readings
pub static JOYSTICK_CHANNEL: Channel<ThreadModeRawMutex, JoystickReading, 8> = Channel::new();

/// Get the joystick channel sender
pub fn get_joystick_sender() -> Sender<'static, ThreadModeRawMutex, JoystickReading, 8> {
    JOYSTICK_CHANNEL.sender()
}

/// Get the joystick channel receiver
pub fn get_joystick_receiver() -> Receiver<'static, ThreadModeRawMutex, JoystickReading, 8> {
    JOYSTICK_CHANNEL.receiver()
}

/// Structure containing joystick ADC reading data
#[derive(Clone, Copy, Debug)]
pub struct JoystickReading {
    pub x_axis: AxisReading,
    pub y_axis: AxisReading,
    pub z_axis: AxisReading, // Could be button or rotation
    pub timestamp_ms: u64,
}

/// Individual axis reading
#[derive(Clone, Copy, Debug)]
pub struct AxisReading {
    pub raw_value: u16,
    pub voltage_mv: u16,
    pub normalized: i16, // Normalized to -1000 to +1000 range
    pub pin_number: u8,
}

impl JoystickReading {
    /// Create a new joystick reading with calculated normalized values
    pub fn new(x_raw: u16, y_raw: u16, z_raw: u16, timestamp_ms: u64) -> Self {
        Self {
            x_axis: AxisReading::new(x_raw, 28), // X-axis on pin 28
            y_axis: AxisReading::new(y_raw, 27), // Y-axis on pin 27
            z_axis: AxisReading::new(z_raw, 26), // Z-axis (button) on pin 26
            timestamp_ms,
        }
    }

    /// Get a formatted string representation of the joystick state
    pub fn format_reading(&self) -> String<128> {
        let mut msg: String<128> = String::new();
        let _ =
            core::fmt::write(
                &mut msg,
                format_args!(
                "[JOYSTICK] X: {} Y: {} Z: {} | Raw: ({}, {}, {}) | V: ({}.{}V, {}.{}V, {}.{}V)",
                self.x_axis.normalized,
                self.y_axis.normalized,
                self.z_axis.normalized,
                self.x_axis.raw_value,
                self.y_axis.raw_value,
                self.z_axis.raw_value,
                self.x_axis.voltage_mv / 1000, self.x_axis.voltage_mv % 1000,
                self.y_axis.voltage_mv / 1000, self.y_axis.voltage_mv % 1000,
                self.z_axis.voltage_mv / 1000, self.z_axis.voltage_mv % 1000
            ),
            );
        msg
    }
}

impl AxisReading {
    /// Create a new axis reading with voltage and normalized calculations
    pub fn new(raw_value: u16, pin_number: u8) -> Self {
        let voltage_mv = adc_to_millivolts(raw_value);
        let normalized = raw_to_normalized(raw_value);

        Self {
            raw_value,
            voltage_mv,
            normalized,
            pin_number,
        }
    }
}

/// Convert ADC raw value (0-4095) to millivolts (0-3300mV)
fn adc_to_millivolts(adc_value: u16) -> u16 {
    // Use the constant function from the adc module
    adc::raw_to_millivolts(adc_value)
}

/// Convert raw ADC value to normalized joystick coordinate (-1000 to +1000)
/// Assumes center position is around 2047 (half of 4095)
fn raw_to_normalized(raw_value: u16) -> i16 {
    const ADC_CENTER: i32 = adc::CENTER_VALUE as i32; // Expected center position
    const ADC_MAX_RANGE: i32 = adc::CENTER_VALUE as i32; // Maximum deviation from center

    // Convert to signed value relative to center
    let relative_value = (raw_value as i32) - ADC_CENTER;

    // Scale to -1000 to +1000 range
    let normalized = (relative_value * joy_const::NORMALIZED_MAX as i32) / ADC_MAX_RANGE;

    // Clamp to valid range
    normalized
        .max(joy_const::NORMALIZED_MIN as i32)
        .min(joy_const::NORMALIZED_MAX as i32) as i16
}

/// Main joystick reading task that continuously reads from all three ADC pins
#[embassy_executor::task]
pub async fn joystick_reader_task(
    adc_peripheral: Peri<'static, ADC>,
    pin_26: Peri<'static, PIN_26>,
    pin_27: Peri<'static, PIN_27>,
    pin_28: Peri<'static, PIN_28>,
) -> ! {
    info!("Starting joystick reader task");

    // Send startup message
    send_message("Joystick reader started - monitoring pins 26, 27, 28").await;

    // Initialize ADC with proper configuration
    let mut adc_config = AdcConfig::default();
    let mut adc = Adc::new(adc_peripheral, AdcIrqs, adc_config);

    // Configure ADC pins
    let mut x_pin = embassy_rp::adc::Channel::new_pin(pin_28, Pull::None); // X-axis
    let mut y_pin = embassy_rp::adc::Channel::new_pin(pin_27, Pull::None); // Y-axis
    let mut z_pin = embassy_rp::adc::Channel::new_pin(pin_26, Pull::Up); // Z-axis/button

    let sender = get_joystick_sender();
    let mut timestamp_ms = 0u64;
    loop {
        // Read all three ADC channels
        let x_raw = adc.read(&mut x_pin).await.unwrap_or(0);
        let y_raw = adc.read(&mut y_pin).await.unwrap_or(0);
        let z_raw = adc.read(&mut z_pin).await.unwrap_or(0);

        // Create joystick reading
        let reading = JoystickReading::new(x_raw, y_raw, z_raw, timestamp_ms);

        // Send reading to channel for other tasks to consume
        if sender.try_send(reading).is_err() {
            warn!("Joystick channel full, dropping reading");
        }
        timestamp_ms += 50; // Increment timestamp
        Timer::after(Duration::from_millis(50)).await;
    }
}

/// Task that consumes joystick readings and processes them
/// This is an example of how you can read from the joystick channel
#[embassy_executor::task]
pub async fn joystick_consumer_task() -> ! {
    info!("Starting joystick consumer task");

    let receiver = get_joystick_receiver();
    let mut prev_updated_ms = 0u64;
    loop {
        let timestamp_ms = embassy_time::Instant::now().as_millis();
        // Wait for a joystick reading
        let reading = receiver.receive().await;
        //only send the message every 1 second
        if timestamp_ms - prev_updated_ms > 1000 {
            process_joystick_input(reading).await;
            prev_updated_ms = timestamp_ms;
        }
    }
}

/// Example function that processes joystick input and can control other systems
async fn process_joystick_input(reading: JoystickReading) {
    // Pass through raw joystick values without any filtering
    let x = reading.x_axis.normalized;
    let y = reading.y_axis.normalized;
    let z_voltage = reading.z_axis.voltage_mv;

    // Always log joystick state (for debugging - can be reduced in production)
    let mut action_msg: String<128> = String::new();
    let _ = core::fmt::write(
        &mut action_msg,
        format_args!("[JOYSTICK_RAW] X:{} Y:{} Z:{}mV", x, y, z_voltage),
    );

    send_formatted_message(action_msg).await;

    // Here you could add logic to control other systems based on joystick input
    // For example:
    // - Control motor speeds based on raw X/Y values
    // - Use Z-axis voltage for analog trigger/button
    // - Control LED patterns
    // - Send commands over WiFi/USB
}
