use crate::{
    constants::{buffers, timing, voltage},
    usb_serial::send_sync_formatted_message,
};

use embassy_rp::peripherals::{PIN_18, PIN_19};
use embassy_rp::{
    gpio::{Input, Level, Output, Pull},
    Peri,
};
use embassy_time::with_timeout;
use embassy_time::{Duration, Instant, Timer};
use heapless::String;
// Ultrasonic sensor constants
const TRIGGER_PULSE_DURATION_US: u64 = 10; // 10μs trigger pulse
const TRIGGER_SETUP_DELAY_US: u64 = 2; // 2μs setup delay before trigger
const POLLING_DELAY_US: u64 = 1; // 1μs delay in polling loops
const MEASUREMENT_INTERVAL_SECS: u64 = 3; // 3 seconds between measurements

// Distance and timing constants
const MIN_DISTANCE_CM: f32 = 2.0; // Minimum valid distance (2cm)
const MAX_DISTANCE_CM: f32 = 400.0; // Maximum valid distance (400cm)
const SOUND_SPEED_BASE_CM_PER_US: f32 = 0.03313; // Speed of sound at 0°C: 331.3 m/s = 0.03313 cm/μs
const SOUND_SPEED_TEMP_COEFF: f32 = 0.000606; // Temperature coefficient: 0.606 m/s per °C = 0.000606 cm/μs per °C
const DEFAULT_TEMPERATURE_C: f32 = 20.0; // Default temperature if none provided
const TIMEOUT_SAFETY_FACTOR: f32 = 1.5; // Safety factor for timeout calculation

/// Calculate speed of sound based on temperature
/// Formula: v = 331.3 + (0.606 × T) m/s converted to cm/μs
const fn calculate_sound_speed_cm_per_us(temperature_c: f32) -> f32 {
    SOUND_SPEED_BASE_CM_PER_US + (SOUND_SPEED_TEMP_COEFF * temperature_c)
}

// Calculate timeout based on maximum distance at default temperature
// Time for sound to travel max distance and back = (2 * MAX_DISTANCE_CM) / sound_speed
// Add safety factor to account for variations
const fn calculate_timeout_us() -> u64 {
    let sound_speed = calculate_sound_speed_cm_per_us(DEFAULT_TEMPERATURE_C);
    let max_time_us = (2.0 * MAX_DISTANCE_CM / sound_speed) as u64;
    (max_time_us as f32 * TIMEOUT_SAFETY_FACTOR) as u64
}

const TIMEOUT_US: u64 = calculate_timeout_us();

/// Ultrasonic sensor driver for HC-SR04 style sensors
pub struct UltrasonicSensor<'a> {
    trig_pin: Output<'a>,
    echo_pin: Input<'a>,
}

impl<'a> UltrasonicSensor<'a> {
    /// Creates a new ultrasonic sensor instance
    pub fn new(trig_pin: Output<'a>, echo_pin: Input<'a>) -> Self {
        Self { trig_pin, echo_pin }
    }

    /// Measures distance in centimeters using default temperature
    /// Returns None if timeout occurs or invalid reading
    pub async fn measure_distance_cm(&mut self) -> Option<f32> {
        self.measure_distance_cm_with_temp(DEFAULT_TEMPERATURE_C)
            .await
    }

    /// Measures distance in centimeters with temperature compensation
    /// Returns None if timeout occurs or invalid reading
    pub async fn measure_distance_cm_with_temp(&mut self, temperature_c: f32) -> Option<f32> {
        // Ensure trigger pin starts low
        self.trig_pin.set_low();
        Timer::after(Duration::from_micros(TRIGGER_SETUP_DELAY_US)).await;

        // Send trigger pulse
        self.trig_pin.set_high();
        Timer::after(Duration::from_micros(TRIGGER_PULSE_DURATION_US)).await;
        self.trig_pin.set_low();

        // Wait for echo to go high (start of echo pulse)
        let timeout = Duration::from_micros(TIMEOUT_US);
        let start_time = Instant::now();

        while self.echo_pin.get_level() == Level::Low {
            if start_time.elapsed() > timeout {
                let mut msg: String<{ buffers::MESSAGE_BUFFER_SIZE }> = String::new();
                let _ = core::fmt::write(
                    &mut msg,
                    format_args!(
                        "Ultrasonic: Timeout waiting for echo start ({}us max)",
                        TIMEOUT_US
                    ),
                );
                send_sync_formatted_message(msg);
                return None;
            }
            // Small delay to prevent tight loop
            Timer::after(Duration::from_micros(POLLING_DELAY_US)).await;
        }

        // Record when echo started
        let echo_start = Instant::now();

        // Wait for echo to go low (end of echo pulse)
        while self.echo_pin.get_level() == Level::High {
            if echo_start.elapsed() > timeout {
                let mut msg: String<{ buffers::MESSAGE_BUFFER_SIZE }> = String::new();
                let _ = core::fmt::write(
                    &mut msg,
                    format_args!(
                        "Ultrasonic: Timeout waiting for echo end ({}us max)",
                        TIMEOUT_US
                    ),
                );
                send_sync_formatted_message(msg);
                return None;
            }
            Timer::after(Duration::from_micros(POLLING_DELAY_US)).await;
        }

        // Calculate duration of echo pulse
        let echo_duration = echo_start.elapsed();
        let duration_micros = echo_duration.as_micros() as f32;

        // Convert to distance in cm using temperature-compensated speed of sound
        // Distance = (duration * speed) / 2 (divide by 2 for round trip)
        let sound_speed = calculate_sound_speed_cm_per_us(temperature_c);
        let distance_cm = (duration_micros * sound_speed) / 2.0;

        // Sanity check: validate distance is within expected range
        if distance_cm < MIN_DISTANCE_CM || distance_cm > MAX_DISTANCE_CM {
            let mut msg: String<{ buffers::MESSAGE_BUFFER_SIZE }> = String::new();
            let _ = core::fmt::write(
                &mut msg,
                format_args!(
                    "Ultrasonic: Invalid distance reading: {} cm (valid range: {}-{} cm)",
                    distance_cm as u32, MIN_DISTANCE_CM as u32, MAX_DISTANCE_CM as u32
                ),
            );
            send_sync_formatted_message(msg);
            return None;
        }

        Some(distance_cm)
    }
}

/// Task that continuously measures distance every 3 seconds
#[embassy_executor::task]
pub async fn ultrasonic_task(
    trig_pin: Peri<'static, PIN_19>,
    echo_pin: Peri<'static, PIN_18>,
) -> ! {
    let mut trig = Output::new(trig_pin, Level::Low);
    let echo = Input::new(echo_pin, Pull::None);

    loop {
        // Send a 10us pulse on the trigger pin
        trig.set_high();
        Timer::after(Duration::from_micros(10)).await;
        trig.set_low();

        // Wait for the echo pin to go high, with a timeout
        let start_of_echo = with_timeout(Duration::from_millis(100), async {
            while echo.is_low() {}
            Instant::now()
        })
        .await
        .ok();

        if let Some(start_time) = start_of_echo {
            // Wait for the echo pin to go low, with a timeout
            let end_of_echo = with_timeout(Duration::from_millis(100), async {
                while echo.is_high() {}
                Instant::now()
            })
            .await
            .ok();

            if let Some(end_time) = end_of_echo {
                let duration = end_time - start_time;
                let distance_cm = (duration.as_micros() as f32 / 2.0) / 29.1;

                let mut msg: String<{ buffers::MESSAGE_BUFFER_SIZE }> = String::new();
                let _ = core::fmt::write(
                    &mut msg,
                    format_args!(
                        "Distance: {:.2} cm (pulse: {} us)",
                        distance_cm,
                        duration.as_micros()
                    ),
                );
                send_sync_formatted_message(msg);
            } else {
                let mut msg: String<{ buffers::MESSAGE_BUFFER_SIZE }> = String::new();
                let _ = core::fmt::write(
                    &mut msg,
                    format_args!("Echo signal timed out (did not go low)"),
                );
                send_sync_formatted_message(msg);
            }
        } else {
        }

        // Wait before the next measurement
        Timer::after(Duration::from_millis(timing::VISUAL_DELAY_MS)).await;
    }
}
