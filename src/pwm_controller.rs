//! # PWM Controller Module
//!
//! This module provides channel-based communication between ADC readers and PWM/GPIO controllers.
//! It allows you to control PWM outputs or digital GPIO pins based on ADC voltage readings.
//!
//! ## Architecture
//!
//! ```
//! ADC Reader Task  ->  Channel  ->  PWM Controller Task
//!      |                           |
//!   Reads ADC                   Receives data
//!   Sends data                  Controls PWM/GPIO
//! ```
//!
//! ## Usage Examples
//!
//! ### 1. Basic ADC to PWM Control
//!
//! ```rust
//! // In main.rs:
//!
//! // Start ADC reader that sends data to channel
//! spawner.spawn(adc_reader::single_adc_to_channel_task(p.ADC, p.PIN_26)).unwrap();
//!
//! // Start PWM controller that receives from channel
//! spawner.spawn(pwm_controller::single_pwm_controller_task(
//!     p.PWM_SLICE4,
//!     p.PIN_8,
//!     pwm_controller::voltage_to_brightness_config()
//! )).unwrap();
//! ```
//!
//! ### 2. Custom Voltage Range Mapping
//!
//! ```rust
//! let custom_config = PwmControllerConfig {
//!     min_voltage_mv: 500,    // 0% PWM at 0.5V
//!     max_voltage_mv: 2500,   // 100% PWM at 2.5V
//!     invert_output: false,   // Higher voltage = higher PWM
//!     update_rate_ms: 50,     // Update every 50ms
//! };
//!
//! spawner.spawn(pwm_controller::single_pwm_controller_task(
//!     p.PWM_SLICE4, p.PIN_8, custom_config
//! )).unwrap();
//! ```
//!
//! ### 3. GPIO Threshold Control
//!
//! ```rust
//! // Turn on LED when voltage > 1.65V
//! spawner.spawn(adc_reader::single_adc_to_channel_task(p.ADC, p.PIN_26)).unwrap();
//! spawner.spawn(pwm_controller::gpio_controller_task(p.PIN_15, 1650)).unwrap();
//! ```
//!
//! ## Channel Configuration
//!
//! - Channels have a buffer size of 8 readings
//! - ADC tasks use `try_send()` to avoid blocking
//! - If channel is full, new readings are dropped
//! - PWM tasks block on `receive()` waiting for data
//!
//! ## Pin Configuration
//!
//! Make sure your pins don't conflict:
//! - ADC pins: 26, 27, 28 (and internal temp sensor)
//! - PWM pins: Depend on slice - check RP2040 datasheet
//! - Example: PWM_SLICE4 uses pins 8 and 9
//!

use defmt::*;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIN_15, PIN_8, PIN_9, PWM_SLICE4, PWM_SLICE7};
use embassy_rp::pwm::{Config as PwmConfig, Pwm, SetDutyCycle};
use embassy_rp::Peri;

use embassy_time::{Duration, Timer};

use crate::adc_reader::{AdcReading, ADC_CHANNEL, SINGLE_ADC_CHANNEL};
use crate::constants::{pwm, timing, voltage};

/// PWM controller configuration
pub struct PwmControllerConfig {
    pub min_voltage_mv: u16, // Minimum voltage for 0% duty cycle
    pub max_voltage_mv: u16, // Maximum voltage for 100% duty cycle
    pub invert_output: bool, // Invert the PWM relationship
    pub update_rate_ms: u64, // How often to update PWM
}

impl Default for PwmControllerConfig {
    fn default() -> Self {
        Self {
            min_voltage_mv: 0,
            max_voltage_mv: voltage::VCC_3V3_MV,
            invert_output: false,
            update_rate_ms: timing::MEDIUM_UPDATE_MS, // 20Hz update rate
        }
    }
}

/// PWM controller task that receives ADC readings via channel
#[embassy_executor::task]
pub async fn pwm_controller_task(
    pwm_slice: Peri<'static, PWM_SLICE4>,
    pin_a: Peri<'static, PIN_8>,
    pin_b: Peri<'static, PIN_9>,
    config: PwmControllerConfig,
) -> ! {
    info!("Starting PWM controller task");

    // Initialize PWM
    let mut pwm_config = PwmConfig::default();
    pwm_config.top = pwm::STANDARD_TOP_VALUE; // 0-1000 range for easy percentage calculation
    pwm_config.compare_a = 0; // Start at 0% duty cycle
    pwm_config.compare_b = 0;

    let mut pwm = Pwm::new_output_ab(pwm_slice, pin_a, pin_b, pwm_config.clone());

    // Get receiver for ADC data
    let receiver = ADC_CHANNEL.receiver();

    info!("PWM controller ready, waiting for ADC data...");

    loop {
        // Wait for ADC reading
        let readings = receiver.receive().await;

        // Calculate duty cycle based on voltage
        let duty_cycle = calculate_duty_cycle(&readings.adc, &config);

        // Update PWM
        let pwm_value = (duty_cycle as u32 * pwm_config.top as u32) / 100;
        pwm.set_counter(pwm_value as u16);

        info!(
            "ADC: {}mV -> PWM: {}% (value: {})",
            readings.adc.voltage_mv, duty_cycle, pwm_value
        );

        // Optional: Add minimum update delay to prevent too frequent updates
        Timer::after(Duration::from_millis(config.update_rate_ms)).await;
    }
}

/// Single channel PWM controller for simpler use cases
#[embassy_executor::task]
pub async fn single_pwm_controller_task(
    pwm_slice: Peri<'static, PWM_SLICE7>,
    pin_a: Peri<'static, PIN_15>,
    config: PwmControllerConfig,
) -> ! {
    info!("Starting single PWM controller task");

    // Initialize PWM (single channel)
    let mut pwm_config = PwmConfig::default();
    pwm_config.top = pwm::STANDARD_TOP_VALUE;
    pwm_config.compare_a = 0;

    let mut pwm = Pwm::new_output_b(pwm_slice, pin_a, pwm_config.clone());

    // Get receiver for single ADC data
    let receiver = SINGLE_ADC_CHANNEL.receiver();

    info!("Single PWM controller ready, waiting for ADC data...");

    loop {
        // Wait for ADC reading
        let reading = receiver.receive().await;

        // Calculate duty cycle based on voltage
        let duty_cycle = calculate_duty_cycle(&reading, &config);

        let _ = pwm.set_duty_cycle_percent(duty_cycle);

        info!("ADC: {}mV -> PWM: {}%", reading.voltage_mv, duty_cycle);
    }
}

/// GPIO controller task that receives ADC readings and controls digital outputs
#[embassy_executor::task]
pub async fn gpio_controller_task(led_pin: Peri<'static, PIN_8>, threshold_mv: u16) -> ! {
    info!(
        "Starting GPIO controller task (threshold: {}mV)",
        threshold_mv
    );

    let mut led = Output::new(led_pin, Level::Low);
    let receiver = SINGLE_ADC_CHANNEL.receiver();

    loop {
        let reading = receiver.receive().await;

        if reading.voltage_mv > threshold_mv {
            led.set_high();
            info!("GPIO HIGH: {}mV > {}mV", reading.voltage_mv, threshold_mv);
        } else {
            led.set_low();
            info!("GPIO LOW: {}mV <= {}mV", reading.voltage_mv, threshold_mv);
        }
    }
}

/// Calculate PWM duty cycle percentage based on ADC reading and configuration
fn calculate_duty_cycle(reading: &AdcReading, config: &PwmControllerConfig) -> u8 {
    let voltage = reading.voltage_mv;

    // Clamp voltage to configured range
    let clamped_voltage = voltage.clamp(config.min_voltage_mv, config.max_voltage_mv);

    // Calculate percentage within range
    let voltage_range = config.max_voltage_mv - config.min_voltage_mv;
    let voltage_offset = clamped_voltage - config.min_voltage_mv;

    let mut duty_percent = if voltage_range > 0 {
        (voltage_offset as u32 * 100) / voltage_range as u32
    } else {
        0
    };

    // Invert if configured
    if config.invert_output {
        duty_percent = 100 - duty_percent;
    }

    duty_percent.min(100) as u8
}

/// Example configurations
pub fn voltage_to_brightness_config() -> PwmControllerConfig {
    PwmControllerConfig {
        min_voltage_mv: 0,
        max_voltage_mv: voltage::VCC_3V3_MV,
        invert_output: false,
        update_rate_ms: timing::MEDIUM_UPDATE_MS,
    }
}

pub fn voltage_to_speed_config() -> PwmControllerConfig {
    PwmControllerConfig {
        min_voltage_mv: 500,  // 0.5V minimum for motor start
        max_voltage_mv: 3000, // 3.0V maximum
        invert_output: false,
        update_rate_ms: 100,
    }
}

pub fn inverted_control_config() -> PwmControllerConfig {
    PwmControllerConfig {
        min_voltage_mv: 0,
        max_voltage_mv: voltage::VCC_3V3_MV,
        invert_output: true, // Higher voltage = lower PWM
        update_rate_ms: timing::MEDIUM_UPDATE_MS,
    }
}
