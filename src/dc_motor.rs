use defmt::*;
use embassy_rp::{
    adc::{Adc, Channel, Config as AdcConfig},
    gpio::{Level, Output},
    peripherals::{ADC, PIN_26, PWM_SLICE0},
    pwm::{Config as PwmConfig, Pwm, SetDutyCycle},
    Peri,
};
use embassy_time::{Duration, Timer};
use heapless::String;

use crate::adc_reader::AdcIrqs;
use crate::constants::{adc, joystick, pwm, timing, voltage};
use crate::usb_serial::{get_serial_sender, send_sync_formatted_message, SerialMessage};

pub struct DCMotor<'a> {
    input1: Output<'a>,
    input2: Output<'a>,
    enable_pwm: Pwm<'a>, // PWM for speed control on Enable pin
}

impl<'a> DCMotor<'a> {
    pub fn new(
        input1: Output<'a>,
        input2: Output<'a>,
        pwm_slice: Peri<'a, PWM_SLICE0>,
        enable_pin: Peri<'a, embassy_rp::peripherals::PIN_17>, // GPIO 17 for Enable (PWM)
    ) -> Self {
        // Configure PWM for speed control on Enable pin
        let mut pwm_config = PwmConfig::default();
        pwm_config.top = pwm::STANDARD_TOP_VALUE; // 0-1000 range for easy percentage calculation
        pwm_config.compare_b = 0; // Start at 0% duty cycle (stopped) - use compare_b for output_b
        pwm_config.enable = true; // Enable the PWM

        let mut enable_pwm = Pwm::new_output_b(pwm_slice, enable_pin, pwm_config);
        enable_pwm.set_duty_cycle_percent(0); // Start with motor stopped (0% duty cycle)

        Self {
            input1,
            input2,
            enable_pwm,
        }
    }

    /// Set motor direction and speed
    /// speed: 0-100 (percentage)
    /// forward: true for forward, false for reverse
    pub async fn set_direction_and_speed(&mut self, forward: bool, speed: u8) {
        let speed = speed.min(100); // Clamp to 0-100%

        if speed == 0 {
            self.stop().await;
            return;
        }

        // Set direction using Input1 and Input2
        if forward {
            self.input1.set_high();
            self.input2.set_low();
        } else {
            self.input1.set_low();
            self.input2.set_high();
        }
        let mut msg: String<128> = String::new();
        let _ = core::fmt::write(
            &mut msg,
            format_args!(
                "[MOTOR] SET DIRECTION AND SPEED - Forward: {}, Speed: {}%",
                forward, speed
            ),
        );
        send_sync_formatted_message(msg);
        self.enable_pwm.set_duty_cycle_percent(speed);
    }

    pub async fn stop(&mut self) {
        // Stop by setting Enable PWM to 0% (motor won't run regardless of Input1/Input2)
        self.enable_pwm.set_duty_cycle_percent(0);

        // Also set both inputs low for good measure
        self.input1.set_low();
        self.input2.set_low();
    }

    /// Convert raw ADC value to millivolts for display
    pub fn adc_to_voltage_mv(&self, adc_value: u16) -> u16 {
        adc::raw_to_millivolts(adc_value)
    }

    /// Control logic for motor direction and speed based on ADC input
    pub fn get_motor_control(&self, adc_value: u16) -> (Option<bool>, u8) {
        const DEAD_ZONE: u16 = joystick::DEAD_ZONE_ADC; // Dead zone around center (about 0.08V)
        const CENTER: u16 = adc::CENTER_VALUE; // Center position (4095/2)

        if adc_value >= (CENTER - DEAD_ZONE) && adc_value <= (CENTER + DEAD_ZONE) {
            // In dead zone - stop motor
            (None, 0)
        } else if adc_value < (CENTER - DEAD_ZONE) {
            // Left side of center - REVERSE direction
            let deviation = (CENTER - DEAD_ZONE) - adc_value;
            let max_deviation = (CENTER - DEAD_ZONE); // Maximum possible deviation to the left
            let speed_percent = ((deviation as u32 * 100) / max_deviation as u32).min(100) as u8;
            (Some(false), speed_percent) // false = reverse
        } else {
            // Right side of center - FORWARD direction
            let deviation = adc_value - (CENTER + DEAD_ZONE);
            let max_deviation = adc::MAX_VALUE - (CENTER + DEAD_ZONE); // Maximum possible deviation to the right
            let speed_percent = ((deviation as u32 * 100) / max_deviation as u32).min(100) as u8;
            (Some(true), speed_percent) // true = forward
        }
    }

    /// Format speed and direction for display
    pub fn format_motor_state(&self, direction: Option<bool>, speed: u8) -> &'static str {
        match direction {
            Some(true) => "FORWARD",
            Some(false) => "REVERSE",
            None => "STOP",
        }
    }
}

/// DC Motor control task with PWM speed control
#[embassy_executor::task]
pub async fn dc_motor_task(
    adc_peripheral: Peri<'static, ADC>,
    adc_pin: Peri<'static, PIN_26>,
    input1_pin: Peri<'static, embassy_rp::peripherals::PIN_15>,
    input2_pin: Peri<'static, embassy_rp::peripherals::PIN_16>, // Changed to PIN_17
    pwm_slice: Peri<'static, PWM_SLICE0>,
    enable_pin: Peri<'static, embassy_rp::peripherals::PIN_17>, // GPIO 16 for PWM Enable
) -> ! {
    info!("Starting DC Motor control task with PWM speed control");

    // Send startup message using sync version
    let sender = get_serial_sender();
    let _ = sender.try_send(SerialMessage::Text(
        "DC Motor with PWM speed control - Input1: GPIO15, Input2: GPIO17, Enable: GPIO16 (PWM)",
    ));

    // Initialize motor control pins
    let input1 = Output::new(input1_pin, Level::Low);
    let input2 = Output::new(input2_pin, Level::Low);

    // Create motor controller with PWM speed control
    let mut motor = DCMotor::new(input1, input2, pwm_slice, enable_pin);

    // Initialize ADC with proper configuration
    let adc_config = AdcConfig::default();
    let mut adc = Adc::new(adc_peripheral, AdcIrqs, adc_config);

    // Configure ADC pin
    let mut adc_channel = Channel::new_pin(adc_pin, embassy_rp::gpio::Pull::None);

    let mut last_control: (Option<bool>, u8) = (None, 0);

    loop {
        // Read potentiometer value
        let adc_value = adc.read(&mut adc_channel).await.unwrap_or(0);
        let voltage_mv = motor.adc_to_voltage_mv(adc_value);
        let (direction, speed_percent) = motor.get_motor_control(adc_value);

        // Only change motor state if control changed significantly
        if (direction, speed_percent) != last_control {
            match direction {
                Some(true) => {
                    // Forward direction with variable speed
                    motor.set_direction_and_speed(true, speed_percent).await;
                    let mut msg: String<128> = String::new();
                    let _ = core::fmt::write(
                        &mut msg,
                        format_args!(
                            "[MOTOR] FORWARD {}% - Pot: {}.{:03}V (raw: {})",
                            speed_percent,
                            voltage_mv / voltage::MV_PER_VOLT,
                            voltage_mv % voltage::MV_PER_VOLT,
                            adc_value
                        ),
                    );
                    send_sync_formatted_message(msg);
                }
                Some(false) => {
                    // Reverse direction with variable speed
                    motor.set_direction_and_speed(false, speed_percent).await;
                    let mut msg: String<128> = String::new();
                    let _ = core::fmt::write(
                        &mut msg,
                        format_args!(
                            "[MOTOR] REVERSE {}% - Pot: {}.{:03}V (raw: {})",
                            speed_percent,
                            voltage_mv / voltage::MV_PER_VOLT,
                            voltage_mv % voltage::MV_PER_VOLT,
                            adc_value
                        ),
                    );
                    send_sync_formatted_message(msg);
                }
                None => {
                    // Stop motor
                    motor.stop().await;
                    let mut msg: String<128> = String::new();
                    let _ = core::fmt::write(
                        &mut msg,
                        format_args!(
                            "[MOTOR] STOP - Pot: {}.{:03}V (raw: {}) - Center position",
                            voltage_mv / voltage::MV_PER_VOLT,
                            voltage_mv % voltage::MV_PER_VOLT,
                            adc_value
                        ),
                    );
                    send_sync_formatted_message(msg);
                }
            }
            last_control = (direction, speed_percent);
        }

        // Read every 50ms for responsive speed control
        Timer::after(Duration::from_millis(timing::MEDIUM_UPDATE_MS * 5)).await;
    }
}
