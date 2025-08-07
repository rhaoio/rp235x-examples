use crate::adc_reader::AdcIrqs;
use crate::constants::{adc, servo as servo_const, system, timing};
use crate::usb_serial::send_sync_formatted_message;

use embassy_rp::Peri;
use embassy_rp::{
    adc::{Adc, Channel, Config as AdcConfig},
    peripherals::{ADC, PIN_16, PIN_26, PWM_SLICE0},
    pwm::{Config as PwmConfig, Pwm, SetDutyCycle},
};
use embassy_time::{Duration, Timer};
use heapless::String;

pub struct Servo<'a> {
    pwm: Pwm<'a>,
}

impl<'a> Servo<'a> {
    pub fn new(
        pwm_slice: Peri<'static, PWM_SLICE0>,
        pin: Peri<'static, embassy_rp::peripherals::PIN_16>,
    ) -> Self {
        // Servo PWM configuration for 50Hz (20ms period)
        // System clock is 125MHz
        // For 50Hz: period = 20ms = 0.02s
        // We need divider and top such that: 125MHz / divider / (top + 1) = 50Hz
        // Let's use divider = 125.0, then: 125MHz / 125 = 1MHz
        // For 50Hz with 1MHz: top = 1MHz / 50Hz - 1 = 20000 - 1 = 19999

        let mut config = PwmConfig::default();
        config.divider = (system::PWM_CLOCK_DIVIDER as u8).into();
        config.top = servo_const::PWM_TOP_VALUE;
        config.enable = true;

        let pwm = Pwm::new_output_a(pwm_slice, pin, config);

        Self { pwm }
    }

    /// Set servo angle.
    ///
    /// # Arguments
    ///
    /// * `angle` - The angle to set the servo to, from 0.0 to 180.0 degrees.
    ///             Values outside this range will be clamped.
    pub fn set_angle(&mut self, angle: f32) {
        let angle = angle.clamp(
            servo_const::MIN_ANGLE_DEGREES,
            servo_const::MAX_ANGLE_DEGREES,
        );

        // Convert angle to pulse width in microseconds
        // 0° = 500us, 180° = 2500us
        let pulse_width_us = servo_const::MIN_PULSE_WIDTH_US as f32
            + (angle / servo_const::MAX_ANGLE_DEGREES)
                * (servo_const::MAX_PULSE_WIDTH_US - servo_const::MIN_PULSE_WIDTH_US) as f32;

        // Convert pulse width to duty cycle percentage for 20ms period
        // pulse_width_us / 20000us * 100% = duty cycle percentage
        let duty_percent = (pulse_width_us / servo_const::PWM_PERIOD_US as f32 * 100.0) as u8;

        // Set the PWM duty cycle
        let _ = self.pwm.set_duty_cycle_percent(duty_percent);

        let mut msg = String::<128>::new();
        let _ = core::fmt::write(
            &mut msg,
            format_args!(
                "Servo angle: {}° -> {}us pulse width ({}% duty)",
                angle, pulse_width_us as u32, duty_percent
            ),
        );
        send_sync_formatted_message(msg);
    }
}

#[embassy_executor::task]
pub async fn servo_task(
    pwm_slice: Peri<'static, PWM_SLICE0>,
    pin: Peri<'static, embassy_rp::peripherals::PIN_16>,
) -> ! {
    let mut msg = String::<128>::new();
    let _ = core::fmt::write(
        &mut msg,
        format_args!("Starting servo task - Using 50Hz PWM with proper pulse widths"),
    );
    send_sync_formatted_message(msg);

    let mut servo = Servo::new(pwm_slice, pin);

    let mut msg = String::<128>::new();
    let _ = core::fmt::write(&mut msg, format_args!("Servo set to 0 degrees"));
    send_sync_formatted_message(msg);
    servo.set_angle(servo_const::MIN_ANGLE_DEGREES);
    Timer::after(Duration::from_secs(2)).await;

    let mut msg = String::<128>::new();
    let _ = core::fmt::write(&mut msg, format_args!("Servo set to 90 degrees"));
    send_sync_formatted_message(msg);
    servo.set_angle(servo_const::MAX_ANGLE_DEGREES / 2.0);
    Timer::after(Duration::from_secs(2)).await;

    let mut msg = String::<128>::new();
    let _ = core::fmt::write(&mut msg, format_args!("Servo set to 180 degrees"));
    send_sync_formatted_message(msg);
    servo.set_angle(servo_const::MAX_ANGLE_DEGREES);
    Timer::after(Duration::from_secs(2)).await;

    loop {
        let mut msg = String::<128>::new();
        let _ = core::fmt::write(
            &mut msg,
            format_args!("Sweeping servo from 0 to 180 degrees"),
        );
        send_sync_formatted_message(msg);
        for angle in 0..=(servo_const::MAX_ANGLE_DEGREES as u32) {
            servo.set_angle(angle as f32);
            Timer::after(Duration::from_millis(timing::MEDIUM_UPDATE_MS)).await;
        }
        Timer::after(Duration::from_millis(timing::LONG_DELAY_MS)).await;

        let mut msg = String::<128>::new();
        let _ = core::fmt::write(
            &mut msg,
            format_args!("Sweeping servo from 180 to 0 degrees"),
        );
        send_sync_formatted_message(msg);
        for angle in (0..=(servo_const::MAX_ANGLE_DEGREES as u32)).rev() {
            servo.set_angle(angle as f32);
            Timer::after(Duration::from_millis(timing::MEDIUM_UPDATE_MS)).await;
        }
        Timer::after(Duration::from_millis(timing::LONG_DELAY_MS)).await;
    }
}

/// Task that controls servo angle based on potentiometer (ADC) input
#[embassy_executor::task]
pub async fn servo_adc_control_task(
    adc_peripheral: Peri<'static, ADC>,
    adc_pin: Peri<'static, PIN_26>,
    pwm_slice: Peri<'static, PWM_SLICE0>,
    pwm_pin: Peri<'static, embassy_rp::peripherals::PIN_16>,
) -> ! {
    let adc_config = AdcConfig::default();
    let mut adc = Adc::new(adc_peripheral, AdcIrqs, adc_config);
    let mut adc_channel = Channel::new_pin(adc_pin, embassy_rp::gpio::Pull::None);

    let mut servo = Servo::new(pwm_slice, pwm_pin);

    loop {
        let raw_adc = adc.read(&mut adc_channel).await.unwrap_or(0);
        let voltage_mv = adc::raw_to_millivolts(raw_adc);

        // Map ADC value (0-4095) to angle (0-180)
        let angle = (raw_adc as f32 / adc::MAX_VALUE as f32) * servo_const::MAX_ANGLE_DEGREES;
        servo.set_angle(angle);

        Timer::after(Duration::from_millis(timing::FAST_UPDATE_MS)).await;
    }
}

#[embassy_executor::task]
pub async fn servo_sweep_task(
    pwm_slice: Peri<'static, PWM_SLICE0>,
    pwm_pin: Peri<'static, embassy_rp::peripherals::PIN_16>,
) -> ! {
    let mut servo = Servo::new(pwm_slice, pwm_pin);

    loop {
        for angle in (0..=180) {
            servo.set_angle(angle as f32);

            Timer::after(Duration::from_millis(50)).await;
        }
        Timer::after(Duration::from_millis(500)).await;

        for angle in (0..=180).rev() {
            servo.set_angle(angle as f32);

            Timer::after(Duration::from_millis(50)).await;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}
