#![no_std]
#![no_main]

use defmt::*;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::pwm::SetDutyCycle;
use embassy_time::{Duration, Timer};

pub async fn rgb_led(
    led0: &mut impl SetDutyCycle,
    led1: &mut impl SetDutyCycle,
    led2: &mut impl SetDutyCycle,
) -> ! {
    let max_duty = led0.max_duty_cycle();
    info!("RGB LED max duty cycle: {}", max_duty);

    let mut wheel_pos = 0u8;
    let mut counter = 0u16;

    loop {
        let (r, g, b) = six_segment_color_wheel(wheel_pos);

        // Scale to full duty cycle range
        let r_scaled = (r as u32 * max_duty as u32 / 255) as u16;
        let g_scaled = (g as u32 * max_duty as u32 / 255) as u16;
        let b_scaled = (b as u32 * max_duty as u32 / 255) as u16;

        // Set duty cycles
        let _ = led0.set_duty_cycle(r_scaled);
        let _ = led1.set_duty_cycle(g_scaled);
        let _ = led2.set_duty_cycle(b_scaled);

        // Slower increments for smoother, longer transitions
        counter = counter.wrapping_add(1);
        if counter % 4 == 0 {
            // Update color every 4 cycles (80ms)
            wheel_pos = wheel_pos.wrapping_add(1);
        }

        // Fast update rate for smooth PWM
        Timer::after(Duration::from_millis(20)).await;
    }
}

/// Simple color wheel function (FastLED style)
/// Creates smooth rainbow transitions without complex HSV math
/// position: 0-255 where 0=red, 85=green, 170=blue, 255=back to red
/// Returns (r, g, b) where each component is 0-255
fn smooth_color_wheel(position: u8) -> (u8, u8, u8) {
    // Use 85 steps per segment for integer math (255 total positions)
    // This gives us 3 segments of 85 steps each
    match position {
        0..=84 => {
            // Red to Green transition
            let red = 255 - (position * 3);
            let green = position * 3;
            (red, green, 0)
        }
        85..=169 => {
            // Green to Blue transition
            let pos = position - 85;
            let green = 255 - (pos * 3);
            let blue = pos * 3;
            (0, green, blue)
        }
        170..=254 => {
            // Blue to Red transition
            let pos = position - 170;
            let blue = 255 - (pos * 3);
            let red = pos * 3;
            (red, 0, blue)
        }
        255 => (255, 0, 0), // Wrap back to pure red
    }
}

pub fn six_segment_color_wheel(position: u8) -> (u8, u8, u8) {
    // 6 segments of ~42.5 steps each (255 ÷ 6 = 42.5, so we use 42-43 steps)
    // Segments: Red->Yellow->Green->Cyan->Blue->Magenta->Red
    match position {
        0..=42 => {
            // Red to Yellow (increase Green, keep Red at max)
            let green = position * 6; // 0 to 252
            (255, green, 0)
        }
        43..=85 => {
            // Yellow to Green (decrease Red, keep Green at max)
            let pos = position - 43;
            let red = 255 - (pos * 6); // 255 to 3
            (red, 255, 0)
        }
        86..=128 => {
            // Green to Cyan (increase Blue, keep Green at max)
            let pos = position - 86;
            let blue = pos * 6; // 0 to 252
            (0, 255, blue)
        }
        129..=171 => {
            // Cyan to Blue (decrease Green, keep Blue at max)
            let pos = position - 129;
            let green = 255 - (pos * 6); // 255 to 3
            (0, green, 255)
        }
        172..=214 => {
            // Blue to Magenta (increase Red, keep Blue at max)
            let pos = position - 172;
            let red = pos * 6; // 0 to 252
            (red, 0, 255)
        }
        215..=255 => {
            // Magenta to Red (decrease Blue, keep Red at max)
            let pos = position - 215;
            let blue = 255 - (pos * 6); // 255 to max(0, 255-246)
            (255, 0, blue.max(0))
        }
    }
}
/// VU Meter pattern function using digital outputs
/// Shows a progressive lighting pattern from LED 0 to LED 9
pub async fn vu_meter_pattern(
    led0: &mut Output<'static>,
    led1: &mut Output<'static>,
    led2: &mut Output<'static>,
    led3: &mut Output<'static>,
    led4: &mut Output<'static>,
    led5: &mut Output<'static>,
    led6: &mut Output<'static>,
    led7: &mut Output<'static>,
    led8: &mut Output<'static>,
    led9: &mut Output<'static>,
) -> ! {
    let mut current_level = 0;
    let delay = Duration::from_millis(500);

    info!("Starting VU Meter pattern");

    loop {
        info!("VU Level: {}", current_level);

        // VU Meter pattern - light up LEDs progressively based on level
        led0.set_level(if current_level > 0 {
            Level::High
        } else {
            Level::Low
        });
        led1.set_level(if current_level > 1 {
            Level::High
        } else {
            Level::Low
        });
        led2.set_level(if current_level > 2 {
            Level::High
        } else {
            Level::Low
        });
        led3.set_level(if current_level > 3 {
            Level::High
        } else {
            Level::Low
        });
        led4.set_level(if current_level > 4 {
            Level::High
        } else {
            Level::Low
        });
        led5.set_level(if current_level > 5 {
            Level::High
        } else {
            Level::Low
        });
        led6.set_level(if current_level > 6 {
            Level::High
        } else {
            Level::Low
        });
        led7.set_level(if current_level > 7 {
            Level::High
        } else {
            Level::Low
        });
        led8.set_level(if current_level > 8 {
            Level::High
        } else {
            Level::Low
        });
        led9.set_level(if current_level > 9 {
            Level::High
        } else {
            Level::Low
        });

        current_level = (current_level + 1) % 11; // 0-10 levels
        Timer::after(delay).await;
    }
}

/// PWM breathing pattern function
/// Creates a smooth "breathing" effect where all LEDs fade in and out together
/// Works with both Pwm and PwmOutput types
pub async fn pwm_breathing_pattern<T>(pwm0: &mut T) -> !
where
    T: SetDutyCycle,
{
    info!("Starting PWM breathing pattern");

    let mut brightness = 0u16;
    let mut direction = 1i16;
    let step_size = 200u16;
    let max_brightness = pwm0.max_duty_cycle();
    let delay = Duration::from_millis(20);

    info!("Max duty cycle: {}", max_brightness);

    loop {
        // Update brightness
        if direction > 0 {
            brightness = brightness.saturating_add(step_size);
            if brightness >= max_brightness {
                brightness = max_brightness;
                direction = -1;
            }
        } else {
            brightness = brightness.saturating_sub(step_size);
            if brightness == 0 {
                direction = 1;
            }
        }

        // Set PWM duty cycle for all channels (breathing effect)
        let _ = pwm0.set_duty_cycle(brightness);

        Timer::after(delay).await;
    }
}

const METEOR_DUTY_VALUES: [u16; 30] = [
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 20000, 10000, 5000, 2500, 1250, 625, 312, 156, 78, 39, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0,
];

fn meteor_duty_cycles(position: u8, direction: i8) -> [u16; 10] {
    let mut duty_cycles = [0; 10];

    // Calculate the meteor head position and tail
    for i in 0..10 {
        let led_index = i as i8;

        // Calculate offset from meteor position (distance from meteor head)
        let offset = (position as i8) - led_index;

        // Map to duty values array (meteor head is at index 10 in METEOR_DUTY_VALUES)
        // When offset is 0, we want the bright head (index 10)
        // When offset is positive, we want the tail (indices 11, 12, 13, ...)
        // When offset is negative, the meteor hasn't reached this LED yet
        let duty_index = (10 + offset) as usize;

        if duty_index < METEOR_DUTY_VALUES.len() {
            duty_cycles[led_index as usize] = METEOR_DUTY_VALUES[duty_index];
        } else {
            duty_cycles[led_index as usize] = 0;
        }
    }

    duty_cycles
}

/// Creates a meteor shower pattern across 10 LEDs (original version)
/// Works with both Pwm and PwmOutput types
pub async fn meteor_shower_pattern<T0, T1, T2, T3, T4, T5, T6, T7, T8, T9>(
    pwm0: &mut T0,
    pwm1: &mut T1,
    pwm2: &mut T2,
    pwm3: &mut T3,
    pwm4: &mut T4,
    pwm5: &mut T5,
    pwm6: &mut T6,
    pwm7: &mut T7,
    pwm8: &mut T8,
    pwm9: &mut T9,
) -> !
where
    T0: SetDutyCycle,
    T1: SetDutyCycle,
    T2: SetDutyCycle,
    T3: SetDutyCycle,
    T4: SetDutyCycle,
    T5: SetDutyCycle,
    T6: SetDutyCycle,
    T7: SetDutyCycle,
    T8: SetDutyCycle,
    T9: SetDutyCycle,
{
    info!("Starting meteor shower pattern (10 LEDs)");

    let delay = Duration::from_millis(50);
    let mut position = 0i8;
    let mut direction = 1i8;

    // Allow meteor to fully trail off before changing direction
    // Position range: -10 to 20 (gives space for full tail on both sides)
    let min_position = -10i8;
    let max_position = 20i8;

    loop {
        // Get duty cycles for current position and direction
        let duty_cycles = if direction > 0 {
            // Moving left to right
            meteor_duty_cycles(position as u8, direction)
        } else {
            // Moving right to left - reverse the LED order
            let forward_cycles = meteor_duty_cycles((9 - position) as u8, 1);
            let mut reversed_cycles = [0u16; 10];
            for i in 0..10 {
                reversed_cycles[i] = forward_cycles[9 - i];
            }
            reversed_cycles
        };

        // Set PWM duty cycles for all LEDs
        let _ = pwm0.set_duty_cycle(duty_cycles[0]);
        let _ = pwm1.set_duty_cycle(duty_cycles[1]);
        let _ = pwm2.set_duty_cycle(duty_cycles[2]);
        let _ = pwm3.set_duty_cycle(duty_cycles[3]);
        let _ = pwm4.set_duty_cycle(duty_cycles[4]);
        let _ = pwm5.set_duty_cycle(duty_cycles[5]);
        let _ = pwm6.set_duty_cycle(duty_cycles[6]);
        let _ = pwm7.set_duty_cycle(duty_cycles[7]);
        let _ = pwm8.set_duty_cycle(duty_cycles[8]);
        let _ = pwm9.set_duty_cycle(duty_cycles[9]);

        // Update position
        position += direction;

        // Change direction when meteor has fully trailed off
        if position > max_position {
            direction = -1;
            position = max_position;
        } else if position < min_position {
            direction = 1;
            position = min_position;
        }

        Timer::after(delay).await;
    }
}
