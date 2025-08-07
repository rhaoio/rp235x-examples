use core::f32::consts::PI;

use embassy_rp::gpio::Output;
use embassy_time::Duration;
use num_traits::float::FloatCore;

pub async fn set_tone_value(pin: &mut Output<'_>) {
    let mut sin_value = 0.0;
    // Tone_value is frequency;
    let mut tone_value = 0;

    for i in 0..36 {
        let v = i as f32 * 10.0;
        sin_value = libm::sin((v * PI / 180.0).into());
        tone_value += 2000 + (sin_value.round()) as i32 * 500;
        if tone_value == 0 {
            let _ = pin.set_level(embassy_rp::gpio::Level::Low);
        } else {
            for j in 0..(10 * tone_value / 1000) {
                let _ = pin.set_level(embassy_rp::gpio::Level::High);
                // convert tone freq into u64
                let freq: u64 = tone_value as u64;
                let delay = 100000 / freq / 2;
                embassy_time::Timer::after(Duration::from_millis(delay)).await;
                let _ = pin.set_level(embassy_rp::gpio::Level::Low);
                embassy_time::Timer::after(Duration::from_millis(delay)).await;
            }
        }
    }
}
