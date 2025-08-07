#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_time::{Duration, Timer};

use {defmt_rtt as _, panic_probe as _};

// Include the buzzer module inline
mod buzzer {
    use core::f32::consts::PI;
    use embassy_rp::gpio::Output;
    use embassy_time::Duration;
    use num_traits::float::FloatCore;

    pub async fn set_tone_value(pin: &mut Output<'_>) {
        let mut tone_value = 0;

        for i in 0..36 {
            let v = i as f32 * 10.0;
            let sin_value = libm::sin((v * PI / 180.0).into());
            tone_value += 2000 + (sin_value.round()) as i32 * 500;
            if tone_value == 0 {
                pin.set_level(embassy_rp::gpio::Level::Low);
            } else {
                for _j in 0..(10 * tone_value / 1000) {
                    pin.set_level(embassy_rp::gpio::Level::High);
                    let freq: u64 = tone_value as u64;
                    let delay = 100000 / freq / 2;
                    embassy_time::Timer::after(Duration::from_millis(delay)).await;
                    pin.set_level(embassy_rp::gpio::Level::Low);
                    embassy_time::Timer::after(Duration::from_millis(delay)).await;
                }
            }
        }
    }
}

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Buzzer Tone Generator Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"Simple buzzer tone generator example copied from main.rs on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Buzzer Tone Generator Example Starting");

    info!("Pin connections:");
    info!("  Buzzer: GPIO 15");
    info!("  Button: GPIO 16");
    info!("  Press button to generate tone");

    // Buzzer related pins - copied exactly from main.rs
    let mut buzzer_pin = Output::new(p.PIN_15, Level::Low);
    let btn = Input::new(p.PIN_16, Pull::None);

    loop {
        let level = btn.get_level();
        if level == Level::Low {
            buzzer::set_tone_value(&mut buzzer_pin).await;
        } else {
            buzzer_pin.set_low();
        }

        // Small delay to prevent tight loop
        Timer::after(Duration::from_millis(10)).await;
    }
}
