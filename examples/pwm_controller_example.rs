#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;

use {defmt_rtt as _, panic_probe as _};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"PWM Controller Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"ADC-controlled PWM output example copied from main.rs on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Include necessary modules
use rp_pico2_examples::adc_reader;
use rp_pico2_examples::pwm_controller;
use rp_pico2_examples::usb_serial;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("PWM Controller Example Starting");

    info!("Pin connections:");
    info!("  PWM Output: GPIO 15");
    info!("  ADC Input: GPIO 26 (potentiometer)");
    info!("  Connect LED with resistor to PWM output");

    // Start USB Serial task
    unwrap!(spawner.spawn(usb_serial::usb_serial_task(p.USB)));

    // ===== CHANNEL-BASED ADC TO PWM CONTROL EXAMPLES =====
    // Single ADC channel controlling single PWM output
    unwrap!(spawner.spawn(adc_reader::single_adc_to_channel_task(
        p.ADC, p.PIN_26 // ADC input pin
    )));

    unwrap!(spawner.spawn(pwm_controller::single_pwm_controller_task(
        p.PWM_SLICE7,
        p.PIN_15, // PWM output pin
        pwm_controller::voltage_to_brightness_config()
    )));

    // Keep main task alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
