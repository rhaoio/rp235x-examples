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
    embassy_rp::binary_info::rp_program_name!(c"Joystick Control Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"Analog joystick reader example copied from main.rs on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Include necessary modules
use rp_pico2_examples::joystick;
use rp_pico2_examples::usb_serial;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Joystick Control Example Starting");

    info!("Pin connections:");
    info!("  X-axis: GPIO 26 (ADC0)");
    info!("  Y-axis: GPIO 27 (ADC1)");
    info!("  Z-axis/button: GPIO 28 (ADC2)");

    // Start USB Serial task
    unwrap!(spawner.spawn(usb_serial::usb_serial_task(p.USB)));

    // ===== JOYSTICK ADC READER =====
    // Start joystick reader task that monitors pins 26, 27, and 28
    // Note: Only one ADC task can run at a time due to interrupt handler conflicts
    unwrap!(spawner.spawn(joystick::joystick_reader_task(
        p.ADC, p.PIN_26, // X-axis
        p.PIN_27, // Y-axis
        p.PIN_28  // Z-axis/button
    )));

    // Start joystick consumer task to process the readings
    unwrap!(spawner.spawn(joystick::joystick_consumer_task()));

    // Keep main task alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
