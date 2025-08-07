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
    embassy_rp::binary_info::rp_program_name!(c"Ultrasonic Sensor Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"HC-SR04 ultrasonic sensor example copied from main.rs on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Include necessary modules
use rp_pico2_examples::ultrasonic;
use rp_pico2_examples::usb_serial;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Ultrasonic Sensor Example Starting");

    info!("Pin connections:");
    info!("  Trigger: GPIO 19");
    info!("  Echo: GPIO 18");
    info!("  HC-SR04 ultrasonic sensor");

    // Start USB Serial task for debugging
    unwrap!(spawner.spawn(usb_serial::usb_serial_task(p.USB)));

    // ===== ULTRASONIC SENSOR =====
    // HC-SR04 ultrasonic sensor: Trig on GPIO 19, Echo on GPIO 18
    unwrap!(spawner.spawn(ultrasonic::ultrasonic_task(p.PIN_19, p.PIN_18)));

    // Keep main task alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
