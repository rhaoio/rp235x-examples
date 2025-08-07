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
    embassy_rp::binary_info::rp_program_name!(c"Servo Control Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"Servo control with potentiometer example copied from main.rs on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

use rp_pico2_examples::servo;
use rp_pico2_examples::usb_serial;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Servo Control Example Starting");

    info!("Pin connections:");
    info!("  Servo Signal: GPIO 16 (PWM)");
    info!("  Potentiometer: GPIO 26 (ADC)");
    info!("  Servo VCC: 5V, GND: Ground");

    // Start USB Serial task
    unwrap!(spawner.spawn(usb_serial::usb_serial_task(p.USB)));

    // ===== SERVO CONTROL =====
    // Servo control using GPIO 16 for PWM signal
    unwrap!(spawner.spawn(servo::servo_adc_control_task(
        p.ADC,
        p.PIN_26,
        p.PWM_SLICE0,
        p.PIN_16
    )));

    // Keep main task alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
