#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::block::ImageDef;

use {defmt_rtt as _, panic_probe as _};

use rp_pico2_examples::dc_motor;
use rp_pico2_examples::usb_serial;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"DC Motor Control Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"DC motor with L293D driver and potentiometer control example copied from main.rs on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("DC Motor Control Example Starting");

    info!("Pin connections:");
    info!("  L293D Input1: GPIO 15");
    info!("  L293D Input2: GPIO 16");
    info!("  L293D Enable (PWM): GPIO 17");
    info!("  Potentiometer: GPIO 26 (ADC)");

    // Start USB Serial task
    unwrap!(spawner.spawn(usb_serial::usb_serial_task(p.USB)));

    // ===== DC MOTOR CONTROL =====
    // DC Motor with L293D driver and potentiometer control
    // L293D Input1: GPIO 15, Input2: GPIO 16, Potentiometer: GPIO 26 (ADC)
    unwrap!(spawner.spawn(dc_motor::dc_motor_task(
        p.ADC,
        p.PIN_26,
        p.PIN_15,
        p.PIN_16,
        p.PWM_SLICE0,
        p.PIN_17
    )));

    // Keep main task alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
