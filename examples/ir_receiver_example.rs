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
    embassy_rp::binary_info::rp_program_name!(c"IR Receiver Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"IR receiver with LED and buzzer control example copied from main.rs on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Include necessary modules
use rp_pico2_examples::ir_receiver;
use rp_pico2_examples::usb_serial;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("IR Receiver Example Starting");

    info!("Pin connections:");
    info!("  IR Receiver: GPIO 16");
    info!("  LED (PWM): GPIO 14");
    info!("  Buzzer: GPIO 15");

    // Start USB Serial task
    unwrap!(spawner.spawn(usb_serial::usb_serial_task(p.USB)));

    // ===== IR RECEIVER =====
    // IR receiver on GPIO 16 - decodes remote control signals
    // Also controls an LED on GPIO 14 (PWM) and a buzzer on GPIO 15.
    unwrap!(spawner.spawn(ir_receiver::ir_receiver_task(
        p.PIN_16,
        p.PIN_14,
        p.PIN_15,
        p.PWM_SLICE7
    )));

    // Keep main task alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
