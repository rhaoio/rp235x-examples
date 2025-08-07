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
    embassy_rp::binary_info::rp_program_name!(c"74HC595 Shift Register Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"74HC595 shift register LED patterns example copied from main.rs on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Include necessary modules
use rp_pico2_examples::shift_register;
use rp_pico2_examples::usb_serial;
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("74HC595 Shift Register Example Starting");

    info!("Pin connections:");
    info!("  DS (Data Serial): GPIO 18");
    info!("  ST_CP (Latch): GPIO 20");
    info!("  SH_CP (Clock): GPIO 21");
    info!("  Connect LEDs to Q0-Q7 outputs with appropriate resistors");

    // Start USB Serial task
    unwrap!(spawner.spawn(usb_serial::usb_serial_task(p.USB)));

    // ===== 74HC595 SHIFT REGISTER DEMOS =====
    // LED Bar Demo - VU meter pattern with 8 individual LEDs
    unwrap!(spawner.spawn(shift_register::shift_register_task(
        p.PIN_18, // DS (Data Serial)
        p.PIN_20, // ST_CP (Storage Register Clock/Latch)
        p.PIN_21  // SH_CP (Shift Register Clock)
    )));

    // Keep main task alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
