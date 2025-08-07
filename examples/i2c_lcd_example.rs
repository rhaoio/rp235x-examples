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
    embassy_rp::binary_info::rp_program_name!(c"I2C LCD Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"I2C LCD display example copied from main.rs on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Include necessary modules inline
use rp_pico2_examples::i2c_lcd;
use rp_pico2_examples::usb_serial;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("I2C LCD Example Starting");

    info!("Pin connections:");
    info!("  I2C SDA: GPIO 4");
    info!("  I2C SCL: GPIO 5");
    info!("  I2C LCD display");

    // Start USB Serial task
    unwrap!(spawner.spawn(usb_serial::usb_serial_task(p.USB)));

    // ===== I2C LCD =====
    unwrap!(spawner.spawn(i2c_lcd::lcd_demo_task(p.I2C0, p.PIN_4, p.PIN_5)));

    // Keep main task alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
