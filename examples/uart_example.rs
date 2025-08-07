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
    embassy_rp::binary_info::rp_program_name!(c"UART Communication Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"UART serial communication example copied from main.rs on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Include necessary modules
use rp_pico2_examples::uart;
use rp_pico2_examples::usb_serial;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("UART Communication Example Starting");

    info!("Pin connections:");
    info!("  UART TX: GPIO 4");
    info!("  UART RX: GPIO 5");
    info!("  Baud rate: 115200 (default)");
    info!("  Connect to another UART device or USB-to-serial adapter");

    // Start USB Serial task for debugging
    unwrap!(spawner.spawn(usb_serial::usb_serial_task(p.USB)));

    // Start UART communication task
    unwrap!(spawner.spawn(uart::uart_task(p.UART1, p.PIN_4, p.PIN_5)));

    // Keep main task alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
