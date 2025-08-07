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
    embassy_rp::binary_info::rp_program_name!(c"ADC Reader Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"ADC reader with temperature sensor example copied from main.rs on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Include necessary modules
use rp_pico2_examples::adc_reader;
use rp_pico2_examples::usb_serial;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("ADC Reader Example Starting");

    info!("Pin connections:");
    info!("  Analog input: GPIO 26 (ADC0)");
    info!("  Temperature sensor: Built-in");

    // Start USB Serial task
    unwrap!(spawner.spawn(usb_serial::usb_serial_task(p.USB)));

    // Start ADC reader task for monitoring analog inputs
    unwrap!(spawner.spawn(adc_reader::adc_reader_task(
        p.ADC,
        p.PIN_26,
        p.ADC_TEMP_SENSOR
    )));

    // Keep main task alive
    loop {
        embassy_time::Timer::after(embassy_time::Duration::from_millis(1000)).await;
    }
}
