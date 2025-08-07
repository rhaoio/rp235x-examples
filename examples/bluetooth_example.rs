#![no_std]
#![no_main]

use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;

use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio::{InterruptHandler, Pio};

use rp_pico2_examples::bluetooth;
use rp_pico2_examples::common::cyw43_task;
use static_cell::StaticCell;
use trouble_host::prelude::*;

use {defmt_rtt as _, panic_probe as _};

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Bluetooth BLE Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"Standalone example demonstrating Bluetooth BLE functionality on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("Bluetooth BLE Example Starting");

    let fw = include_bytes!("../firmware/43439A0.bin");
    let btfw = include_bytes!("../firmware/43439A0_btfw.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio0 = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio0.common,
        pio0.sm0,
        DEFAULT_CLOCK_DIVIDER,
        pio0.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, bt_device, _control, runner) =
        cyw43::new_with_bluetooth(state, pwr, spi, fw, btfw).await;

    spawner.spawn(cyw43_task(runner)).unwrap();

    let controller: ExternalController<_, 10> = ExternalController::new(bt_device);

    bluetooth::run(controller).await;
}
