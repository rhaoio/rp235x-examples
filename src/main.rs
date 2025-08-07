#![no_std]
#![no_main]

use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;

use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;

use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::pio::{InterruptHandler, Pio};

use rp_pico2_examples::common::cyw43_task;

use rp_pico2_examples::wifi_control::wifi_led_task;
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

mod adc_reader;
mod bluetooth;
mod buzzer;
mod common;
mod constants;
mod debouncer;
mod gpio_control;
mod http_server;
mod i2c_lcd;
mod joystick;
mod led_patterns;
mod neo_pixel;
mod pwm_controller;

mod dc_motor;
mod ir_receiver;
mod servo;
mod shift_register;
mod uart;
mod ultrasonic;
mod usb_serial;
mod wifi_control;

use embassy_time::Duration;
use neo_pixel::NeoPixelStrip;

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"RP2350x"),
    embassy_rp::binary_info::rp_program_description!(
        c"This example tests the RP Pico 2 W's Various Features"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    let fw = include_bytes!("../firmware/43439A0.bin");
    let clm = include_bytes!("../firmware/43439A0_clm.bin");
    let _btfw = include_bytes!("../firmware/43439A0_btfw.bin");

    info!("Initializing Pico 2 W with NeoPixel support");

    // Initialize WiFi/CYW43 using PIO0
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
    // Start USB Serial task (much more convenient than GPIO UART for development)
    unwrap!(spawner.spawn(usb_serial::usb_serial_task(p.USB)));

    // Initialize CYW43439 with both WiFi and Bluetooth support
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    spawner.spawn(cyw43_task(runner)).unwrap();

    spawner.spawn(wifi_led_task(control)).unwrap();
    // Keep main task alive
    loop {
        embassy_time::Timer::after(Duration::from_millis(1000)).await;
    }
}
