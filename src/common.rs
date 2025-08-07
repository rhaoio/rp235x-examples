use cyw43_pio::PioSpi;
use embassy_rp::{
    gpio::Output,
    peripherals::{DMA_CH0, PIO0},
};

use crate::neo_pixel::NeoPixelStrip;

#[embassy_executor::task]
pub async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
pub async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}
