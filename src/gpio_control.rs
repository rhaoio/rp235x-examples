use crate::http_server::{GpioCommand, GPIO_CHANNEL};
use crate::log_info;

use embassy_rp::gpio::Output;
use embassy_time::{Duration, Timer};

/// GPIO Control Task
/// Listens for commands from the HTTP server and controls GPIO pin 16
#[embassy_executor::task]
pub async fn gpio_control_task(mut gpio_pin: Output<'static>) -> ! {
    log_info!("🔌 GPIO Control Task started for Pin 16");

    // Set initial state to LOW
    gpio_pin.set_low();
    log_info!("🔵 GPIO Pin 16 initialized to LOW (0V)");

    loop {
        // Wait for commands from HTTP server
        let receiver = GPIO_CHANNEL.receiver();
        let command = receiver.receive().await;

        match command {
            GpioCommand::SetHigh => {
                gpio_pin.set_high();
                log_info!("🔴 GPIO Pin 16 set to HIGH (3.3V)");
            }
            GpioCommand::SetLow => {
                gpio_pin.set_low();
                log_info!("🔵 GPIO Pin 16 set to LOW (0V)");
            }
        }

        // Small delay to prevent rapid switching
        Timer::after(Duration::from_millis(10)).await;
    }
}
