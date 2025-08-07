use embassy_rp::{
    peripherals::{PIN_4, PIN_5, UART1},
    uart::{self, Uart},
    Peri,
};

#[embassy_executor::task]
pub async fn uart_task(
    uart1: Peri<'static, UART1>,
    pin_4: Peri<'static, PIN_4>,
    pin_5: Peri<'static, PIN_5>,
) -> ! {
    let config = uart::Config::default();
    let mut uart = Uart::new_blocking(uart1, pin_4, pin_5, config);

    uart.blocking_write("Hello World!\r\n".as_bytes()).unwrap();

    loop {
        uart.blocking_write("hello there!\r\n".as_bytes()).unwrap();
        cortex_m::asm::delay(1_000_000);
    }
}
