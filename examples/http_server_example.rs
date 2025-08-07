#![no_std]
#![no_main]

use cyw43::JoinOptions;
use cyw43_pio::{PioSpi, DEFAULT_CLOCK_DIVIDER};
use defmt::*;
use embassy_executor::Spawner;
use embassy_net::tcp::TcpSocket;
use embassy_net::{Config, Stack, StackResources};
use embassy_rp::bind_interrupts;
use embassy_rp::block::ImageDef;
use embassy_rp::clocks::RoscRng;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::{InterruptHandler, Pio};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::Timer;
use embedded_io_async::Write;
use heapless::{String, Vec};
use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

// Replace these with your WiFi credentials
const WIFI_SSID: &str = "your_wifi_ssid";
const WIFI_PASSPHRASE: &str = "your_wifi_password";

#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"HTTP Server Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"Standalone HTTP server example with GPIO control on RP Pico 2 W"
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
});

#[derive(Debug, Clone)]
pub enum GpioCommand {
    SetHigh,
    SetLow,
}

pub static GPIO_CHANNEL: Channel<ThreadModeRawMutex, GpioCommand, 10> = Channel::new();

#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<'static, Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn net_task(mut runner: embassy_net::Runner<'static, cyw43::NetDriver<'static>>) -> ! {
    runner.run().await
}

#[embassy_executor::task]
async fn gpio_control_task(mut gpio_pin: Output<'static>) -> ! {
    info!("GPIO control task started for Pin 17");
    gpio_pin.set_low();

    loop {
        let receiver = GPIO_CHANNEL.receiver();
        let command = receiver.receive().await;

        match command {
            GpioCommand::SetHigh => {
                gpio_pin.set_high();
                info!("GPIO Pin 17 set HIGH");
            }
            GpioCommand::SetLow => {
                gpio_pin.set_low();
                info!("GPIO Pin 17 set LOW");
            }
        }
        Timer::after_millis(10).await;
    }
}

fn parse_http_request(buffer: &str) -> Option<(&str, &str)> {
    let mut lines = buffer.lines();
    let request_line = lines.next()?;
    let parts: Vec<&str, 3> = request_line.split(' ').collect();
    if parts.len() >= 2 {
        Some((parts[0], parts[1]))
    } else {
        None
    }
}

fn create_html_response(led_state: bool) -> String<2048> {
    let mut html: String<2048> = String::new();
    let _ = core::fmt::write(
        &mut html,
        format_args!(
            r#"HTTP/1.1 200 OK
Content-Type: text/html
Connection: close

<!DOCTYPE html>
<html>
<head>
    <title>Pico2W GPIO Control</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 40px; }}
        .button {{ padding: 20px 40px; margin: 10px; font-size: 18px; }}
        .on {{ background-color: #4CAF50; color: white; }}
        .off {{ background-color: #f44336; color: white; }}
        .status {{ font-size: 24px; margin: 20px 0; }}
    </style>
</head>
<body>
    <h1>Pico2W GPIO Control</h1>
    <div class="status">LED Status: {}</div>
    <a href="/led/on"><button class="button on">Turn LED ON</button></a>
    <a href="/led/off"><button class="button off">Turn LED OFF</button></a>
    <br><br>
    <h2>API Endpoints:</h2>
    <ul>
        <li><a href="/status">GET /status</a> - JSON status</li>
        <li>GET /led/on - Turn LED on</li>
        <li>GET /led/off - Turn LED off</li>
    </ul>
</body>
</html>"#,
            if led_state { "ON" } else { "OFF" }
        ),
    );
    html
}

fn create_json_response(led_state: bool) -> String<2048> {
    let mut json: String<2048> = String::new();
    let _ = core::fmt::write(
        &mut json,
        format_args!(
            r#"HTTP/1.1 200 OK
Content-Type: application/json
Connection: close

{{"status":"ok","led_state":{},"gpio_pin":17}}"#,
            if led_state { "true" } else { "false" }
        ),
    );
    json
}

fn create_404_response() -> String<2048> {
    let mut response: String<2048> = String::new();
    let _ = core::fmt::write(
        &mut response,
        format_args!(
            r#"HTTP/1.1 404 Not Found
Content-Type: text/plain
Connection: close
404 Not Found
The requested resource was not found on this server."#
        ),
    );
    response
}

#[embassy_executor::task]
async fn http_server_task(stack: Stack<'static>) -> ! {
    let mut rx_buffer = [0; 1024];
    let mut tx_buffer = [0; 1024];
    let mut led_state = false;

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        info!("HTTP server listening on port 80");

        if let Err(e) = socket.accept(80).await {
            warn!("Accept error: {:?}", e);
            continue;
        }

        info!("New HTTP connection established");

        let mut buffer = [0; 1024];
        match socket.read(&mut buffer).await {
            Ok(n) if n > 0 => {
                let request = core::str::from_utf8(&buffer[..n]).unwrap_or("");
                info!("HTTP request received: {}", request);

                if let Some((method, path)) = parse_http_request(request) {
                    info!("Method: {}, Path: {}", method, path);

                    let response = match path {
                        "/" => create_html_response(led_state),
                        "/led/on" => {
                            let sender = GPIO_CHANNEL.sender();
                            sender.send(GpioCommand::SetHigh).await;
                            led_state = true;
                            info!("LED turned ON via HTTP");
                            create_html_response(led_state)
                        }
                        "/led/off" => {
                            let sender = GPIO_CHANNEL.sender();
                            sender.send(GpioCommand::SetLow).await;
                            led_state = false;
                            info!("LED turned OFF via HTTP");
                            create_html_response(led_state)
                        }
                        "/status" => create_json_response(led_state),
                        _ => String::from(create_404_response()),
                    };

                    if let Err(e) = socket.write_all(response.as_bytes()).await {
                        warn!("Write error: {:?}", e);
                    }
                } else {
                    warn!("Invalid HTTP request format");
                }
            }
            Ok(_) => {
                info!("Empty request received");
            }
            Err(e) => {
                warn!("Read error: {:?}", e);
            }
        }

        socket.close();
        Timer::after_millis(100).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());
    info!("HTTP Server Example Starting");

    let fw = include_bytes!("../firmware/43439A0.bin");
    let clm = include_bytes!("../firmware/43439A0_clm.bin");

    // Initialize WiFi
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
    let (net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    spawner.spawn(cyw43_task(runner)).unwrap();

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    let mut rng = RoscRng;
    let seed = rng.next_u64();
    let config = Config::dhcpv4(Default::default());

    static RESOURCES: StaticCell<StackResources<5>> = StaticCell::new();
    let (stack, runner) = embassy_net::new(
        net_device,
        config,
        RESOURCES.init(StackResources::new()),
        seed,
    );

    spawner.spawn(net_task(runner)).unwrap();

    info!("Connecting to WiFi...");
    loop {
        match control
            .join(WIFI_SSID, JoinOptions::new(WIFI_PASSPHRASE.as_bytes()))
            .await
        {
            Ok(_) => {
                info!("WiFi connected successfully");
                break;
            }
            Err(err) => {
                warn!("WiFi join failed with status={}", err.status);
                Timer::after_secs(1).await;
            }
        }
    }

    // Wait for DHCP to complete
    while !stack.is_config_up() {
        Timer::after_millis(100).await;
    }

    while !stack.is_link_up() {
        Timer::after_millis(500).await;
    }

    stack.wait_config_up().await;

    if let Some(config) = stack.config_v4() {
        info!("Network configured: IP address: {:?}", config.address);
        info!(
            "HTTP server will be available at: http://{}/",
            config.address.address()
        );
    }

    // Start GPIO control task
    let gpio_pin = Output::new(p.PIN_17, Level::Low);
    spawner.spawn(gpio_control_task(gpio_pin)).unwrap();

    spawner.spawn(http_server_task(stack)).unwrap();

    // Keep main task alive
    loop {
        Timer::after_secs(60).await;
        info!("HTTP server is running...");
    }
}
