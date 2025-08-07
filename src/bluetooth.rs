use embassy_futures::join::join;
use embassy_futures::select::select;

use embassy_time::Timer;
use trouble_host::prelude::*;

use crate::{http_server::GPIO_CHANNEL, log_info};

/// Max number of connections
const CONNECTIONS_MAX: usize = 1;

/// Max number of L2CAP channels.
const L2CAP_CHANNELS_MAX: usize = 2; // Signal + att

// GATT Server definition
#[gatt_server]
pub struct Server {
    battery_service: BatteryService,
    uart_service: UartService,
}

/// Battery service
#[gatt_service(uuid = service::BATTERY)]
pub struct BatteryService {
    /// Battery Level
    #[descriptor(uuid = descriptors::VALID_RANGE, read, value = [0, 100])]
    #[descriptor(uuid = descriptors::MEASUREMENT_DESCRIPTION, name = "hello", read, value = "Battery Level")]
    #[characteristic(uuid = characteristic::BATTERY_LEVEL, read, notify, value = 10)]
    level: u8,
    #[characteristic(uuid = "408813df-5dd4-1f87-ec11-cdb001100000", write, read, notify)]
    status: bool,
}

/// Nordic UART Service for serial communication
#[gatt_service(uuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")]
pub struct UartService {
    /// RX Characteristic - for receiving data FROM phone
    #[characteristic(
        uuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
        write,
        write_without_response
    )]
    rx: [u8; 20],
    /// TX Characteristic - for sending data TO phone  
    #[characteristic(uuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E", read, notify)]
    tx: [u8; 20],
}

/// Run the BLE stack.
pub async fn run<C>(controller: C)
where
    C: Controller,
{
    // Generate a truly random address for each boot to avoid phone caching issues
    // The first byte must have bits set to '01' for a static random address (0x40-0x7F or 0xC0-0xFF)
    let mut rng = embassy_rp::clocks::RoscRng;
    let mut addr_bytes = [0u8; 6];
    rng.fill_bytes(&mut addr_bytes);
    addr_bytes[0] = (addr_bytes[0] & 0x3F) | 0x40; // Ensure bits [7:6] = 01 for static random
    let address: Address = Address::random(addr_bytes);
    log_info!("Our address = {:?}", address);

    let mut resources: HostResources<DefaultPacketPool, CONNECTIONS_MAX, L2CAP_CHANNELS_MAX> =
        HostResources::new();
    let stack = trouble_host::new(controller, &mut resources).set_random_address(address);
    let Host {
        mut peripheral,
        runner,
        ..
    } = stack.build();

    log_info!("Starting advertising and GATT service");
    let server = Server::new_with_config(GapConfig::Peripheral(PeripheralConfig {
        name: "TrouBLE",
        appearance: &appearance::power_device::GENERIC_POWER_DEVICE,
    }))
    .unwrap();

    let _ = join(ble_task(runner), async {
        loop {
            match advertise("Pico2W-BLE", &mut peripheral, &server).await {
                Ok(conn) => {
                    // set up tasks when the connection is established to a central, so they don't run when no one is connected.
                    let a = gatt_events_task(&server, &conn);
                    let b = custom_task(&server, &conn, &stack);
                    // run until any task ends (usually because the connection has been closed),
                    // then return to advertising state.
                    select(a, b).await;
                }
                Err(e) => {
                    #[cfg(feature = "defmt")]
                    let e = defmt::Debug2Format(&e);
                    panic!("[adv] error: {:?}", e);
                }
            }
        }
    })
    .await;
}

/// This is a background task that is required to run forever alongside any other BLE tasks.
///
/// ## Alternative
///
/// If you didn't require this to be generic for your application, you could statically spawn this with i.e.
///
/// ```rust,ignore
///
/// #[embassy_executor::task]
/// async fn ble_task(mut runner: Runner<'static, SoftdeviceController<'static>>) {
///     runner.run().await;
/// }
///
/// spawner.must_spawn(ble_task(runner));
/// ```
async fn ble_task<C: Controller, P: PacketPool>(mut runner: Runner<'_, C, P>) {
    loop {
        if let Err(e) = runner.run().await {
            #[cfg(feature = "defmt")]
            let e = defmt::Debug2Format(&e);
            panic!("[ble_task] error: {:?}", e);
        }
    }
}

/// Stream Events until the connection closes.
///
/// This function will handle the GATT events and process them.
/// This is how we interact with read and write requests.
pub async fn gatt_events_task<P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
) -> Result<(), Error> {
    let level = server.battery_service.level;
    let status = server.battery_service.status;
    let uart_rx = server.uart_service.rx;
    let uart_tx = server.uart_service.tx;
    log_info!("[gatt] Starting GATT events task");

    let reason = loop {
        match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                log_info!("[gatt] Disconnected: {:?}", reason);
                break reason;
            }
            GattConnectionEvent::ConnectionParamsUpdated {
                conn_interval,
                peripheral_latency,
                supervision_timeout,
            } => {
                log_info!(
                    "[gatt] Connection params updated: {:?}, {:?}, {:?}",
                    conn_interval,
                    peripheral_latency,
                    supervision_timeout
                );
            }
            GattConnectionEvent::PhyUpdated { tx_phy, rx_phy } => {
                log_info!("[gatt] PHY updated: {:?}, {:?}", tx_phy, rx_phy);
            }
            GattConnectionEvent::Gatt { event } => {
                let result = match &event {
                    GattEvent::Read(event) => {
                        log_info!("[gatt] Read Event on handle: {:?}", event.handle());
                        if event.handle() == level.handle {
                            let value = server.get(&level);
                            log_info!("[gatt] Read Level Characteristic: {:?}", value);
                        } else if event.handle() == status.handle {
                            let value = server.get(&status);
                            log_info!("[gatt] Read Status Characteristic: {:?}", value);
                        } else if event.handle() == uart_tx.handle {
                            let value = server.get(&uart_tx);
                            log_info!("[gatt] Read UART TX Characteristic: {:?}", value);
                        }
                        // Allow reads without encryption for demo purposes
                        None
                    }
                    GattEvent::Write(event) => {
                        log_info!("[gatt] Write Event on handle: {:?}", event.handle());
                        if event.handle() == level.handle {
                            log_info!("[gatt] Write to Level Characteristic: {:?}", event.data());
                        } else if event.handle() == status.handle {
                            log_info!("[gatt] Write to Status Characteristic: {:?}", event.data());
                        } else if event.handle() == uart_rx.handle {
                            let received_data = event.data();
                            if let Ok(message) = core::str::from_utf8(received_data) {
                                let trimmed = message.trim();
                                log_info!("[gatt] UART RX received: '{}'", trimmed);
                                let sender = GPIO_CHANNEL.sender();
                                // Handle LED control commands
                                match trimmed {
                                    "led_on" => {
                                        sender.send(crate::http_server::GpioCommand::SetHigh).await;
                                        log_info!("[gpio] LED turned ON (pin 17 HIGH)");
                                    }
                                    "led_off" => {
                                        sender.send(crate::http_server::GpioCommand::SetLow).await;
                                        log_info!("[gpio] LED turned OFF (pin 17 LOW)");
                                    }
                                    _ => {
                                        log_info!("[gatt] Unknown command: '{}'", trimmed);
                                    }
                                }
                            } else {
                                log_info!("[gatt] UART RX received invalid UTF-8 data");
                            }
                        }
                        // Allow writes without encryption for demo purposes
                        None
                    }
                    _ => {
                        log_info!("[gatt] Other GATT event (notifications config, etc.)");
                        None
                    }
                };

                // Send appropriate reply based on security check
                let reply_result = if let Some(code) = result {
                    event.reject(code)
                } else {
                    event.accept()
                };

                match reply_result {
                    Ok(reply) => {
                        log_info!("[gatt] Sending reply");
                        reply.send().await;
                    }
                    Err(e) => {
                        log_info!("[gatt] error sending response: {:?}", e);
                    }
                };
            }
        }
    };
    log_info!("[gatt] disconnected: {:?}", reason);
    Ok(())
}

/// Create an advertiser to use to connect to a BLE Central, and wait for it to connect.
async fn advertise<'values, 'server, C: Controller>(
    name: &'values str,
    peripheral: &mut Peripheral<'values, C, DefaultPacketPool>,
    server: &'server Server<'values>,
) -> Result<GattConnection<'values, 'server, DefaultPacketPool>, BleHostError<C::Error>> {
    let mut advertiser_data = [0; 31];
    let len = AdStructure::encode_slice(
        &[
            AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
            AdStructure::ServiceUuids16(&[[0x18, 0x0f]]), // Battery Service
            AdStructure::CompleteLocalName(name.as_bytes()),
        ],
        &mut advertiser_data[..],
    )?;

    // Phone-friendly advertising parameters
    let adv_params = AdvertisementParameters {
        interval_min: embassy_time::Duration::from_millis(100), // 100ms
        interval_max: embassy_time::Duration::from_millis(150), // 150ms
        ..Default::default()
    };

    // Add UART service UUID in scan response data
    let uart_uuid = [
        0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40,
        0x6E,
    ];
    let mut scan_data = [0; 31];
    let scan_len = AdStructure::encode_slice(
        &[AdStructure::ServiceUuids128(&[uart_uuid])],
        &mut scan_data[..],
    )?;

    let advertiser = peripheral
        .advertise(
            &adv_params,
            Advertisement::ConnectableScannableUndirected {
                adv_data: &advertiser_data[..len],
                scan_data: &scan_data[..scan_len],
            },
        )
        .await?;
    log_info!("[adv] advertising");
    let conn = advertiser.accept().await?.with_attribute_server(server)?;
    log_info!("[adv] connection established");
    Ok(conn)
}

/// Example task to use the BLE notifier interface.
/// This task will notify the connected central of a counter value every 2 seconds.
/// It will also read the RSSI value every 2 seconds.
/// and will stop when the connection is closed by the central or an error occurs.
pub async fn custom_task<C: Controller, P: PacketPool>(
    server: &Server<'_>,
    conn: &GattConnection<'_, '_, P>,
    stack: &Stack<'_, C, P>,
) {
    let mut tick: u8 = 0;
    let level = server.battery_service.level;
    let uart_tx = server.uart_service.tx;
    loop {
        tick = tick.wrapping_add(1);
        log_info!("[custom_task] notifying connection of tick {}", tick);
        if level.notify(conn, &tick).await.is_err() {
            log_info!("[custom_task] error notifying connection");
            break;
        };

        // Send a message via UART TX
        let message = b"Hello from Pico2W!\n";
        let mut uart_data = [0u8; 20];
        let len = message.len().min(20);
        uart_data[..len].copy_from_slice(&message[..len]);
        if uart_tx.notify(conn, &uart_data).await.is_err() {
            log_info!("[custom_task] error sending UART message");
        }
        // read RSSI (Received Signal Strength Indicator) of the connection.
        if let Ok(rssi) = conn.raw().rssi(stack).await {
            log_info!("[custom_task] RSSI: {:?}", rssi);
        } else {
            log_info!("[custom_task] error getting RSSI");
            break;
        };
        Timer::after_secs(10).await; // Very slow notifications for better phone compatibility
    }
}
