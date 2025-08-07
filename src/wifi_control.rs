use core::str::from_utf8;

use defmt::*;

use crate::log_info;
use embassy_net::{
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
    udp::UdpSocket,
    Stack,
};
use embassy_time::{Duration, Instant, Timer};

use reqwless::client::{HttpClient, TlsConfig, TlsVerify};
use reqwless::request::Method;
use serde::Deserialize;

pub const WIFI_SSID: &str = "WIFI_SSID";
pub const WIFI_PASSPHRASE: &str = "PW";

pub const AP_SSID: &str = "RaspberryPi1337";
pub const AP_PASSPHRASE: &str = "xdding12345";

// DHCP Server Configuration
const DHCP_SERVER_IP: [u8; 4] = [192, 168, 4, 1]; // AP IP
const DHCP_CLIENT_IP_START: [u8; 4] = [192, 168, 4, 100]; // First client IP
const DHCP_CLIENT_IP_END: u8 = 200; // Last assignable IP: .200
const DHCP_SUBNET_MASK: [u8; 4] = [255, 255, 255, 0];
const DHCP_LEASE_TIME: u32 = 86400; // 24 hours in seconds

// DHCP Lease Management
const MAX_DHCP_LEASES: usize = 10; // Support up to 10 concurrent clients
const LEASE_RENEWAL_TIME: u32 = DHCP_LEASE_TIME / 2; // Renew at 50% of lease time

#[derive(Clone, Copy, Debug)]
struct DhcpLease {
    mac_address: [u8; 6],
    ip_address: [u8; 4],
    lease_start: Instant,
    lease_duration: Duration,
    is_active: bool,
}

impl DhcpLease {
    const fn new() -> Self {
        Self {
            mac_address: [0; 6],
            ip_address: [0; 4],
            lease_start: Instant::from_ticks(0),
            lease_duration: Duration::from_secs(0),
            is_active: false,
        }
    }

    fn is_expired(&self, now: Instant) -> bool {
        if !self.is_active {
            return true;
        }
        now.duration_since(self.lease_start) > self.lease_duration
    }

    fn matches_mac(&self, mac: &[u8]) -> bool {
        self.is_active && self.mac_address[..] == mac[..6]
    }

    fn matches_ip(&self, ip: &[u8; 4]) -> bool {
        self.is_active && self.ip_address == *ip
    }
}

// Global lease table (in a real system, this would be in a struct)
static mut LEASE_TABLE: [DhcpLease; MAX_DHCP_LEASES] = [DhcpLease::new(); MAX_DHCP_LEASES];

/// DHCP Lease Management Functions
struct DhcpLeaseManager;

impl DhcpLeaseManager {
    /// Find existing lease for a MAC address
    fn find_lease_by_mac(mac: &[u8]) -> Option<[u8; 4]> {
        unsafe {
            let now = Instant::now();
            for lease in &LEASE_TABLE {
                if lease.matches_mac(mac) && !lease.is_expired(now) {
                    return Some(lease.ip_address);
                }
            }
        }
        None
    }

    /// Check if an IP is already leased to someone else
    fn is_ip_taken(ip: &[u8; 4], requesting_mac: &[u8]) -> bool {
        unsafe {
            let now = Instant::now();
            for lease in &LEASE_TABLE {
                if lease.matches_ip(ip) && !lease.is_expired(now) {
                    // IP is taken if it's assigned to a different MAC
                    return !lease.matches_mac(requesting_mac);
                }
            }
        }
        false
    }

    /// Assign a new lease or renew existing one
    fn assign_lease(mac: &[u8], ip: &[u8; 4]) -> bool {
        unsafe {
            let now = Instant::now();

            // First, try to find existing lease for this MAC
            for lease in &mut LEASE_TABLE {
                if lease.matches_mac(mac) {
                    // Renew existing lease
                    lease.ip_address = *ip;
                    lease.lease_start = now;
                    lease.lease_duration = Duration::from_secs(DHCP_LEASE_TIME as u64);
                    log_info!("Renewed lease for MAC {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X} → {}.{}.{}.{}", 
                        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                        ip[0], ip[1], ip[2], ip[3]);
                    return true;
                }
            }

            // Find an empty slot or expired lease
            for lease in &mut LEASE_TABLE {
                if !lease.is_active || lease.is_expired(now) {
                    lease.mac_address[..6].copy_from_slice(&mac[..6]);
                    lease.ip_address = *ip;
                    lease.lease_start = now;
                    lease.lease_duration = Duration::from_secs(DHCP_LEASE_TIME as u64);
                    lease.is_active = true;
                    log_info!("Assigned new lease for MAC {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X} → {}.{}.{}.{}", 
                        mac[0], mac[1], mac[2], mac[3], mac[4], mac[5],
                        ip[0], ip[1], ip[2], ip[3]);
                    return true;
                }
            }
        }
        log_info!("DHCP lease table full! Cannot assign more IPs.");
        false
    }

    /// Find next available IP address
    fn find_available_ip(requesting_mac: &[u8]) -> Option<[u8; 4]> {
        // Check if this MAC already has a lease
        if let Some(existing_ip) = Self::find_lease_by_mac(requesting_mac) {
            return Some(existing_ip);
        }

        // Find next free IP in our range
        for ip_last_octet in DHCP_CLIENT_IP_START[3]..=DHCP_CLIENT_IP_END {
            let candidate_ip = [
                DHCP_SERVER_IP[0],
                DHCP_SERVER_IP[1],
                DHCP_SERVER_IP[2],
                ip_last_octet,
            ];

            if !Self::is_ip_taken(&candidate_ip, requesting_mac) {
                return Some(candidate_ip);
            }
        }

        None // No IPs available
    }

    /// Clean up expired leases (call periodically)
    fn cleanup_expired_leases() {
        unsafe {
            let now = Instant::now();
            let mut cleaned = 0;

            for lease in &mut LEASE_TABLE {
                if lease.is_active && lease.is_expired(now) {
                    log_info!(
                        "Cleaning up expired lease: {}.{}.{}.{}",
                        lease.ip_address[0],
                        lease.ip_address[1],
                        lease.ip_address[2],
                        lease.ip_address[3]
                    );
                    lease.is_active = false;
                    cleaned += 1;
                }
            }

            if cleaned > 0 {
                log_info!("Cleaned up {} expired leases", cleaned);
            }
        }
    }
}

// DHCP Ports
const DHCP_SERVER_PORT: u16 = 67;
const DHCP_CLIENT_PORT: u16 = 68;

// DHCP Packet Structure Constants
const DHCP_MIN_PACKET_SIZE: usize = 240;
const DHCP_HEADER_SIZE: usize = 236;
const DHCP_OPTIONS_START: usize = 240;
const DHCP_CHADDR_SIZE: usize = 16; // Client hardware address field size
const DHCP_SNAME_SIZE: usize = 64; // Server name field size
const DHCP_FILE_SIZE: usize = 128; // File name field size
const DHCP_MAGIC_COOKIE: [u8; 4] = [99, 130, 83, 99];

// DHCP Message Types
const DHCP_DISCOVER: u8 = 1;
const DHCP_OFFER: u8 = 2;
const DHCP_REQUEST: u8 = 3;
const DHCP_ACK: u8 = 5;

// DHCP Options
const DHCP_OPTION_MESSAGE_TYPE: u8 = 53;
const DHCP_OPTION_SERVER_ID: u8 = 54;
const DHCP_OPTION_SUBNET_MASK: u8 = 1;
const DHCP_OPTION_ROUTER: u8 = 3;
const DHCP_OPTION_DNS: u8 = 6;
const DHCP_OPTION_LEASE_TIME: u8 = 51;
const DHCP_OPTION_END: u8 = 255;
const DHCP_OPTION_PAD: u8 = 0;

// DHCP Packet Field Offsets
const DHCP_OP_OFFSET: usize = 0;
const DHCP_HTYPE_OFFSET: usize = 1;
const DHCP_HLEN_OFFSET: usize = 2;
const DHCP_XID_OFFSET: usize = 4; // Transaction ID
const DHCP_CHADDR_OFFSET: usize = 28; // Client MAC address

// DHCP Operations
const DHCP_BOOTREQUEST: u8 = 1;
const DHCP_BOOTREPLY: u8 = 2;

// Hardware Types
const DHCP_HTYPE_ETHERNET: u8 = 1;
const DHCP_HLEN_ETHERNET: u8 = 6;

// Buffer Sizes
const DHCP_SOCKET_RX_BUFFER_SIZE: usize = 1024;
const DHCP_SOCKET_TX_BUFFER_SIZE: usize = 1024;
const DHCP_RESPONSE_BUFFER_SIZE: usize = 512;
const DHCP_PACKET_BUFFER_SIZE: usize = 512;
const DHCP_META_BUFFER_SIZE: usize = 16;

/// WiFi LED control task
/// Blinks the onboard WiFi LED independently of other patterns
#[embassy_executor::task]
pub async fn wifi_led_task(mut control: cyw43::Control<'static>) -> ! {
    let delay = Duration::from_millis(1000);
    loop {
        info!("WiFi led on!");
        control.gpio_set(0, true).await;
        Timer::after(delay).await;

        info!("WiFi led off!");
        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}

pub async fn handle_wifi_stack(stack: &Stack<'static>, seed: u64) -> ! {
    // And now we can use it!

    loop {
        let mut rx_buffer = [0; 8192];
        let mut tls_read_buffer = [0; 16640];
        let mut tls_write_buffer = [0; 16640];

        let client_state = TcpClientState::<1, 1024, 1024>::new();
        let tcp_client = TcpClient::new(*stack, &client_state);
        let dns_client = DnsSocket::new(*stack);
        let tls_config = TlsConfig::new(
            seed,
            &mut tls_read_buffer,
            &mut tls_write_buffer,
            TlsVerify::None,
        );

        let mut http_client = HttpClient::new_with_tls(&tcp_client, &dns_client, tls_config);
        let url = "https://worldtimeapi.org/api/timezone/Europe/Berlin";
        // for non-TLS requests, use this instead:
        // let mut http_client = HttpClient::new(&tcp_client, &dns_client);
        // let url = "http://worldtimeapi.org/api/timezone/Europe/Berlin";

        info!("connecting to {}", &url);

        let mut request = match http_client.request(Method::GET, &url).await {
            Ok(req) => req,
            Err(e) => {
                log_info!("Failed to make HTTP request: {:?}", e);
                continue;
            }
        };

        let response = match request.send(&mut rx_buffer).await {
            Ok(resp) => resp,
            Err(_e) => {
                log_info!("Failed to send HTTP request");
                continue;
            }
        };

        let body = match from_utf8(response.body().read_to_end().await.unwrap()) {
            Ok(b) => b,
            Err(_e) => {
                log_info!("Failed to read response body");
                continue;
            }
        };
        log_info!("Response body: {:?}", &body);

        // parse the response body and update the RTC

        #[derive(Deserialize)]
        struct ApiResponse<'a> {
            datetime: &'a str,
            // other fields as needed
        }

        let bytes = body.as_bytes();
        match serde_json_core::de::from_slice::<ApiResponse>(bytes) {
            Ok((output, _used)) => {
                log_info!("Datetime: {:?}", output.datetime);
            }
            Err(_e) => {
                log_info!("Failed to parse response body");
                continue;
            }
        }

        Timer::after(Duration::from_secs(5)).await;
    }
}

pub async fn handle_ap_mode(
    mut control: cyw43::Control<'static>,
    stack: &'static Stack<'static>,
) -> ! {
    log_info!("AP mode started");

    log_info!("waiting for stack to be up...");
    stack.wait_config_up().await;
    log_info!("Stack is up! AP is ready for connections.");

    // Just blink the LED to indicate AP is running
    loop {
        control.gpio_set(0, true).await; // LED on
        Timer::after(Duration::from_millis(500)).await;
        control.gpio_set(0, false).await; // LED off
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// Simple DHCP server for AP mode
#[embassy_executor::task]
pub async fn dhcp_server_task(stack: &'static Stack<'static>) -> ! {
    log_info!("Starting DHCP server...");

    let mut rx_buffer = [0; DHCP_SOCKET_RX_BUFFER_SIZE];
    let mut tx_buffer = [0; DHCP_SOCKET_TX_BUFFER_SIZE];
    let mut rx_meta = [embassy_net::udp::PacketMetadata::EMPTY; DHCP_META_BUFFER_SIZE];
    let mut tx_meta = [embassy_net::udp::PacketMetadata::EMPTY; DHCP_META_BUFFER_SIZE];

    let mut socket = UdpSocket::new(
        *stack,
        &mut rx_meta,
        &mut rx_buffer,
        &mut tx_meta,
        &mut tx_buffer,
    );

    if let Err(e) = socket.bind(DHCP_SERVER_PORT) {
        log_info!("Failed to bind DHCP server socket: {:?}", e);
        loop {
            Timer::after(Duration::from_secs(1)).await;
        }
    }

    log_info!("DHCP server listening on port {}", DHCP_SERVER_PORT);

    let mut cleanup_counter = 0u32;

    loop {
        let mut buf = [0; DHCP_PACKET_BUFFER_SIZE];

        match socket.recv_from(&mut buf).await {
            Ok((len, remote_addr)) => {
                log_info!(
                    "Received DHCP packet from {:?}, length: {}",
                    remote_addr,
                    len
                );

                // Periodically clean up expired leases (every 100 packets or so)
                cleanup_counter += 1;
                if cleanup_counter % 100 == 0 {
                    DhcpLeaseManager::cleanup_expired_leases();
                }

                if len >= DHCP_MIN_PACKET_SIZE {
                    let response = handle_dhcp_packet_secure(&buf, len);

                    if let Some(response_data) = response {
                        let dest = embassy_net::IpEndpoint::new(
                            embassy_net::IpAddress::v4(255, 255, 255, 255), // Broadcast
                            DHCP_CLIENT_PORT,
                        );

                        if let Err(e) = socket.send_to(&response_data, dest).await {
                            log_info!("Failed to send DHCP response: {:?}", e);
                        } else {
                            log_info!("Sent DHCP response");
                        }
                    }
                }
            }
            Err(e) => {
                log_info!("DHCP socket error: {:?}", e);
                Timer::after(Duration::from_millis(100)).await;
            }
        }
    }
}

/// Claude Generated DHCP !
fn handle_dhcp_packet_secure(
    packet: &[u8; DHCP_PACKET_BUFFER_SIZE],
    packet_len: usize,
) -> Option<heapless::Vec<u8, DHCP_RESPONSE_BUFFER_SIZE>> {
    if packet_len < DHCP_MIN_PACKET_SIZE {
        return None;
    }

    // Parse basic DHCP header using constants
    let op = packet[DHCP_OP_OFFSET];
    let htype = packet[DHCP_HTYPE_OFFSET];
    let hlen = packet[DHCP_HLEN_OFFSET];
    let transaction_id = &packet[DHCP_XID_OFFSET..DHCP_XID_OFFSET + 4];
    let client_mac = &packet[DHCP_CHADDR_OFFSET..DHCP_CHADDR_OFFSET + 6];

    // Validate DHCP request format
    if op != DHCP_BOOTREQUEST || htype != DHCP_HTYPE_ETHERNET || hlen != DHCP_HLEN_ETHERNET {
        log_info!("❌ Invalid DHCP request format");
        return None; // Not a valid DHCP request
    }

    // Find DHCP message type in options section
    let mut message_type = 0;
    let mut i = DHCP_OPTIONS_START;

    while i < packet_len {
        if packet[i] == DHCP_OPTION_END {
            break;
        }
        if packet[i] == DHCP_OPTION_MESSAGE_TYPE && i + 2 < packet_len {
            message_type = packet[i + 2];
            break;
        }
        if packet[i] == DHCP_OPTION_PAD {
            i += 1;
        } else if i + 1 < packet_len {
            i += 2 + packet[i + 1] as usize; // Skip option
        } else {
            break;
        }
    }

    log_info!(
        "🔒 DHCP message type: {} from MAC {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
        message_type,
        client_mac[0],
        client_mac[1],
        client_mac[2],
        client_mac[3],
        client_mac[4],
        client_mac[5]
    );

    match message_type {
        DHCP_DISCOVER => {
            // 🔍 Find or assign an available IP using secure lease management
            if let Some(client_ip) = DhcpLeaseManager::find_available_ip(client_mac) {
                log_info!(
                    "💡 Offering IP: {}.{}.{}.{} to MAC {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                    client_ip[0],
                    client_ip[1],
                    client_ip[2],
                    client_ip[3],
                    client_mac[0],
                    client_mac[1],
                    client_mac[2],
                    client_mac[3],
                    client_mac[4],
                    client_mac[5]
                );

                create_dhcp_response(transaction_id, client_mac, &client_ip, DHCP_OFFER)
            } else {
                log_info!("❌ No available IP addresses for MAC {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", 
                    client_mac[0], client_mac[1], client_mac[2], client_mac[3], client_mac[4], client_mac[5]);
                None
            }
        }
        DHCP_REQUEST => {
            // 🎯 Assign lease only if available or already belongs to this MAC
            if let Some(client_ip) = DhcpLeaseManager::find_available_ip(client_mac) {
                if DhcpLeaseManager::assign_lease(client_mac, &client_ip) {
                    log_info!("✅ Confirmed IP: {}.{}.{}.{} for MAC {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", 
                        client_ip[0], client_ip[1], client_ip[2], client_ip[3],
                        client_mac[0], client_mac[1], client_mac[2], client_mac[3], client_mac[4], client_mac[5]);

                    create_dhcp_response(transaction_id, client_mac, &client_ip, DHCP_ACK)
                } else {
                    log_info!("❌ Failed to assign lease for MAC {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", 
                        client_mac[0], client_mac[1], client_mac[2], client_mac[3], client_mac[4], client_mac[5]);
                    None
                }
            } else {
                log_info!("❌ No IP available for REQUEST from MAC {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}", 
                    client_mac[0], client_mac[1], client_mac[2], client_mac[3], client_mac[4], client_mac[5]);
                None
            }
        }
        _ => {
            log_info!("❓ Unsupported DHCP message type: {}", message_type);
            None
        }
    }
}

fn create_dhcp_response(
    transaction_id: &[u8],
    client_mac: &[u8],
    client_ip: &[u8; 4],
    message_type: u8,
) -> Option<heapless::Vec<u8, DHCP_RESPONSE_BUFFER_SIZE>> {
    let mut response = heapless::Vec::<u8, DHCP_RESPONSE_BUFFER_SIZE>::new();

    // DHCP Header - Build the response packet according to RFC 2131
    response.push(DHCP_BOOTREPLY).ok()?; // op: BOOTREPLY
    response.push(DHCP_HTYPE_ETHERNET).ok()?; // htype: Ethernet
    response.push(DHCP_HLEN_ETHERNET).ok()?; // hlen: MAC address length
    response.push(0).ok()?; // hops

    // Transaction ID (must match the request)
    response.extend_from_slice(transaction_id).ok()?;

    // secs (0), flags (0)
    response.extend_from_slice(&[0, 0, 0, 0]).ok()?;

    // ciaddr (client IP - 0 for OFFER/ACK when client doesn't have IP yet)
    response.extend_from_slice(&[0, 0, 0, 0]).ok()?;

    // yiaddr (your IP - the IP we're offering/confirming)
    response.extend_from_slice(client_ip).ok()?;

    // siaddr (server IP - next server to use in bootstrap)
    response.extend_from_slice(&DHCP_SERVER_IP).ok()?;

    // giaddr (gateway IP - 0 since client and server are on same subnet)
    response.extend_from_slice(&[0, 0, 0, 0]).ok()?;

    // chaddr (client hardware address - MAC)
    response.extend_from_slice(client_mac).ok()?;
    // Pad chaddr field to 16 bytes
    for _ in 0..(DHCP_CHADDR_SIZE - 6) {
        response.push(0).ok()?;
    }

    // sname (server name) and file (boot file name) - all zeros
    for _ in 0..(DHCP_SNAME_SIZE + DHCP_FILE_SIZE) {
        response.push(0).ok()?;
    }

    // Magic cookie - identifies this as a DHCP packet
    response.extend_from_slice(&DHCP_MAGIC_COOKIE).ok()?;

    // === DHCP Options Section ===

    // Option 53: DHCP Message Type
    response
        .extend_from_slice(&[DHCP_OPTION_MESSAGE_TYPE, 1, message_type])
        .ok()?;

    // Option 54: Server Identifier (our IP)
    response
        .extend_from_slice(&[DHCP_OPTION_SERVER_ID, 4])
        .ok()?;
    response.extend_from_slice(&DHCP_SERVER_IP).ok()?;

    // Option 1: Subnet Mask
    response
        .extend_from_slice(&[DHCP_OPTION_SUBNET_MASK, 4])
        .ok()?;
    response.extend_from_slice(&DHCP_SUBNET_MASK).ok()?;

    // Option 3: Router (default gateway)
    response.extend_from_slice(&[DHCP_OPTION_ROUTER, 4]).ok()?;
    response.extend_from_slice(&DHCP_SERVER_IP).ok()?;

    // Option 6: DNS Servers (we act as DNS server too)
    response.extend_from_slice(&[DHCP_OPTION_DNS, 4]).ok()?;
    response.extend_from_slice(&DHCP_SERVER_IP).ok()?;

    // Option 51: IP Address Lease Time
    response
        .extend_from_slice(&[DHCP_OPTION_LEASE_TIME, 4])
        .ok()?;
    response
        .extend_from_slice(&DHCP_LEASE_TIME.to_be_bytes())
        .ok()?;

    // Option 255: End of options
    response.push(DHCP_OPTION_END).ok()?;

    Some(response)
}
