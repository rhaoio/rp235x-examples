use crate::log_info;

use embassy_net::{tcp::TcpSocket, Stack};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Timer};
use embedded_io_async::Write;
use heapless::{String, Vec};

// HTTP Server Configuration
const HTTP_SERVER_PORT: u16 = 80;
const HTTP_BUFFER_SIZE: usize = 1024;
const HTTP_RESPONSE_BUFFER_SIZE: usize = 4096;
const MAX_CONCURRENT_CONNECTIONS: usize = 3;

// GPIO Control Channel
pub static GPIO_CHANNEL: Channel<ThreadModeRawMutex, GpioCommand, 10> = Channel::new();

// HTTP Status Codes
const HTTP_OK: &str = "200 OK";
const HTTP_NOT_FOUND: &str = "404 Not Found";
const HTTP_BAD_REQUEST: &str = "400 Bad Request";
const HTTP_INTERNAL_ERROR: &str = "500 Internal Server Error";

// HTTP Response Headers
const HTTP_HEADER_CONTENT_TYPE_HTML: &str = "Content-Type: text/html\r\n";
const HTTP_HEADER_CONTENT_TYPE_JSON: &str = "Content-Type: application/json\r\n";
const HTTP_HEADER_CONNECTION_CLOSE: &str = "Connection: close\r\n";

#[derive(Debug, Clone)]
pub enum HttpMethod {
    GET,
    POST,
    PUT,
    DELETE,
    Unknown,
}

#[derive(Debug)]
pub struct HttpRequest<'a> {
    pub method: HttpMethod,
    pub path: &'a str,
    pub headers: Vec<(&'a str, &'a str), 10>,
    pub body: &'a str,
}

#[derive(Debug)]
pub struct HttpResponse {
    pub status: String<32>,
    pub headers: String<256>,
    pub body: String<2048>,
}

impl HttpResponse {
    pub fn new() -> Self {
        Self {
            status: String::new(),
            headers: String::new(),
            body: String::new(),
        }
    }

    pub fn ok() -> Self {
        let mut response = Self::new();
        response.status.push_str(HTTP_OK).ok();
        response
            .headers
            .push_str(HTTP_HEADER_CONTENT_TYPE_HTML)
            .ok();
        response.headers.push_str(HTTP_HEADER_CONNECTION_CLOSE).ok();
        response
    }

    pub fn not_found() -> Self {
        let mut response = Self::new();
        response.status.push_str(HTTP_NOT_FOUND).ok();
        response
            .headers
            .push_str(HTTP_HEADER_CONTENT_TYPE_HTML)
            .ok();
        response.headers.push_str(HTTP_HEADER_CONNECTION_CLOSE).ok();
        response
            .body
            .push_str("<html><body><h1>404 Not Found</h1></body></html>")
            .ok();
        response
    }

    pub fn json(json_content: &str) -> Self {
        let mut response = Self::new();
        response.status.push_str(HTTP_OK).ok();
        response
            .headers
            .push_str(HTTP_HEADER_CONTENT_TYPE_JSON)
            .ok();
        response.headers.push_str(HTTP_HEADER_CONNECTION_CLOSE).ok();
        response.body.push_str(json_content).ok();
        response
    }

    pub fn error(error_msg: &str) -> Self {
        let mut response = Self::new();
        response.status.push_str(HTTP_INTERNAL_ERROR).ok();
        response
            .headers
            .push_str(HTTP_HEADER_CONTENT_TYPE_HTML)
            .ok();
        response.headers.push_str(HTTP_HEADER_CONNECTION_CLOSE).ok();
        response
            .body
            .push_str("<html><body><h1>500 Internal Server Error</h1><p>")
            .ok();
        response.body.push_str(error_msg).ok();
        response.body.push_str("</p></body></html>").ok();
        response
    }

    pub fn to_bytes(&self) -> Vec<u8, HTTP_RESPONSE_BUFFER_SIZE> {
        let mut response_bytes = Vec::new();

        // Status line
        response_bytes.extend_from_slice(b"HTTP/1.1 ").ok();
        response_bytes
            .extend_from_slice(self.status.as_bytes())
            .ok();
        response_bytes.extend_from_slice(b"\r\n").ok();

        // Headers
        response_bytes
            .extend_from_slice(self.headers.as_bytes())
            .ok();

        // Content-Length header
        let content_length = self.body.len();
        response_bytes.extend_from_slice(b"Content-Length: ").ok();
        let mut len_str = String::<16>::new();
        core::fmt::write(&mut len_str, format_args!("{}", content_length)).ok();
        response_bytes.extend_from_slice(len_str.as_bytes()).ok();
        response_bytes.extend_from_slice(b"\r\n\r\n").ok();

        log_info!(
            "Response so far: {} bytes (before body)",
            response_bytes.len()
        );
        log_info!("Body to add: {} bytes", self.body.len());

        // Body
        if let Err(_) = response_bytes.extend_from_slice(self.body.as_bytes()) {
            log_info!(
                "❌ Failed to add body - buffer full! Current: {}, Capacity: {}",
                response_bytes.len(),
                response_bytes.capacity()
            );
        } else {
            log_info!("✅ Body added successfully");
        }

        log_info!("Final response: {} bytes", response_bytes.len());
        response_bytes
    }
}

/// Parse HTTP request from buffer
fn parse_http_request(buffer: &str) -> Result<HttpRequest, &'static str> {
    let lines: Vec<&str, 20> = buffer.lines().collect();

    if lines.is_empty() {
        return Err("Empty request");
    }

    // Parse request line (e.g., "GET /path HTTP/1.1")
    let request_parts: Vec<&str, 3> = lines[0].split_whitespace().collect();
    if request_parts.len() < 2 {
        return Err("Invalid request line");
    }

    let method = match request_parts[0] {
        "GET" => HttpMethod::GET,
        "POST" => HttpMethod::POST,
        "PUT" => HttpMethod::PUT,
        "DELETE" => HttpMethod::DELETE,
        _ => HttpMethod::Unknown,
    };

    let path = request_parts[1];

    // Parse headers
    let mut headers = Vec::new();
    let mut header_end = 1;

    for (i, line) in lines.iter().enumerate().skip(1) {
        if line.is_empty() {
            header_end = i;
            break;
        }

        if let Some(colon_pos) = line.find(':') {
            let key = line[..colon_pos].trim();
            let value = line[colon_pos + 1..].trim();
            headers.push((key, value)).ok();
        }
    }

    // Get body (everything after empty line) by finding it in the original buffer
    let body = if let Some(double_newline) = buffer.find("\r\n\r\n") {
        &buffer[double_newline + 4..] // Skip past "\r\n\r\n"
    } else if let Some(double_newline) = buffer.find("\n\n") {
        &buffer[double_newline + 2..] // Skip past "\n\n"
    } else {
        "" // No body found
    };

    Ok(HttpRequest {
        method,
        path,
        headers,
        body,
    })
}

/// Route handler function type
pub type RouteHandler = fn(&HttpRequest) -> HttpResponse;

/// Simple router for HTTP requests
pub struct HttpRouter {
    routes: Vec<(&'static str, HttpMethod, RouteHandler), 20>,
}

impl HttpRouter {
    pub fn new() -> Self {
        Self { routes: Vec::new() }
    }

    pub fn get(&mut self, path: &'static str, handler: RouteHandler) -> &mut Self {
        match self.routes.push((path, HttpMethod::GET, handler)) {
            Ok(()) => {
                log_info!("✅ Added GET route: {}", path);
            }
            Err(_) => {
                log_info!(
                    "❌ Failed to add GET route: {} (routes full: {})",
                    path,
                    self.routes.len()
                );
            }
        }
        self
    }

    pub fn post(&mut self, path: &'static str, handler: RouteHandler) -> &mut Self {
        match self.routes.push((path, HttpMethod::POST, handler)) {
            Ok(()) => {
                log_info!("✅ Added POST route: {}", path);
            }
            Err(_) => {
                log_info!(
                    "❌ Failed to add POST route: {} (routes full: {})",
                    path,
                    self.routes.len()
                );
            }
        }
        self
    }

    pub fn handle_request(&self, request: &HttpRequest) -> HttpResponse {
        log_info!("Looking for: {:?} {}", request.method, request.path);

        // Simple direct route checking to avoid iteration issues
        match (request.method.clone(), request.path) {
            (HttpMethod::GET, "/") => {
                log_info!("Matched root");
                handle_root(request)
            }
            (HttpMethod::GET, "/status") => {
                log_info!("Matched status");
                handle_status(request)
            }
            (HttpMethod::GET, "/info") => {
                log_info!("Matched info");
                handle_info(request)
            }
            (HttpMethod::GET, "/bluetooth") => {
                log_info!("Matched Bluetooth status");
                handle_bluetooth_status(request)
            }
            (HttpMethod::GET, "/api/health") => {
                log_info!("Matched health");
                handle_health(request)
            }
            (HttpMethod::GET, "/gpio") => {
                log_info!("Matched GPIO page");
                let response = handle_gpio_control(request);
                log_info!(
                    "GPIO response - Status: {}, Headers: {} chars, Body: {} chars",
                    response.status.len(),
                    response.headers.len(),
                    response.body.len()
                );
                response
            }
            (HttpMethod::POST, "/api/gpio/high") => {
                log_info!("Matched GPIO high");
                handle_gpio_high(request)
            }
            (HttpMethod::POST, "/api/gpio/low") => {
                log_info!("Matched GPIO low");
                handle_gpio_low(request)
            }
            _ => {
                log_info!("No route found");
                HttpResponse::not_found()
            }
        }
    }
}

/// Default route handlers
pub fn handle_root(_request: &HttpRequest) -> HttpResponse {
    let mut response = HttpResponse::ok();
    response.body.push_str(r#"
<!DOCTYPE html>
<html>
<head>
    <title>Pico 2W HTTP Server</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; background: #f0f0f0; }
        .container { background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }
        h1 { color: #333; }
        .status { background: #e8f5e8; padding: 10px; border-radius: 5px; margin: 10px 0; }
        .button { background: #4CAF50; color: white; padding: 10px 20px; text-decoration: none; border-radius: 5px; margin: 5px; display: inline-block; }
        .button:hover { background: #45a049; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚀 Pico 2W HTTP Server</h1>
        <div class="status">
            <strong>Status:</strong> Server is running successfully!
        </div>
        <p>Welcome to the embedded HTTP server running on your Raspberry Pi Pico 2W!</p>
        
        <h3>Available Endpoints:</h3>
        <ul>
            <li><a href="/gpio" class="button">🔌 /gpio</a> - GPIO Pin 16 Control</li>
            <li><a href="/bluetooth" class="button">🦷 /bluetooth</a> - Bluetooth Status</li>
            <li><a href="/status" class="button">📊 /status</a> - System status (JSON)</li>
            <li><a href="/info" class="button">ℹ️ /info</a> - Device information</li>
            <li><a href="/api/health" class="button">🏥 /api/health</a> - Health check</li>
        </ul>
        
        <hr>
        <p><em>Powered by Embassy on Rust 🦀</em></p>
    </div>
</body>
</html>
    "#).ok();
    response
}

pub fn handle_status(_request: &HttpRequest) -> HttpResponse {
    HttpResponse::json(
        r#"{"status":"ok","uptime":"running","memory":"available","connections":"active"}"#,
    )
}

pub fn handle_info(_request: &HttpRequest) -> HttpResponse {
    let mut response = HttpResponse::ok();
    response.body.push_str(r#"
<!DOCTYPE html>
<html>
<head>
    <title>Device Info - Pico 2W</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; background: #f0f0f0; }
        .container { background: white; padding: 20px; border-radius: 10px; }
        .info-item { background: #f9f9f9; padding: 10px; margin: 5px 0; border-left: 4px solid #4CAF50; }
    </style>
</head>
<body>
    <div class="container">
        <h1>Device Information</h1>
        <div class="info-item"><strong>Device:</strong> Raspberry Pi Pico 2W</div>
        <div class="info-item"><strong>Firmware:</strong> Embassy Rust</div>
        <div class="info-item"><strong>WiFi:</strong> CYW43439</div>
        <div class="info-item"><strong>Bluetooth:</strong> CYW43439 (Module Ready)</div>
        <div class="info-item"><strong>HTTP Server:</strong> Custom Implementation</div>
        <a href="/">← Back to Home</a>
    </div>
</body>
</html>
    "#).ok();
    response
}

pub fn handle_health(_request: &HttpRequest) -> HttpResponse {
    HttpResponse::json(r#"{"health":"healthy","timestamp":"now","version":"1.0.0"}"#)
}

pub fn handle_gpio_control(_request: &HttpRequest) -> HttpResponse {
    let mut response = HttpResponse::ok();
    response
        .body
        .push_str(
            r#"
<!DOCTYPE html>
<html>
<head>
    <title>GPIO Control - Pin 16</title>
</head>
<body>
    <h1>GPIO Pin 16 Control</h1>
    
    <p><strong>Pin Status:</strong> Ready for control</p>
    
    <button onclick="setGpioHigh()">SET HIGH (3.3V)</button>
    <button onclick="setGpioLow()">SET LOW (0V)</button>
    
    <div id="result"></div>
    
    <p><a href="/">Back to Home</a></p>

    <script>
        function setGpioHigh() {
            fetch('/api/gpio/high', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    document.getElementById('result').innerHTML = 
                        '<p>GPIO Pin 16 set to HIGH (3.3V)</p>';
                })
                .catch(error => {
                    document.getElementById('result').innerHTML = 
                        '<p>Error: ' + error + '</p>';
                });
        }
        
        function setGpioLow() {
            fetch('/api/gpio/low', { method: 'POST' })
                .then(response => response.json())
                .then(data => {
                    document.getElementById('result').innerHTML = 
                        '<p>GPIO Pin 16 set to LOW (0V)</p>';
                })
                .catch(error => {
                    document.getElementById('result').innerHTML = 
                        '<p>Error: ' + error + '</p>';
                });
        }
    </script>
</body>
</html>
    "#,
        )
        .ok();
    response
}

pub fn handle_gpio_high(_request: &HttpRequest) -> HttpResponse {
    // Send command to GPIO control task
    let sender = GPIO_CHANNEL.sender();
    match sender.try_send(GpioCommand::SetHigh) {
        Ok(()) => {
            log_info!("🔴 GPIO Pin 16 commanded to HIGH");
            HttpResponse::json(r#"{"status":"ok","action":"gpio_high","pin":16,"voltage":"3.3V"}"#)
        }
        Err(_) => {
            log_info!("❌ Failed to send GPIO HIGH command");
            HttpResponse::error("Failed to control GPIO")
        }
    }
}

pub fn handle_gpio_low(_request: &HttpRequest) -> HttpResponse {
    // Send command to GPIO control task
    let sender = GPIO_CHANNEL.sender();
    match sender.try_send(GpioCommand::SetLow) {
        Ok(()) => {
            log_info!("🔵 GPIO Pin 16 commanded to LOW");
            HttpResponse::json(r#"{"status":"ok","action":"gpio_low","pin":16,"voltage":"0V"}"#)
        }
        Err(_) => {
            log_info!("❌ Failed to send GPIO LOW command");
            HttpResponse::error("Failed to control GPIO")
        }
    }
}

pub fn handle_bluetooth_status(_request: &HttpRequest) -> HttpResponse {
    // let bt_status = crate::bluetooth::get_bluetooth_status();
    let mut response = HttpResponse::ok();
    response
        .body
        .push_str(
            r#"
<!DOCTYPE html>
<html>
<head>
    <title>Simple Bluetooth - Pico 2W</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; background: #f0f0f0; }
        .container { background: white; padding: 20px; border-radius: 10px; }
        .status { background: #e8f5e8; padding: 15px; margin: 10px 0; border-radius: 8px; }
    </style>
</head>
<body>
    <div class="container">
        <h1>🦷 Simple Bluetooth</h1>
        
        <div class="status">
            <p><strong>Device:</strong> Pico2W-BLE</p>
            <p><strong>Status:</strong> Running</p>
            <p><strong>Mode:</strong> Simple Demo</p>
        </div>
        
        <p>This is a simple Bluetooth demonstration running on CYW43439.</p>
        <p>Check the logs to see advertising activity.</p>
        
        <p><a href="/">← Back to Home</a></p>
    </div>
</body>
</html>
    "#,
        )
        .ok();
    response
}

/// HTTP Server task that accepts connections and handles requests
#[embassy_executor::task]
pub async fn http_server_task(stack: Stack<'static>) -> ! {
    log_info!("🌐 Starting HTTP server on port {}", HTTP_SERVER_PORT);

    // Set up router with default routes
    let mut router = HttpRouter::new();
    log_info!("🔧 Setting up HTTP routes...");
    router
        .get("/", handle_root)
        .get("/status", handle_status)
        .get("/info", handle_info)
        .get("/bluetooth", handle_bluetooth_status)
        .get("/api/health", handle_health)
        .get("/gpio", handle_gpio_control)
        .post("/api/gpio/high", handle_gpio_high)
        .post("/api/gpio/low", handle_gpio_low);

    log_info!(
        "✅ HTTP routes configured: {} total routes",
        router.routes.len()
    );

    // Server socket buffers
    let mut rx_buffer = [0; HTTP_BUFFER_SIZE];
    let mut tx_buffer = [0; HTTP_BUFFER_SIZE];

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);

        log_info!(
            "📡 Listening for HTTP connections on port {}",
            HTTP_SERVER_PORT
        );
        if let Err(e) = socket.accept(HTTP_SERVER_PORT).await {
            log_info!("❌ Accept error: {:?}", e);
            Timer::after(Duration::from_millis(100)).await;
            continue;
        }

        log_info!("✅ New HTTP connection accepted from client");

        // Handle the connection
        if let Err(e) = handle_connection(&mut socket, &router).await {
            log_info!("⚠️ Connection handling error: {:?}", e);
        }

        log_info!("🔚 Connection closed");
    }
}

/// Handle a single HTTP connection
async fn handle_connection(
    socket: &mut TcpSocket<'_>,
    router: &HttpRouter,
) -> Result<(), &'static str> {
    let mut buffer = [0; HTTP_BUFFER_SIZE];
    let mut total_received = 0;

    // Read the HTTP request
    loop {
        match socket.read(&mut buffer[total_received..]).await {
            Ok(0) => {
                log_info!("❌ Client closed connection during read");
                return Err("Client disconnected");
            }
            Ok(n) => {
                total_received += n;

                // Check if we have a complete HTTP request (ends with \r\n\r\n)
                let received_data = &buffer[..total_received];
                if let Ok(request_str) = core::str::from_utf8(received_data) {
                    if request_str.contains("\r\n\r\n") {
                        break;
                    }
                }

                // Prevent buffer overflow
                if total_received >= buffer.len() - 1 {
                    log_info!("⚠️ Request too large, processing partial request");
                    break;
                }
            }
            Err(e) => {
                log_info!("❌ Read error: {:?}", e);
                return Err("Read error");
            }
        }
    }

    // Parse and handle the request
    let request_str =
        core::str::from_utf8(&buffer[..total_received]).map_err(|_| "Invalid UTF-8 in request")?;

    log_info!("📥 Received HTTP request: {} bytes", total_received);
    log_info!("📋 Full request: {}", request_str);

    let response = match parse_http_request(request_str) {
        Ok(request) => {
            log_info!("🔍 Parsed request: {:?} {}", request.method, request.path);
            router.handle_request(&request)
        }
        Err(e) => {
            log_info!("❌ Failed to parse request: {}", e);
            HttpResponse::error("Bad Request")
        }
    };

    // Send the response
    let response_bytes = response.to_bytes();
    log_info!("📤 Sending HTTP response: {} bytes", response_bytes.len());

    match socket.write_all(&response_bytes).await {
        Ok(()) => {
            log_info!("✅ Response sent successfully");
            socket.flush().await.ok();
        }
        Err(e) => {
            log_info!("❌ Write error: {:?}", e);
            return Err("Write error");
        }
    }

    Ok(())
}

#[derive(Clone, Copy, Debug)]
pub enum GpioCommand {
    SetHigh,
    SetLow,
}
