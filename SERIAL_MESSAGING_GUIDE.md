# Serial Messaging

Thread-safe messaging system for USB serial output.

## Usage

```rust
use crate::usb_serial::{send_message, send_adc_reading, send_formatted_message};

// Simple text
send_message("ADC started").await;

// ADC readings (formatted as "[ADC] Channel X: Y mV")
send_adc_reading(0, 1650).await;

// Custom formatted messages
let mut msg: String<128> = String::new();
write!(&mut msg, "Temp: {:.1}°C", 23.5).unwrap();
send_formatted_message(msg).await;
```

## Message Types
```rust
pub enum SerialMessage {
    Text(&'static str),
    AdcReading { channel: u8, value: u16 },
    Formatted(String<128>),
}
```

## Connection
```bash
screen /dev/ttyACM0 115200
``` 