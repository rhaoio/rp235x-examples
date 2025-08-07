# USB Communication

Bidirectional USB serial communication for Pico 2 W.

## Quick Start
```bash
cargo build --release
# Flash .uf2 to Pico 2 W in bootloader mode
screen /dev/ttyACM0 115200
```

## Commands
- `status` - Device status
- `led_on/led_off` - LED control
- `echo <text>` - Echo text back

## Troubleshooting
```bash
# Find device
ls /dev/ttyACM*

# Fix permissions
sudo usermod -a -G dialout $USER

# Screen commands
screen -ls          # list sessions
screen -r           # reattach
pkill screen        # kill all
```

## Adding Commands
Edit `process_command` in `src/usb_serial.rs`:
```rust
} else if command_bytes == b"your_command" {
    let _ = class.write_packet(b"Response\r\n").await;
}
``` 