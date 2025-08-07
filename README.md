# Pico 2 W Quick Start Examples

Embassy-based examples for Raspberry Pi Pico 2 W with various peripherals. Mostly from Freenove examples/tutorials and adapted to Rust.

Adapted from:
https://github.com/ImplFerris/pico2-quick

## Quick Start

After the device is connected/accessible from your environment, just run:

```bash
cargo run --release
# Copy target/thumbv8m.main-none-eabihf/release/*.uf2 to Pico 2 W in bootloader mode
```

## Examples

Each `.rs` file in `src/` is a standalone example. Copy the desired example to `main.rs` and build.

## Communication

- USB Serial: Connect via `screen /dev/ttyACM0 115200`
- WiFi: HTTP server and wireless communication examples included

## Hardware

Supports common peripherals: ADC, I2C LCD, servos, motors, sensors, LEDs, and more.
