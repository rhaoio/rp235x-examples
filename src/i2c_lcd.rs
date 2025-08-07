//! # I2C LCD Module
//!
//! This module provides control for LCD1602 displays connected via PCF8574 I2C I/O expander.
//! The PCF8574 is commonly used as a backpack to convert parallel LCD interface to I2C.
//!
//! ## Troubleshooting Gibberish Text
//!
//! If you see gibberish text, try:
//! 1. Check wiring: SDA→GPIO4, SCL→GPIO5, VCC→5V, GND→GND
//! 2. Adjust contrast potentiometer on LCD backpack
//! 3. Try slower I2C speed (50kHz)
//! 4. Check power supply is 5V, not 3.3V
//!
//! ## Wiring
//!
//! Standard I2C LCD1602 with PCF8574 backpack:
//! - VCC -> 5V (LCD requires 5V, but I2C can work at 3.3V)
//! - GND -> GND
//! - SDA -> GPIO 4 (I2C0 SDA)
//! - SCL -> GPIO 5 (I2C0 SCL)
//!
//! ## PCF8574 Pin Mapping to LCD
//!
//! ```
//! PCF8574 Pin | LCD Pin | Function
//! ------------|---------|----------
//! P0          | RS      | Register Select
//! P1          | RW      | Read/Write (tied to GND)
//! P2          | E       | Enable
//! P3          | Backlight | LED+ (via transistor)
//! P4          | D4      | Data bit 4
//! P5          | D5      | Data bit 5
//! P6          | D6      | Data bit 6
//! P7          | D7      | Data bit 7
//! ```
//!
//! ## Usage Examples
//!
//! ### Basic Text Display
//!
//! ```rust
//! // In main.rs:
//! spawner.spawn(i2c_lcd::lcd_demo_task(p.I2C0, p.PIN_4, p.PIN_5)).unwrap();
//! ```

use embassy_rp::{
    i2c::{Blocking, Config as I2cConfig, I2c},
    peripherals::{I2C0, PIN_4, PIN_5},
    Peri,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use heapless::String;

use crate::constants::{buffers, i2c};
use crate::usb_serial::send_sync_formatted_message;

/// Default I2C address for PCF8574 (can be 0x20-0x27 depending on A0,A1,A2 jumpers)
pub const PCF8574_DEFAULT_ADDRESS: u8 = i2c::PCF8574_ADDR_MAX;

/// Alternative common I2C address for PCF8574
pub const PCF8574_ALT_ADDRESS: u8 = i2c::PCF8574_ALT_ADDR;

/// PCF8574 to LCD1602 Pin Mapping (Critical for understanding the "magic bytes")
///
/// The PCF8574 I/O expander has 8 pins (P0-P7) that connect to the LCD as follows:
///
/// PCF8574 Pin | LCD Pin | Function        | Bit Position
/// ------------|---------|-----------------|-------------
/// P0          | RS      | Register Select | Bit 0 (0x01)
/// P1          | RW      | Read/Write      | Bit 1 (0x02) - Always 0 (write mode)
/// P2          | EN      | Enable Clock    | Bit 2 (0x04)
/// P3          | BL      | Backlight       | Bit 3 (0x08)
/// P4          | D4      | Data Bit 4      | Bit 4 (0x10)
/// P5          | D5      | Data Bit 5      | Bit 5 (0x20)
/// P6          | D6      | Data Bit 6      | Bit 6 (0x40)
/// P7          | D7      | Data Bit 7      | Bit 7 (0x80)
///
/// Key Control Bits:
/// - RS=0: Command mode (writing instructions to LCD controller)
/// - RS=1: Data mode (writing characters to display)
/// - EN: Must pulse HIGH→LOW to clock data into LCD (enable strobe)
/// - BL: Controls backlight (1=on, 0=off)
///
/// 4-Bit Communication Protocol:
/// Each 8-bit value is sent as TWO 4-bit nibbles (high nibble first, then low nibble)
/// This requires TWO I2C transactions per LCD byte.

/// Constants for PCF8574 bit manipulation
/// These are the "magic numbers" that control LCD behavior
pub mod pcf8574_bits {
    /// Register Select: 0=Command mode, 1=Data mode
    pub const RS: u8 = 0x01; // P0 -> LCD RS pin

    /// Read/Write: 0=Write, 1=Read (we always write, so this stays 0)
    pub const RW: u8 = 0x02; // P1 -> LCD RW pin

    /// Enable: Must pulse HIGH then LOW to clock data into LCD
    pub const EN: u8 = 0x04; // P2 -> LCD EN pin

    /// Backlight control: 1=On, 0=Off
    pub const BACKLIGHT: u8 = 0x08; // P3 -> LCD backlight control

    /// Data bits occupy the upper 4 bits (P4-P7 -> LCD D4-D7)
    pub const DATA_MASK: u8 = 0xF0; // Bits 4-7 for LCD data lines
}

/// LCD1602 Commands
pub mod lcd_commands {
    pub const CLEAR_DISPLAY: u8 = 0x01;
    pub const RETURN_HOME: u8 = 0x02;
    pub const ENTRY_MODE_SET: u8 = 0x04;
    pub const DISPLAY_CONTROL: u8 = 0x08;
    pub const CURSOR_SHIFT: u8 = 0x10;
    pub const FUNCTION_SET: u8 = 0x20;
    pub const SET_CGRAM_ADDR: u8 = 0x40;
    pub const SET_DDRAM_ADDR: u8 = 0x80;

    // Entry mode flags
    pub const ENTRY_RIGHT: u8 = 0x00;
    pub const ENTRY_LEFT: u8 = 0x02;
    pub const ENTRY_SHIFT_INCREMENT: u8 = 0x01;
    pub const ENTRY_SHIFT_DECREMENT: u8 = 0x00;

    // Display control flags
    pub const DISPLAY_ON: u8 = 0x04;
    pub const DISPLAY_OFF: u8 = 0x00;
    pub const CURSOR_ON: u8 = 0x02;
    pub const CURSOR_OFF: u8 = 0x00;
    pub const BLINK_ON: u8 = 0x01;
    pub const BLINK_OFF: u8 = 0x00;

    // Function set flags
    pub const FOUR_BIT_MODE: u8 = 0x00;
    pub const EIGHT_BIT_MODE: u8 = 0x10;
    pub const ONE_LINE: u8 = 0x00;
    pub const TWO_LINE: u8 = 0x08;
    pub const FONT_5X8: u8 = 0x00;
    pub const FONT_5X10: u8 = 0x04;
}

/// LCD message for channel communication
#[derive(Clone)]
pub struct LcdMessage {
    pub text: String<32>, // Max 32 characters per message
    pub row: u8,
    pub col: u8,
    pub clear_first: bool,
}

impl LcdMessage {
    pub fn new(text: &str, row: u8, col: u8) -> Self {
        let mut msg_text = String::new();
        let _ = msg_text.push_str(text);
        Self {
            text: msg_text,
            row,
            col,
            clear_first: false,
        }
    }

    pub fn with_clear(text: &str, row: u8, col: u8) -> Self {
        let mut msg_text = String::new();
        let _ = msg_text.push_str(text);
        Self {
            text: msg_text,
            row,
            col,
            clear_first: true,
        }
    }
}

/// Channel for sending LCD messages between tasks
pub static LCD_MESSAGE_CHANNEL: Channel<
    ThreadModeRawMutex,
    LcdMessage,
    { buffers::SERIAL_CHANNEL_DEPTH },
> = Channel::new();

/// Helper function to send debug messages via USB serial
fn send_debug_msg(msg: &str) {
    let mut debug_msg = String::<{ buffers::MESSAGE_BUFFER_SIZE }>::new();
    let _ = core::fmt::write(&mut debug_msg, format_args!("[LCD] {}", msg));
    send_sync_formatted_message(debug_msg);
}

/// LCD1602 controller structure
pub struct Lcd1602<'d> {
    i2c: I2c<'d, I2C0, Blocking>,
    address: u8,
    backlight_on: bool,
}

impl<'d> Lcd1602<'d> {
    /// Create new LCD controller
    pub fn new(i2c: I2c<'d, I2C0, Blocking>, address: u8) -> Self {
        Self {
            i2c,
            address,
            backlight_on: true,
        }
    }

    /// Initialize the LCD in 4-bit mode with improved timing
    /// This sequence is mandated by the HD44780 datasheet
    pub async fn init(&mut self) -> Result<(), embassy_rp::i2c::Error> {
        let mut debug_msg = String::<64>::new();
        let _ = core::fmt::write(
            &mut debug_msg,
            format_args!("Initializing LCD at 0x{:02x}", self.address),
        );
        send_debug_msg(&debug_msg);

        // Wait for LCD to power up (longer delay for stability)
        Timer::after(Duration::from_millis(50)).await;

        // HD44780 Initialization Sequence for 4-bit mode:
        // This is the official initialization sequence from the datasheet

        // Step 1: Send 0x03 three times (this forces LCD into a known state)
        // Why 0x03? It's the "function set" command for 8-bit mode
        // We send it in 4-bit format to reset any confusion about current mode
        self.send_nibble(0x03, false).await?; // First reset attempt
        Timer::after(Duration::from_millis(5)).await;

        self.send_nibble(0x03, false).await?; // Second reset attempt
        Timer::after(Duration::from_millis(1)).await;

        self.send_nibble(0x03, false).await?; // Third reset attempt
        Timer::after(Duration::from_millis(1)).await;

        // Step 2: Switch to 4-bit mode
        // Send 0x02 to tell LCD we're using 4-bit interface
        self.send_nibble(0x02, false).await?; // Switch to 4-bit mode
        Timer::after(Duration::from_millis(1)).await;

        // Step 3: Now we can use full 8-bit commands (sent as two 4-bit nibbles)
        // Function Set: 4-bit mode, 2 lines, 5x8 font
        // Command breakdown: 0x28 = 0010 1000
        // Bit 5=0: 4-bit mode, Bit 3=1: 2 lines, Bit 2=0: 5x8 font
        self.send_byte(
            lcd_commands::FUNCTION_SET | lcd_commands::TWO_LINE | lcd_commands::FONT_5X8,
            false,
        )
        .await?;
        Timer::after(Duration::from_millis(1)).await;

        // Display Control: Display on, cursor off, blink off
        // Command breakdown: 0x0C = 0000 1100
        // Bit 2=1: Display on, Bit 1=0: Cursor off, Bit 0=0: Blink off
        self.send_byte(
            lcd_commands::DISPLAY_CONTROL
                | lcd_commands::DISPLAY_ON
                | lcd_commands::CURSOR_OFF
                | lcd_commands::BLINK_OFF,
            false,
        )
        .await?;
        Timer::after(Duration::from_millis(1)).await;

        // Clear Display
        // This clears all display data and returns cursor to home position
        self.send_byte(lcd_commands::CLEAR_DISPLAY, false).await?;
        Timer::after(Duration::from_millis(2)).await; // Clear needs extra time

        // Entry Mode: Increment cursor, no display shift
        // Command breakdown: 0x06 = 0000 0110
        // Bit 1=1: Increment cursor after each character
        self.send_byte(
            lcd_commands::ENTRY_MODE_SET | lcd_commands::ENTRY_LEFT,
            false,
        )
        .await?;
        Timer::after(Duration::from_millis(1)).await;

        send_debug_msg("LCD initialization completed successfully");
        Ok(())
    }

    /// Clear the display
    pub async fn clear(&mut self) -> Result<(), embassy_rp::i2c::Error> {
        self.send_byte(lcd_commands::CLEAR_DISPLAY, false).await?;
        Timer::after(Duration::from_millis(2)).await; // Clear command takes longer
        Ok(())
    }

    /// Set cursor position on the display
    /// LCD1602 has two rows with specific memory addresses due to HD44780 internal architecture
    pub async fn set_cursor(&mut self, row: u8, col: u8) -> Result<(), embassy_rp::i2c::Error> {
        // LCD1602 memory mapping (why these specific addresses):
        // Row 0: starts at address 0x00 (displayed as top row)
        // Row 1: starts at address 0x40 (displayed as bottom row, NOT 0x10!)
        // This quirky addressing is due to the HD44780's internal DDRAM layout
        let row_offset = [0x00u8, 0x40u8]; // Row start addresses
        let address = lcd_commands::SET_DDRAM_ADDR | (col + row_offset[row as usize]);

        // Send "Set DDRAM Address" command (0x80 | address)
        // Example: Position (1,5) -> 0x80 | (0x40 + 5) = 0xC5
        self.send_byte(address, false).await?;
        Timer::after(Duration::from_micros(50)).await; // Small delay after cursor set
        Ok(())
    }

    /// Print text at current cursor position
    pub async fn print(&mut self, text: &str) -> Result<(), embassy_rp::i2c::Error> {
        for byte in text.bytes() {
            self.send_byte(byte, true).await?;
            // Small delay between characters for stability
            Timer::after(Duration::from_micros(100)).await;
        }
        Ok(())
    }

    /// Print text at specific position
    pub async fn print_at(
        &mut self,
        row: u8,
        col: u8,
        text: &str,
    ) -> Result<(), embassy_rp::i2c::Error> {
        self.set_cursor(row, col).await?;
        self.print(text).await
    }

    /// Control backlight
    pub async fn set_backlight(&mut self, on: bool) -> Result<(), embassy_rp::i2c::Error> {
        self.backlight_on = on;
        // Send a dummy command to update backlight state
        self.send_byte(0, false).await
    }

    /// Send command to LCD
    async fn send_command(&mut self, cmd: u8) -> Result<(), embassy_rp::i2c::Error> {
        self.send_byte(cmd, false).await
    }

    /// Send data to LCD
    async fn send_data(&mut self, data: u8) -> Result<(), embassy_rp::i2c::Error> {
        self.send_byte(data, true).await
    }

    /// Send a single nibble (4 bits) to the LCD via PCF8574
    /// This is where the "magic" happens - converting 4-bit LCD data to 8-bit I2C data
    async fn send_nibble(&mut self, nibble: u8, rs: bool) -> Result<(), embassy_rp::i2c::Error> {
        // Step 1: Prepare the byte for PCF8574
        // We need to map 4 LCD data bits to the upper 4 bits of PCF8574 (P4-P7)
        let data_bits = (nibble & 0x0F) << 4; // Shift 4-bit nibble to upper position (D4-D7)

        // Step 2: Set control bits
        let rs_bit = if rs { pcf8574_bits::RS } else { 0 }; // Command vs Data mode
        let rw_bit = 0; // Always 0 (write mode)
        let backlight_bit = if self.backlight_on {
            pcf8574_bits::BACKLIGHT
        } else {
            0
        };

        // Step 3: Combine all bits into final byte
        // Format: [D7 D6 D5 D4] [BL EN RW RS]
        //         P7 P6 P5 P4   P3 P2 P1 P0
        let base_byte = data_bits | backlight_bit | rw_bit | rs_bit;

        // Step 4: Send with Enable HIGH (data setup)
        // The LCD reads data on the falling edge of EN, so we:
        // 1. Set EN=1 with data present (setup time)
        let enable_high = base_byte | pcf8574_bits::EN;
        self.write_i2c_byte(enable_high).await?;
        Timer::after(Duration::from_micros(1)).await; // Setup time

        // 2. Set EN=0 (falling edge triggers LCD to read the data)
        let enable_low = base_byte; // Same data, but EN=0
        self.write_i2c_byte(enable_low).await?;
        Timer::after(Duration::from_micros(50)).await; // Hold time

        Ok(())
    }

    /// Send a complete 8-bit value to LCD by splitting into two 4-bit nibbles
    /// This is why we need "two transactions" for each LCD command/character
    async fn send_byte(&mut self, value: u8, rs: bool) -> Result<(), embassy_rp::i2c::Error> {
        // Send high nibble first (bits 7-4)
        // Example: For value 0x48 ('H'), high nibble = 0x04
        self.send_nibble((value >> 4) & 0x0F, rs).await?;

        // Then send low nibble (bits 3-0)
        // Example: For value 0x48 ('H'), low nibble = 0x08
        self.send_nibble(value & 0x0F, rs).await?;

        Ok(())
    }

    /// Write a single character to the current cursor position
    /// Uses RS=1 (data mode) to send character data instead of commands
    pub async fn write_char(&mut self, ch: char) -> Result<(), embassy_rp::i2c::Error> {
        // Convert character to ASCII byte
        let ascii_byte = ch as u8;

        // Send with RS=1 (data mode) - this tells LCD it's character data, not a command
        // The LCD will automatically store this in its display RAM and show it on screen
        self.send_byte(ascii_byte, true).await?;
        Timer::after(Duration::from_micros(50)).await; // Character write time
        Ok(())
    }

    /// Low-level I2C byte write - sends one byte directly to PCF8574
    /// This is the fundamental building block for all LCD communication
    async fn write_i2c_byte(&mut self, data: u8) -> Result<(), embassy_rp::i2c::Error> {
        // Send single byte to PCF8574 via I2C
        // The PCF8574 will immediately output this byte on its P0-P7 pins
        self.i2c.blocking_write(self.address, &[data])
    }

    /// Detect available PCF8574 device addresses
    async fn scan_addresses(&mut self) -> u8 {
        let mut debug_msg = String::<64>::new();
        let _ = core::fmt::write(&mut debug_msg, format_args!("Scanning I2C addresses..."));
        send_debug_msg(&debug_msg);

        // Try standard PCF8574 addresses
        let addresses = [0x27, 0x3F, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21, 0x20];

        for &addr in addresses.iter() {
            // Try a simple write to see if device responds
            if self.i2c.blocking_write(addr, &[0x00]).is_ok() {
                let mut found_msg = String::<64>::new();
                let _ = core::fmt::write(
                    &mut found_msg,
                    format_args!("Found PCF8574 at 0x{:02x}", addr),
                );
                send_debug_msg(&found_msg);
                return addr;
            }
        }

        send_debug_msg("No PCF8574 found, using default 0x27");
        0x27 // Default address
    }
}

/// LCD controller task that receives messages via channel
#[embassy_executor::task]
pub async fn lcd_controller_task(
    i2c_peripheral: Peri<'static, I2C0>,
    sda_pin: Peri<'static, PIN_4>,
    scl_pin: Peri<'static, PIN_5>,
) -> ! {
    send_debug_msg("Starting LCD controller task");

    // Initialize I2C
    let mut config = I2cConfig::default();
    config.frequency = 100_000; // 100kHz for reliable communication

    let i2c = I2c::new_blocking(i2c_peripheral, scl_pin, sda_pin, config);

    let mut lcd = Lcd1602::new(i2c, PCF8574_DEFAULT_ADDRESS);

    // Initialize LCD
    match lcd.init().await {
        Ok(()) => send_debug_msg("LCD initialized successfully"),
        Err(_) => {
            send_debug_msg("Failed with default address, trying alt");
            lcd.address = PCF8574_ALT_ADDRESS;
            match lcd.init().await {
                Ok(()) => send_debug_msg("LCD initialized with alt address"),
                Err(_) => send_debug_msg("Failed with both addresses"),
            }
        }
    }

    let receiver = LCD_MESSAGE_CHANNEL.receiver();

    loop {
        let message = receiver.receive().await;

        if message.clear_first {
            let _ = lcd.clear().await;
        }

        match lcd.print_at(message.row, message.col, &message.text).await {
            Ok(()) => {
                let mut debug_msg = String::<64>::new();
                let _ = core::fmt::write(
                    &mut debug_msg,
                    format_args!(
                        "Printed '{}' at ({},{})",
                        message.text, message.row, message.col
                    ),
                );
                send_debug_msg(&debug_msg);
            }
            Err(_) => send_debug_msg("LCD write error"),
        }
    }
}

/// Demo task for debugging LCD issues
#[embassy_executor::task]
pub async fn lcd_demo_task(
    i2c_peripheral: Peri<'static, I2C0>,
    sda_pin: Peri<'static, PIN_4>,
    scl_pin: Peri<'static, PIN_5>,
) -> ! {
    send_debug_msg("Starting LCD demo/debug task");

    // Try different I2C speeds for debugging
    let mut config = I2cConfig::default();
    config.frequency = 50_000; // Start with conservative 50kHz

    let i2c = I2c::new_blocking(i2c_peripheral, scl_pin, sda_pin, config);

    let mut lcd = Lcd1602::new(i2c, PCF8574_DEFAULT_ADDRESS);

    // Initialize LCD with debug output
    let mut init_success = false;
    let addresses_to_try = [
        PCF8574_DEFAULT_ADDRESS,
        PCF8574_ALT_ADDRESS,
        0x20,
        0x21,
        0x22,
        0x23,
        0x24,
        0x25,
        0x26,
    ];

    for &addr in &addresses_to_try {
        lcd.address = addr;
        let mut debug_msg = String::<64>::new();
        let _ = core::fmt::write(
            &mut debug_msg,
            format_args!("Trying address 0x{:02x}", addr),
        );
        send_debug_msg(&debug_msg);

        match lcd.init().await {
            Ok(()) => {
                let mut success_msg = String::<64>::new();
                let _ =
                    core::fmt::write(&mut success_msg, format_args!("SUCCESS at 0x{:02x}", addr));
                send_debug_msg(&success_msg);
                init_success = true;
                break;
            }
            Err(_) => {
                let mut fail_msg = String::<64>::new();
                let _ = core::fmt::write(&mut fail_msg, format_args!("FAILED at 0x{:02x}", addr));
                send_debug_msg(&fail_msg);
            }
        }
        Timer::after(Duration::from_millis(500)).await;
    }

    if !init_success {
        send_debug_msg("CRITICAL: No LCD found at any address!");
        send_debug_msg("Check: 1) Wiring 2) Power (5V) 3) Contrast pot");
        loop {
            Timer::after(Duration::from_secs(5)).await;
            send_debug_msg("Still no LCD...");
        }
    }

    let mut test_phase = 0u8;

    loop {
        match test_phase {
            0 => {
                // Test 1: Simple single character
                send_debug_msg("Test 1: Single character");
                let _ = lcd.clear().await;
                let _ = lcd.print_at(0, 0, "A").await;
                Timer::after(Duration::from_secs(3)).await;
            }
            1 => {
                // Test 2: Simple word
                send_debug_msg("Test 2: Simple word");
                let _ = lcd.clear().await;
                let _ = lcd.print_at(0, 0, "HELLO").await;
                Timer::after(Duration::from_secs(3)).await;
            }
            2 => {
                // Test 3: Numbers
                send_debug_msg("Test 3: Numbers");
                let _ = lcd.clear().await;
                let _ = lcd.print_at(0, 0, "12345").await;
                Timer::after(Duration::from_secs(3)).await;
            }
            3 => {
                // Test 4: Both lines
                send_debug_msg("Test 4: Both lines");
                let _ = lcd.clear().await;
                let _ = lcd.print_at(0, 0, "Line 1").await;
                let _ = lcd.print_at(1, 0, "Line 2").await;
                Timer::after(Duration::from_secs(3)).await;
            }
            4 => {
                // Test 5: Backlight control
                send_debug_msg("Test 5: Backlight");
                let _ = lcd.clear().await;
                let _ = lcd.print_at(0, 0, "Backlight Test").await;

                for i in 0..3 {
                    let mut blink_msg = String::<32>::new();
                    let _ = core::fmt::write(&mut blink_msg, format_args!("Blink {}", i + 1));
                    send_debug_msg(&blink_msg);

                    let _ = lcd.set_backlight(false).await;
                    Timer::after(Duration::from_millis(500)).await;
                    let _ = lcd.set_backlight(true).await;
                    Timer::after(Duration::from_millis(500)).await;
                }
                Timer::after(Duration::from_secs(2)).await;
            }
            _ => {
                // Reset test cycle
                test_phase = 0;
                continue;
            }
        }

        test_phase += 1;
        if test_phase > 4 {
            test_phase = 0;
            send_debug_msg("=== Test cycle complete, restarting ===");
            Timer::after(Duration::from_secs(2)).await;
        }
    }
}
