//! 74HC595 Shift Register LED Bar Control Module
//!
//! This module provides control for an 8-LED bar using a 74HC595 shift register.
//!
//! ## How the 74HC595 Works
//!
//! The 74HC595 is a serial-to-parallel shift register that converts serial data input
//! into 8 parallel outputs. It works in three stages:
//!
//! 1. **Serial Data Input**: Data bits are fed serially into the DS (Data Serial) pin
//! 2. **Shift Register**: Each pulse on SH_CP (Shift Register Clock) shifts one bit into
//!    an internal 8-bit shift register. Bits enter at DS and existing bits move down the chain.
//! 3. **Output Register**: When ST_CP (Storage Register Clock/Latch) is pulsed, all 8 bits
//!    from the shift register are copied to the output register, which drives pins Q0-Q7.
//!
//! This allows us to control 8 LEDs using only 3 GPIO pins from the microcontroller.
//!
//! ## LED Control Process
//!
//! 1. We maintain an 8-bit value representing LED states (1=on, 0=off)
//! 2. To update LEDs, we send this 8-bit value serially (MSB first) to the shift register
//! 3. For each bit: set DS pin high/low, pulse SH_CP to shift it in
//! 4. After all 8 bits are shifted in, pulse ST_CP to update the LED outputs
//!
//! ## Wiring
//! - DS (Data Serial) - Connected to GPIO 18
//! - ST_CP (Storage Register Clock/Latch) - Connected to GPIO 20  
//! - SH_CP (Shift Register Clock) - Connected to GPIO 21
//! - VCC - Connect to 3.3V or 5V
//! - GND - Connect to ground
//! - Q0-Q7 - Connect to LEDs (with appropriate current limiting resistors ~220-470Ω)
//!
//! ## Usage
//! ```rust
//! // In main.rs, spawn the demo task:
//! unwrap!(spawner.spawn(shift_register::shift_register_task(
//!     p.PIN_18, // DS (Data Serial)
//!     p.PIN_20, // ST_CP (Storage Register Clock/Latch)
//!     p.PIN_21  // SH_CP (Shift Register Clock)
//! )));
//!
//! // Or use manually:
//! let data_pin = Output::new(p.PIN_18, Level::Low);
//! let latch_pin = Output::new(p.PIN_20, Level::Low);
//! let clock_pin = Output::new(p.PIN_21, Level::Low);
//! let mut shift_reg = ShiftRegister::new(data_pin, latch_pin, clock_pin);
//!
//! // Control individual LEDs
//! shift_reg.set_led(0, true);  // Turn on LED 0 (connected to Q0)
//! shift_reg.set_led(7, false); // Turn off LED 7 (connected to Q7)
//! shift_reg.update_async().await; // Apply changes to physical LEDs
//!
//! // Set pattern directly (8-bit binary pattern)
//! shift_reg.set_pattern(0b10101010); // 0xAA - Alternating pattern (Q0,Q2,Q4,Q6 on)
//! shift_reg.update_async().await;
//! ```

use defmt::*;
use embassy_rp::{
    gpio::{Level, Output},
    Peri,
};
use embassy_time::{Duration, Timer};
use heapless::String;

use crate::usb_serial::{send_formatted_message, send_message};

/// Number of LEDs in the shift register LED bar
pub const NUM_SHIFT_LEDS: usize = 8;

/// 74HC595 Shift Register Controller
/// Controls an 8-LED bar using a 74HC595 shift register
pub struct ShiftRegister<'a> {
    data_pin: Output<'a>,  // DS (Data Serial) - pin 18
    latch_pin: Output<'a>, // ST_CP (Storage Register Clock) - pin 20
    clock_pin: Output<'a>, // SH_CP (Shift Register Clock) - pin 21
    led_state: u8,         // Current state of all 8 LEDs (bit pattern)
}

impl<'a> ShiftRegister<'a> {
    /// Create a new ShiftRegister controller
    pub fn new(data_pin: Output<'a>, latch_pin: Output<'a>, clock_pin: Output<'a>) -> Self {
        let mut shift_reg = Self {
            data_pin,
            latch_pin,
            clock_pin,
            led_state: 0,
        };

        // Initialize pins to known state
        shift_reg.data_pin.set_low();
        shift_reg.latch_pin.set_low();
        shift_reg.clock_pin.set_low();

        // Clear all LEDs on startup
        shift_reg.clear();
        shift_reg.update();

        shift_reg
    }

    /// Set a single LED on or off
    /// led_index: 0-7 for the 8 LEDs (0=Q0/leftmost, 7=Q7/rightmost)
    /// state: true = on, false = off
    pub fn set_led(&mut self, led_index: u8, state: bool) {
        if led_index < NUM_SHIFT_LEDS as u8 {
            if state {
                // Set the bit: OR with a mask that has 1 in the target position
                // e.g., led_index=2 creates mask 0b00000100, OR sets that bit to 1
                self.led_state |= 1 << led_index;
            } else {
                // Clear the bit: AND with inverted mask (all 1s except target position)
                // e.g., led_index=2 creates mask 0b11111011, AND clears that bit to 0
                self.led_state &= !(1 << led_index);
            }
        }
    }

    /// Set all LEDs based on a bit pattern
    /// pattern: 8-bit value where each bit represents an LED (bit 0 = LED 0, etc.)
    pub fn set_pattern(&mut self, pattern: u8) {
        self.led_state = pattern;
    }

    /// Get the current LED state pattern
    pub fn get_pattern(&self) -> u8 {
        self.led_state
    }

    /// Clear all LEDs (set all bits to 0)
    pub fn clear(&mut self) {
        self.led_state = 0; // 0b00000000 - all LEDs off
    }

    /// Set all LEDs on (set all bits to 1)
    pub fn set_all(&mut self) {
        self.led_state = 0xFF; // 0b11111111 - all 8 LEDs on
    }

    /// Toggle a specific LED
    pub fn toggle_led(&mut self, led_index: u8) {
        if led_index < NUM_SHIFT_LEDS as u8 {
            self.led_state ^= 1 << led_index;
        }
    }

    /// Shift the pattern left (rotate)
    pub fn shift_left(&mut self, wrap: bool) {
        if wrap {
            let msb = (self.led_state & 0x80) >> 7;
            self.led_state = (self.led_state << 1) | msb;
        } else {
            self.led_state <<= 1;
        }
    }

    /// Shift the pattern right (rotate)
    pub fn shift_right(&mut self, wrap: bool) {
        if wrap {
            let lsb = (self.led_state & 0x01) << 7;
            self.led_state = (self.led_state >> 1) | lsb;
        } else {
            self.led_state >>= 1;
        }
    }

    /// Update the physical LEDs with the current state
    /// This sends the 8-bit pattern serially to the 74HC595 shift register
    pub fn update(&mut self) {
        // Start with latch (ST_CP) low to prevent premature output updates
        self.latch_pin.set_low();

        // Send 8 bits serially, MSB first (bit 7 down to bit 0)
        // MSB first means LED 7 data goes in first, but ends up controlling Q7
        for i in (0..8).rev() {
            // i = 7, 6, 5, 4, 3, 2, 1, 0

            // Extract the bit at position i from led_state
            // e.g., if led_state = 0b10110001 and i = 5:
            // (0b10110001 >> 5) = 0b00000101, then & 1 = 1
            let bit = (self.led_state >> i) & 1;

            // Set DS (data serial) pin based on the bit value
            if bit == 1 {
                self.data_pin.set_high(); // DS = 1 (LED will be ON)
            } else {
                self.data_pin.set_low(); // DS = 0 (LED will be OFF)
            }

            // Pulse SH_CP (shift register clock) to shift this bit into the register
            self.clock_pin.set_high();
            cortex_m::asm::delay(10); // 10 CPU cycles delay for setup time (~100ns at 100MHz)
            self.clock_pin.set_low();
            cortex_m::asm::delay(10); // 10 CPU cycles delay for hold time
        }

        // Pulse ST_CP (storage register clock/latch) to transfer data to output pins
        // This is when the LEDs actually change state
        self.latch_pin.set_high();
        cortex_m::asm::delay(10); // Brief pulse width
        self.latch_pin.set_low(); // Return to low state
    }

    /// Update with automatic refresh
    pub async fn update_async(&mut self) {
        self.update();
        // Small delay to allow LEDs to stabilize
        Timer::after(Duration::from_micros(100)).await;
    }
}

/// LED Bar pattern functions

/// VU Meter pattern - shows level from 0-8 LEDs lit
/// This demonstrates how bit patterns control which LEDs are on/off
pub async fn vu_meter_pattern(shift_reg: &mut ShiftRegister<'_>) -> ! {
    let mut level = 0;

    info!("Starting 74HC595 VU Meter pattern");
    info!("This pattern fills LEDs from left to right like a volume meter");
    loop {
        // Calculate bit pattern for VU meter
        let pattern = if level == 0 {
            0x00 // 0b00000000 - all LEDs off
        } else {
            // Create a mask with 'level' number of 1s from the right
            // level=1: (1<<1)-1 = 2-1 = 1 = 0b00000001 (1 LED on: Q0)
            // level=2: (1<<2)-1 = 4-1 = 3 = 0b00000011 (2 LEDs on: Q0,Q1)
            // level=3: (1<<3)-1 = 8-1 = 7 = 0b00000111 (3 LEDs on: Q0,Q1,Q2)
            // level=8: (1<<8)-1 = 256-1 = 255 = 0b11111111 (8 LEDs on: Q0-Q7)
            let max_bit = 1 << (level - 1);
            max_bit + (max_bit - 1)
        };
        let mut adc_msg: String<128> = String::new();
        let _ = core::fmt::write(
            &mut adc_msg,
            format_args!(
                "VU Level: {} LEDs (pattern: 0x{:02X} = 0b{:08b})",
                level, pattern, pattern
            ),
        );
        send_formatted_message(adc_msg).await;
        shift_reg.set_pattern(pattern);
        shift_reg.update_async().await;

        level = (level + 1) % 9; // Cycle through levels 0-8 (0,1,2,3,4,5,6,7,8,0,1,...)
        Timer::after(Duration::from_millis(800)).await; // Slower for better observation
    }
}

/// Simple demo function that shows the VU meter pattern
/// This demonstrates the basic operation of the 74HC595 shift register
pub async fn run_shift_register_demo(mut shift_reg: ShiftRegister<'_>) -> ! {
    info!("Starting 74HC595 Shift Register LED Bar Demo");
    info!("Demonstrating VU Meter pattern to show how bit patterns control LEDs");

    // Run the VU meter pattern indefinitely
    // This will show levels 0-8, demonstrating how each bit controls an LED
    vu_meter_pattern(&mut shift_reg).await
}

/// Task wrapper for the shift register demo
#[embassy_executor::task]
pub async fn shift_register_task(
    data_pin: Peri<'static, embassy_rp::peripherals::PIN_18>,
    latch_pin: Peri<'static, embassy_rp::peripherals::PIN_20>,
    clock_pin: Peri<'static, embassy_rp::peripherals::PIN_21>,
) -> ! {
    let data_pin = Output::new(data_pin, Level::Low);
    let latch_pin = Output::new(latch_pin, Level::Low);
    let clock_pin = Output::new(clock_pin, Level::Low);

    let shift_reg = ShiftRegister::new(data_pin, latch_pin, clock_pin);

    run_shift_register_demo(shift_reg).await
}

/// Simple usage example - shows how to manually control the shift register
/// This demonstrates individual LED control and direct bit pattern setting
pub async fn simple_usage_example(shift_reg: &mut ShiftRegister<'_>) {
    info!("74HC595 Simple Usage Example");
    info!("Demonstrating individual LED control and bit patterns");

    // Turn on all LEDs one by one from left to right (Q0 to Q7)
    info!("Turning on LEDs sequentially...");
    for i in 0..NUM_SHIFT_LEDS {
        shift_reg.set_led(i as u8, true);
        shift_reg.update_async().await;
        info!("LED {} ON - Pattern: 0b{:08b}", i, shift_reg.get_pattern());
        Timer::after(Duration::from_millis(300)).await;
    }

    Timer::after(Duration::from_millis(500)).await;

    // Turn off all LEDs one by one from right to left (Q7 to Q0)
    info!("Turning off LEDs in reverse...");
    for i in (0..NUM_SHIFT_LEDS).rev() {
        shift_reg.set_led(i as u8, false);
        shift_reg.update_async().await;
        info!("LED {} OFF - Pattern: 0b{:08b}", i, shift_reg.get_pattern());
        Timer::after(Duration::from_millis(300)).await;
    }

    Timer::after(Duration::from_millis(500)).await;

    // Set specific pattern (0b10101010 = 0xAA - alternating LEDs: Q1,Q3,Q5,Q7 on)
    info!("Setting alternating pattern 1...");
    shift_reg.set_pattern(0b10101010); // 0xAA
    shift_reg.update_async().await;
    info!("Pattern: 0x{:02X} = 0b{:08b}", 0xAA, 0xAA);
    Timer::after(Duration::from_millis(1000)).await;

    // Set different pattern (0b01010101 = 0x55 - other alternating LEDs: Q0,Q2,Q4,Q6 on)
    info!("Setting alternating pattern 2...");
    shift_reg.set_pattern(0b01010101); // 0x55
    shift_reg.update_async().await;
    info!("Pattern: 0x{:02X} = 0b{:08b}", 0x55, 0x55);
    Timer::after(Duration::from_millis(1000)).await;

    // Clear all LEDs
    info!("Clearing all LEDs...");
    shift_reg.clear(); // Sets pattern to 0x00
    shift_reg.update_async().await;
    info!("Pattern: 0x{:02X} = 0b{:08b}", 0x00, 0x00);
}
