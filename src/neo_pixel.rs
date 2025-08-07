use defmt::*;
use embassy_rp::peripherals::PIO1;
use embassy_rp::pio_programs::ws2812::PioWs2812;
use embassy_time::{Duration, Timer};
use smart_leds::RGB8;

pub const NUM_LEDS: usize = 8;

/// Task to control the NeoPixel strip
#[embassy_executor::task]
async fn neopixel_task(strip: NeoPixelStrip<'static>) -> ! {
    // Run the demo patterns (this function runs forever)
    run_neopixel_demo(strip).await;

    // This line will never be reached, but Rust needs it for type checking
    core::unreachable!()
}

/// Color constants for easy use
pub struct Color;

impl Color {
    pub const fn rgb(r: u8, g: u8, b: u8) -> RGB8 {
        RGB8 { r, g, b }
    }

    pub const RED: RGB8 = RGB8 { r: 255, g: 0, b: 0 };
    pub const GREEN: RGB8 = RGB8 { r: 0, g: 255, b: 0 };
    pub const BLUE: RGB8 = RGB8 { r: 0, g: 0, b: 255 };
    pub const WHITE: RGB8 = RGB8 {
        r: 255,
        g: 255,
        b: 255,
    };
    pub const YELLOW: RGB8 = RGB8 {
        r: 255,
        g: 255,
        b: 0,
    };
    pub const CYAN: RGB8 = RGB8 {
        r: 0,
        g: 255,
        b: 255,
    };
    pub const MAGENTA: RGB8 = RGB8 {
        r: 255,
        g: 0,
        b: 255,
    };
    pub const OFF: RGB8 = RGB8 { r: 0, g: 0, b: 0 };
}

/// NeoPixel strip wrapper for easier management
pub struct NeoPixelStrip<'a> {
    ws2812: PioWs2812<'a, PIO1, 0, NUM_LEDS>,
    buffer: [RGB8; NUM_LEDS],
    brightness: u8,
}

impl<'a> NeoPixelStrip<'a> {
    pub fn new(ws2812: PioWs2812<'a, PIO1, 0, NUM_LEDS>) -> Self {
        Self {
            ws2812,
            buffer: [RGB8::default(); NUM_LEDS],
            brightness: 32, // Default to 12.5% brightness
        }
    }

    /// Set a single LED color
    pub fn set_led(&mut self, index: usize, color: RGB8) {
        if index < NUM_LEDS {
            self.buffer[index] = color;
        }
    }

    /// Set all LEDs to the same color
    pub fn set_all(&mut self, color: RGB8) {
        for i in 0..NUM_LEDS {
            self.buffer[i] = color;
        }
    }

    /// Clear all LEDs (turn them off)
    pub fn clear(&mut self) {
        self.set_all(Color::OFF);
    }

    /// Write the current buffer to the LED strip
    pub async fn show(&mut self) {
        // Create dimmed buffer using current brightness setting
        let mut dimmed_buffer = [RGB8::default(); NUM_LEDS];
        for i in 0..NUM_LEDS {
            dimmed_buffer[i] = Self::dim_color(self.buffer[i], self.brightness);
        }

        self.ws2812.write(&dimmed_buffer).await;
    }

    /// Set global brightness (0-255)
    pub fn set_brightness(&mut self, brightness: u8) {
        self.brightness = brightness;
    }

    /// Get current brightness
    pub fn get_brightness(&self) -> u8 {
        self.brightness
    }

    /// Get the current color of an LED
    pub fn get_led(&self, index: usize) -> RGB8 {
        if index < NUM_LEDS {
            self.buffer[index]
        } else {
            Color::OFF
        }
    }

    /// Scale brightness of a color (0-255)
    pub fn dim_color(color: RGB8, brightness: u8) -> RGB8 {
        RGB8 {
            r: ((color.r as u16 * brightness as u16) / 255) as u8,
            g: ((color.g as u16 * brightness as u16) / 255) as u8,
            b: ((color.b as u16 * brightness as u16) / 255) as u8,
        }
    }

    /// Convert HSV to RGB
    pub fn hsv_to_rgb(h: u8, s: u8, v: u8) -> RGB8 {
        if s == 0 {
            return RGB8 { r: v, g: v, b: v };
        }

        let region = h / 43;
        let remainder = (h - (region * 43)) * 6;

        let p = (v as u16 * (255 - s as u16)) >> 8;
        let q = (v as u16 * (255 - ((s as u16 * remainder as u16) >> 8))) >> 8;
        let t = (v as u16 * (255 - ((s as u16 * (255 - remainder as u16)) >> 8))) >> 8;

        match region {
            0 => RGB8 {
                r: v,
                g: t as u8,
                b: p as u8,
            },
            1 => RGB8 {
                r: q as u8,
                g: v,
                b: p as u8,
            },
            2 => RGB8 {
                r: p as u8,
                g: v,
                b: t as u8,
            },
            3 => RGB8 {
                r: p as u8,
                g: q as u8,
                b: v,
            },
            4 => RGB8 {
                r: t as u8,
                g: p as u8,
                b: v,
            },
            _ => RGB8 {
                r: v,
                g: p as u8,
                b: q as u8,
            },
        }
    }

    /// Rainbow pattern
    pub fn rainbow(&mut self, offset: u8) {
        for i in 0..NUM_LEDS {
            let hue = ((i * 256 / NUM_LEDS) as u8).wrapping_add(offset);
            let color = Self::hsv_to_rgb(hue, 255, 255);
            self.set_led(i, color);
        }
    }

    /// Knight Rider scanner pattern
    pub fn knight_rider(&mut self, position: u8, color: RGB8) {
        self.clear();
        let pos = (position / 4) as usize % (NUM_LEDS * 2 - 2);

        let led_pos = if pos < NUM_LEDS {
            pos
        } else {
            (NUM_LEDS * 2 - 2) - pos
        };

        if led_pos < NUM_LEDS {
            self.set_led(led_pos, color);
            // Add trailing fade effect
            if led_pos > 0 {
                self.set_led(led_pos - 1, Self::dim_color(color, 100));
            }
            if led_pos + 1 < NUM_LEDS {
                self.set_led(led_pos + 1, Self::dim_color(color, 100));
            }
        }
    }

    /// VU meter pattern
    pub fn vu_meter(&mut self, level: u8, color: RGB8) {
        self.clear();
        let num_lit = ((level as usize * NUM_LEDS) / 8).min(NUM_LEDS);

        for i in 0..num_lit {
            let brightness = if i == num_lit - 1 { 255 } else { 180 };
            self.set_led(i, Self::dim_color(color, brightness));
        }
    }
}

/// Interactive NeoPixel control with button pattern switching
pub async fn run_neopixel_interactive(
    mut strip: NeoPixelStrip<'_>,
    btn: embassy_rp::gpio::Input<'_>,
) -> ! {
    info!("Starting interactive NeoPixel control");

    let mut btn = crate::debouncer::Debouncer::new(btn, Duration::from_millis(20));
    let mut pattern = 0u8;
    let mut counter = 0u8;

    loop {
        // Check for button press to change pattern
        if let Ok(_) = embassy_time::with_timeout(Duration::from_millis(50), btn.debounce()).await {
            if btn.debounce().await == embassy_rp::gpio::Level::Low {
                pattern = (pattern + 1) % 5;
                info!("Switching to pattern {}", pattern);
                counter = 0; // Reset counter for new pattern
                             // Wait for button release
                btn.debounce().await;
            }
        }

        match pattern {
            0 => {
                // Rainbow cycle
                strip.rainbow(counter);
                strip.show().await;
                counter = counter.wrapping_add(2);
            }
            1 => {
                // Knight Rider
                strip.knight_rider(counter, Color::RED);
                strip.show().await;
                counter = counter.wrapping_add(1);
            }
            2 => {
                // VU Meter simulation
                let level = (counter / 16) % 9;
                strip.vu_meter(level, Color::GREEN);
                strip.show().await;
                counter = counter.wrapping_add(1);
            }
            3 => {
                // Color fade
                let color = NeoPixelStrip::hsv_to_rgb(counter, 255, 255);
                strip.set_all(color);
                strip.show().await;
                counter = counter.wrapping_add(1);
            }
            4 => {
                // Breathing white
                let brightness = if counter < 128 {
                    counter * 2
                } else {
                    (255 - counter) * 2
                };
                strip.set_all(NeoPixelStrip::dim_color(Color::WHITE, brightness));
                strip.show().await;
                counter = counter.wrapping_add(2);
            }
            _ => pattern = 0,
        }

        Timer::after(Duration::from_millis(50)).await;
    }
}

/// Demo pattern that cycles through different effects automatically
pub async fn run_neopixel_demo(mut strip: NeoPixelStrip<'_>) {
    info!("Starting NeoPixel demo patterns");
    let mut counter = 0u8;
    let mut pattern = 0u8;

    loop {
        match pattern {
            0 => {
                // Rainbow cycle
                strip.rainbow(counter);
                strip.show().await;
                counter = counter.wrapping_add(2);
                if counter == 0 {
                    pattern = 1;
                }
            }
            1 => {
                // Knight Rider
                strip.knight_rider(counter, Color::RED);
                strip.show().await;
                counter = counter.wrapping_add(1);
                if counter == 0 {
                    pattern = 2;
                }
            }
            2 => {
                // VU Meter simulation
                let level = (counter / 16) % 9;
                strip.vu_meter(level, Color::GREEN);
                strip.show().await;
                counter = counter.wrapping_add(1);
                if counter == 0 {
                    pattern = 3;
                }
            }
            3 => {
                // Color fade
                let color = NeoPixelStrip::hsv_to_rgb(counter, 255, 255);
                strip.set_all(color);
                strip.show().await;
                counter = counter.wrapping_add(1);
                if counter == 0 {
                    pattern = 4;
                }
            }
            4 => {
                // Breathing white
                let brightness = if counter < 128 {
                    counter * 2
                } else {
                    (255 - counter) * 2
                };
                strip.set_all(NeoPixelStrip::dim_color(Color::WHITE, brightness));
                strip.show().await;
                counter = counter.wrapping_add(2);
                if counter == 0 {
                    pattern = 0;
                }
            }
            _ => pattern = 0,
        }

        Timer::after(Duration::from_millis(50)).await;
    }
}

/// Simple test pattern for verification
pub async fn simple_test_pattern(mut strip: NeoPixelStrip<'_>) {
    info!("Running simple NeoPixel test");

    loop {
        // Test each LED individually
        for i in 0..NUM_LEDS {
            strip.clear();
            strip.set_led(i, Color::RED);
            strip.show().await;
            Timer::after(Duration::from_millis(200)).await;
        }

        // All red
        strip.set_all(Color::RED);
        strip.show().await;
        Timer::after(Duration::from_millis(500)).await;

        // All green
        strip.set_all(Color::GREEN);
        strip.show().await;
        Timer::after(Duration::from_millis(500)).await;

        // All blue
        strip.set_all(Color::BLUE);
        strip.show().await;
        Timer::after(Duration::from_millis(500)).await;

        // All off
        strip.clear();
        strip.show().await;
        Timer::after(Duration::from_millis(500)).await;
    }
}
