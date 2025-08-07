//! # Hardware Constants Module
//!
//! This module contains all commonly used constants throughout the project,
//! organized by functional area for better maintainability and documentation.

/// ADC (Analog-to-Digital Converter) Constants
pub mod adc {
    /// Maximum ADC value for 12-bit resolution (2^12 - 1)
    pub const MAX_VALUE: u16 = 4095;

    /// ADC center value (half of max range)
    pub const CENTER_VALUE: u16 = 2047;

    /// Reference voltage in millivolts (3.3V)
    pub const REFERENCE_VOLTAGE_MV: u16 = 3300;

    /// Convert ADC raw value to millivolts
    pub const fn raw_to_millivolts(raw: u16) -> u16 {
        ((raw as u32 * REFERENCE_VOLTAGE_MV as u32) / MAX_VALUE as u32) as u16
    }

    /// Convert millivolts to ADC raw value
    pub const fn millivolts_to_raw(mv: u16) -> u16 {
        ((mv as u32 * MAX_VALUE as u32) / REFERENCE_VOLTAGE_MV as u32) as u16
    }
}

/// System Clock Constants
pub mod system {
    /// System clock frequency in MHz
    pub const CLOCK_FREQ_MHZ: u32 = 125;

    /// System clock frequency in Hz
    pub const CLOCK_FREQ_HZ: u32 = 125_000_000;

    /// Common PWM clock divider to get 1MHz
    pub const PWM_CLOCK_DIVIDER: u32 = 125;

    /// Resulting frequency after dividing by PWM_CLOCK_DIVIDER (1MHz)
    pub const PWM_BASE_FREQ_HZ: u32 = CLOCK_FREQ_HZ / PWM_CLOCK_DIVIDER;
}

/// Servo Motor Constants
pub mod servo {
    /// Minimum servo pulse width in microseconds (0 degrees)
    pub const MIN_PULSE_WIDTH_US: u32 = 500;

    /// Maximum servo pulse width in microseconds (180 degrees)
    pub const MAX_PULSE_WIDTH_US: u32 = 2500;

    /// Standard servo PWM frequency in Hz
    pub const PWM_FREQUENCY_HZ: u32 = 50;

    /// Servo PWM period in microseconds (1/50Hz = 20ms)
    pub const PWM_PERIOD_US: u32 = 20000;

    /// PWM top value for 50Hz (1MHz / 50Hz - 1)
    pub const PWM_TOP_VALUE: u16 = 19999;

    /// Maximum servo angle in degrees
    pub const MAX_ANGLE_DEGREES: f32 = 180.0;

    /// Minimum servo angle in degrees
    pub const MIN_ANGLE_DEGREES: f32 = 0.0;
}

/// PWM (Pulse Width Modulation) Constants
pub mod pwm {
    /// Standard PWM top value for percentage-based control
    pub const STANDARD_TOP_VALUE: u16 = 1000;

    /// Maximum duty cycle percentage
    pub const MAX_DUTY_PERCENT: u8 = 100;

    /// Minimum duty cycle percentage
    pub const MIN_DUTY_PERCENT: u8 = 0;
}

/// Voltage and Power Constants
pub mod voltage {
    /// Standard 3.3V rail in millivolts
    pub const VCC_3V3_MV: u16 = 3300;

    /// Standard 5V rail in millivolts
    pub const VCC_5V_MV: u16 = 5000;

    /// Millivolts per volt conversion factor
    pub const MV_PER_VOLT: u16 = 1000;
}

/// Timing Constants (in milliseconds unless specified)
pub mod timing {
    /// Standard delay for visual feedback (500ms)
    pub const VISUAL_DELAY_MS: u64 = 500;

    /// Standard long delay (1 second)
    pub const LONG_DELAY_MS: u64 = 1000;

    /// Standard short delay (100ms)
    pub const SHORT_DELAY_MS: u64 = 100;

    /// Fast update rate for smooth control (20ms = 50Hz)
    pub const FAST_UPDATE_MS: u64 = 20;

    /// Medium update rate (50ms = 20Hz)
    pub const MEDIUM_UPDATE_MS: u64 = 50;

    /// USB serial heartbeat interval (10 seconds)
    pub const USB_HEARTBEAT_MS: u64 = 10000;
}

/// Joystick and Control Constants
pub mod joystick {
    /// Normalized joystick range maximum
    pub const NORMALIZED_MAX: i16 = 1000;

    /// Normalized joystick range minimum
    pub const NORMALIZED_MIN: i16 = -1000;

    /// Joystick dead zone around center (in ADC units)
    pub const DEAD_ZONE_ADC: u16 = 100;
}

/// Temperature Sensor Constants
pub mod temperature {
    /// Temperature sensor scaling factor (tenths of degrees)
    pub const SCALE_FACTOR_TENTHS: u16 = 10;
}

/// Communication Buffer Sizes
pub mod buffers {
    /// USB packet buffer size
    pub const USB_PACKET_SIZE: usize = 64;

    /// Line buffer size for command input
    pub const LINE_BUFFER_SIZE: usize = 128;

    /// Formatted message buffer size
    pub const MESSAGE_BUFFER_SIZE: usize = 128;

    /// Channel buffer depth for ADC readings
    pub const ADC_CHANNEL_DEPTH: usize = 8;

    /// Serial message channel depth
    pub const SERIAL_CHANNEL_DEPTH: usize = 10;
}

/// LED and Display Constants
pub mod display {
    /// Number of LEDs in NeoPixel strip
    pub const NEOPIXEL_COUNT: usize = 8;

    /// Number of LEDs in shift register
    pub const SHIFT_REGISTER_LEDS: usize = 8;

    /// Maximum LED brightness value
    pub const MAX_BRIGHTNESS: u8 = 255;

    /// Half brightness value
    pub const HALF_BRIGHTNESS: u8 = 128;

    /// LCD1602 display dimensions
    pub const LCD_ROWS: u8 = 2;
    pub const LCD_COLS: u8 = 16;
}

/// I2C Communication Constants
pub mod i2c {
    /// Standard I2C frequency for most devices (100kHz)
    pub const STANDARD_FREQUENCY_HZ: u32 = 100_000;

    /// Fast I2C frequency (400kHz)
    pub const FAST_FREQUENCY_HZ: u32 = 400_000;

    /// PCF8574 I2C address range start
    pub const PCF8574_ADDR_MIN: u8 = 0x20;

    /// PCF8574 I2C address range end
    pub const PCF8574_ADDR_MAX: u8 = 0x27;

    /// Common alternative PCF8574 address
    pub const PCF8574_ALT_ADDR: u8 = 0x3F;
}
