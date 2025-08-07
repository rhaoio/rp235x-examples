use defmt::*;
use embassy_rp::adc::{Adc, Config as AdcConfig, InterruptHandler as AdcInterruptHandler};
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::{ADC, ADC_TEMP_SENSOR, PIN_26};
use embassy_rp::{bind_interrupts, Peri};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::channel::{Channel, Sender};
use embassy_time::{Duration, Timer};
use heapless::String;

use crate::constants::{adc, buffers};
use crate::usb_serial::{send_formatted_message, send_message, SERIAL_CHANNEL};

bind_interrupts!(pub struct AdcIrqs {
    ADC_IRQ_FIFO => AdcInterruptHandler;
});
// Channel for sending ADC readings to PWM controller
pub static ADC_CHANNEL: Channel<
    ThreadModeRawMutex,
    SensorReadings,
    { buffers::ADC_CHANNEL_DEPTH },
> = Channel::new();

// Channel for sending single ADC readings
pub static SINGLE_ADC_CHANNEL: Channel<
    ThreadModeRawMutex,
    AdcReading,
    { buffers::ADC_CHANNEL_DEPTH },
> = Channel::new();

/// Helper functions to get channel senders
pub fn get_adc_sender() -> Sender<'static, ThreadModeRawMutex, SensorReadings, 8> {
    ADC_CHANNEL.sender()
}

pub fn get_single_adc_sender() -> Sender<'static, ThreadModeRawMutex, AdcReading, 8> {
    SINGLE_ADC_CHANNEL.sender()
}

/// Structure containing ADC reading data
#[derive(Clone, Copy, Debug)]
pub struct AdcReading {
    pub raw_value: u16,
    pub voltage_mv: u16,
    pub pin_number: u8,
}

/// Structure containing temperature sensor reading data
#[derive(Clone, Copy, Debug)]
pub struct TempReading {
    pub raw_value: u16,
    pub temp_celsius_tenths: u16, // Temperature in tenths of degrees (e.g., 235 = 23.5°C)
}

/// Combined ADC and temperature data
#[derive(Clone, Copy, Debug)]
pub struct SensorReadings {
    pub adc: AdcReading,
    pub temperature: TempReading,
}

/// Generic ADC reader function that accepts a callback function for processing readings
/// This is not a task - you need to spawn it as a task yourself
pub async fn generic_adc_reader<F>(
    adc_peripheral: Peri<'static, ADC>,
    pin_26: Peri<'static, PIN_26>,
    temp_sensor: Peri<'static, ADC_TEMP_SENSOR>,
    mut callback: F,
    interval_ms: u64,
) -> !
where
    F: FnMut(SensorReadings) + Send + 'static,
{
    info!("Starting generic ADC reader");

    // Initialize ADC with proper configuration
    let mut adc_config = AdcConfig::default();
    let mut adc = Adc::new(adc_peripheral, AdcIrqs, adc_config);

    // Configure ADC pin and temperature sensor
    let mut pin_26 = embassy_rp::adc::Channel::new_pin(pin_26, Pull::None);
    let mut temp_channel = embassy_rp::adc::Channel::new_temp_sensor(temp_sensor);

    loop {
        // Read ADC value from pin
        let raw_adc = adc.read(&mut pin_26).await.unwrap_or(0);
        let voltage_mv = adc_to_voltage(raw_adc);

        // Read temperature sensor
        let temp_raw = adc.read(&mut temp_channel).await.unwrap_or(0);
        let temp_c = adc_to_temperature_celsius(temp_raw);

        // Create reading structures
        let adc_reading = AdcReading {
            raw_value: raw_adc,
            voltage_mv,
            pin_number: 26,
        };

        let temp_reading = TempReading {
            raw_value: temp_raw,
            temp_celsius_tenths: temp_c,
        };

        let readings = SensorReadings {
            adc: adc_reading,
            temperature: temp_reading,
        };

        // Call the user-provided callback with the readings
        callback(readings);

        // Wait for the specified interval
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

/// Single ADC pin reader that accepts a callback (no temperature sensor)
/// This is not a task - you need to spawn it as a task yourself
pub async fn single_adc_reader<F>(
    adc_peripheral: Peri<'static, ADC>,
    pin_26: Peri<'static, PIN_26>,
    mut callback: F,
    interval_ms: u64,
) -> !
where
    F: FnMut(AdcReading) + Send + 'static,
{
    info!("Starting single ADC reader");

    // Initialize ADC with proper configuration
    let mut adc_config = AdcConfig::default();
    let mut adc = Adc::new(adc_peripheral, AdcIrqs, adc_config);

    // Configure ADC pin
    let mut pin_26 = embassy_rp::adc::Channel::new_pin(pin_26, Pull::None);

    loop {
        // Read ADC value from pin
        let raw_adc = adc.read(&mut pin_26).await.unwrap_or(0);
        let voltage_mv = adc_to_voltage(raw_adc);

        // Create reading structure
        let reading = AdcReading {
            raw_value: raw_adc,
            voltage_mv,
            pin_number: 26,
        };

        // Call the user-provided callback with the reading
        callback(reading);

        // Wait for the specified interval
        Timer::after(Duration::from_millis(interval_ms)).await;
    }
}

/// ADC reader task that periodically reads ADC values and sends them to USB serial
#[embassy_executor::task]
pub async fn adc_reader_task(
    adc_peripheral: Peri<'static, ADC>,
    pin_26: Peri<'static, PIN_26>,
    temp_sensor: Peri<'static, ADC_TEMP_SENSOR>,
) -> ! {
    info!("Starting ADC reader task");

    // Send startup message
    send_message("ADC Reader with temp sensor started").await;

    // Initialize ADC with proper configuration
    let mut adc_config = AdcConfig::default();
    let mut adc = Adc::new(adc_peripheral, AdcIrqs, adc_config);

    // Configure ADC pin and temperature sensor
    let mut pin_26 = embassy_rp::adc::Channel::new_pin(pin_26, Pull::None);
    let mut temp_channel = embassy_rp::adc::Channel::new_temp_sensor(temp_sensor);

    loop {
        // Read ADC value from pin
        let raw_adc = adc.read(&mut pin_26).await.unwrap_or(0);
        let voltage_mv = adc_to_voltage(raw_adc);

        // Read temperature sensor
        let temp_raw = adc.read(&mut temp_channel).await.unwrap_or(0);
        let temp_c = adc_to_temperature_celsius(temp_raw);

        // Send ADC reading
        let mut adc_msg: String<128> = String::new();
        let _ = core::fmt::write(
            &mut adc_msg,
            format_args!(
                "[ADC] Pin: {}.{:03}V (raw: {}) Temp: {}.{}°C",
                voltage_mv / 1000,
                voltage_mv % 1000,
                raw_adc,
                temp_c / 10,
                temp_c % 10
            ),
        );
        send_formatted_message(adc_msg).await;

        // Send temperature reading
        let mut temp_msg: String<128> = String::new();
        let _ = core::fmt::write(
            &mut temp_msg,
            format_args!(
                "[TEMP] {}.{}°C (raw: {})",
                temp_c / 10,
                temp_c % 10,
                temp_raw
            ),
        );
        send_formatted_message(temp_msg).await;

        // Wait 2 seconds between readings
        Timer::after(Duration::from_millis(2000)).await;
    }
}

/// Convert ADC reading to millivolts
/// Pico 2 W: 12-bit ADC (0-4095), 3.3V reference
fn adc_to_voltage(adc_value: u16) -> u16 {
    // Use the constant function from the adc module
    adc::raw_to_millivolts(adc_value)
}

/// Convert ADC reading from temperature sensor to degrees Celsius (x10)
/// Returns temperature in tenths of degrees (e.g., 235 = 23.5°C)
fn adc_to_temperature_celsius(adc_value: u16) -> u16 {
    // Pico temperature sensor: 27°C = ~0.706V, slope = -1.721mV/°C
    // Convert ADC to voltage first
    let voltage_mv = adc_to_voltage(adc_value);

    // Temperature formula: T = 27 - (V - 0.706) / 0.001721
    // Rearranged: T = 27 - (V_mv - 706) / 1.721
    // Return in tenths: T*10 = 270 - (V_mv - 706) * 10 / 1.721

    if voltage_mv >= 706 {
        let temp_tenths = 270 - ((voltage_mv - 706) as u32 * 10 / 1721);
        temp_tenths.min(999) as u16 // Cap at 99.9°C
    } else {
        let temp_tenths = 270 + ((706 - voltage_mv) as u32 * 10 / 1721);
        temp_tenths.min(999) as u16 // Cap at 99.9°C
    }
}

/// Convert millivolts to volts with 3 decimal places
fn millivolts_to_volts_string(millivolts: u16) -> (u16, u16) {
    let volts = millivolts / 1000;
    let remaining_mv = millivolts % 1000;
    (volts, remaining_mv)
}

/// Simple function to send custom ADC messages from anywhere
pub async fn log_adc_event(message: &str) {
    let mut formatted: String<128> = String::new();
    let _ = core::fmt::write(&mut formatted, format_args!("[ADC_EVENT] {}", message));
    send_formatted_message(formatted).await;
}

/// Example of how to send meter readings with custom formatting
pub async fn send_meter_reading(sensor_name: &str, value_mv: u16, unit: &str) {
    let (volts, mv_remainder) = millivolts_to_volts_string(value_mv);
    let mut formatted: String<128> = String::new();
    let _ = core::fmt::write(
        &mut formatted,
        format_args!(
            "[METER] {}: {}.{:03} {}",
            sensor_name, volts, mv_remainder, unit
        ),
    );
    send_formatted_message(formatted).await;
}

/// Debug helper for sending debug info with accurate voltage display
pub async fn debug_adc(message: &str, channel: u8, raw_value: u16) {
    let millivolts = adc_to_voltage(raw_value);
    let (volts, mv_remainder) = millivolts_to_volts_string(millivolts);

    let mut formatted: String<128> = String::new();
    let _ = core::fmt::write(
        &mut formatted,
        format_args!(
            "[DEBUG] {}: CH{} = {}.{:03}V (raw: {})",
            message, channel, volts, mv_remainder, raw_value
        ),
    );
    send_formatted_message(formatted).await;
}

// ===== EXAMPLE USAGE AND WRAPPER TASKS =====

/// Example: Wrapper task that logs ADC readings to USB serial
#[embassy_executor::task]
pub async fn adc_logger_task(
    adc_peripheral: Peri<'static, ADC>,
    pin_26: Peri<'static, PIN_26>,
    temp_sensor: Peri<'static, ADC_TEMP_SENSOR>,
) -> ! {
    generic_adc_reader(
        adc_peripheral,
        pin_26,
        temp_sensor,
        |readings| {
            // This closure will be called for each reading
            // We can't use async functions in closures easily, so we use defmt for logging
            info!(
                "[ADC] Pin: {}.{:03}V (raw: {}) Temp: {}.{}°C",
                readings.adc.voltage_mv / 1000,
                readings.adc.voltage_mv % 1000,
                readings.adc.raw_value,
                readings.temperature.temp_celsius_tenths / 10,
                readings.temperature.temp_celsius_tenths % 10
            );
        },
        2000, // 2 second interval
    )
    .await;
}

/// Example: How to use the callback to control PWM based on voltage
/// Note: You would need to pass in PWM peripheral and configure it
/// This shows the pattern - you'd adapt it to your specific PWM setup
pub async fn voltage_controlled_pwm_example() {
    // Example of how you might set up a callback that controls PWM
    // You would need to move PWM into the callback or use a static/shared reference

    let pwm_callback = |readings: SensorReadings| {
        // Map voltage (0-3300mV) to PWM duty cycle (0-100%)
        let duty_percent = (readings.adc.voltage_mv as u32 * 100) / 3300;

        // Clamp to 0-100%
        let duty_percent = duty_percent.min(100) as u8;

        // Set PWM duty cycle (you'd implement this based on your PWM setup)
        // pwm.set_duty_cycle_percent(duty_percent);

        info!(
            "Voltage: {}mV -> PWM: {}%",
            readings.adc.voltage_mv, duty_percent
        );
    };

    // You would spawn this as a task:
    // spawner.spawn(generic_adc_pwm_task(adc, pin_26, temp_sensor)).unwrap();
}

// ===== CHANNEL-BASED ADC READERS =====

/// ADC reader task that sends full sensor data (ADC + temp) to PWM controller via channel
#[embassy_executor::task]
pub async fn adc_to_channel_task(
    adc_peripheral: Peri<'static, ADC>,
    pin_26: Peri<'static, PIN_26>,
    temp_sensor: Peri<'static, ADC_TEMP_SENSOR>,
) -> ! {
    info!("Starting ADC to channel task");
    let sender = get_adc_sender();
    //let adc_channel = SERIAL_CHANNEL.sender();
    // Initialize ADC with proper configuration
    let mut adc_config = AdcConfig::default();
    let mut adc = Adc::new(adc_peripheral, AdcIrqs, adc_config);

    // Configure ADC pin and temperature sensor
    let mut pin_26 = embassy_rp::adc::Channel::new_pin(pin_26, Pull::None);
    let mut temp_channel = embassy_rp::adc::Channel::new_temp_sensor(temp_sensor);

    loop {
        // Read ADC value from pin
        let raw_adc = adc.read(&mut pin_26).await.unwrap_or(0);
        let voltage_mv = adc_to_voltage(raw_adc);

        // Read temperature sensor
        let temp_raw = adc.read(&mut temp_channel).await.unwrap_or(0);
        let temp_c = adc_to_temperature_celsius(temp_raw);

        // Create reading structures
        let adc_reading = AdcReading {
            raw_value: raw_adc,
            voltage_mv,
            pin_number: 26,
        };

        let temp_reading = TempReading {
            raw_value: temp_raw,
            temp_celsius_tenths: temp_c,
        };

        let readings = SensorReadings {
            adc: adc_reading,
            temperature: temp_reading,
        };

        // Send to channel (non-blocking, will drop if channel is full)
        match sender.try_send(readings) {
            Ok(()) => {
                // Successfully sent
            }
            Err(_) => {
                info!("Channel full, dropping ADC reading");
            }
        }

        // Read every 100ms for responsive control
        Timer::after(Duration::from_millis(100)).await;
    }
}

/// Single ADC reader task that sends only ADC data (no temp) to channel
#[embassy_executor::task]
pub async fn single_adc_to_channel_task(
    adc_peripheral: Peri<'static, ADC>,
    pin_26: Peri<'static, PIN_26>,
) -> ! {
    info!("Starting single ADC to channel task");
    let sender = get_single_adc_sender();
    let adc_channel = SERIAL_CHANNEL.sender();

    // Initialize ADC with proper configuration
    let mut adc_config = AdcConfig::default();
    let mut adc = Adc::new(adc_peripheral, AdcIrqs, adc_config);

    // Configure ADC pin
    let mut pin_26 = embassy_rp::adc::Channel::new_pin(pin_26, Pull::None);

    loop {
        // Read ADC value from pin
        let raw_adc = adc.read(&mut pin_26).await.unwrap_or(0);
        let voltage_mv = adc_to_voltage(raw_adc);

        // Create reading structure
        let reading = AdcReading {
            raw_value: raw_adc,
            voltage_mv,
            pin_number: 26,
        };

        // Send to channel (non-blocking)
        match sender.try_send(reading) {
            Ok(()) => {
                // Successfully sent
            }
            Err(_) => {
                info!("Single ADC channel full, dropping reading");
            }
        }

        match adc_channel.try_send(crate::usb_serial::SerialMessage::AdcReading {
            channel: 26,
            value: reading.voltage_mv as f32,
        }) {
            Ok(()) => {
                // Successfully sent
            }
            Err(_) => {
                info!("Single ADC channel full, dropping reading");
            }
        }
        // Read every 50ms for very responsive control
        Timer::after(Duration::from_millis(50)).await;
    }
}
