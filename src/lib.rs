pub mod mmap_gpio;
mod scoped_thread_priority;

use crate::mmap_gpio::{GpioMmapAccess, GpioOpenError};
use crate::scoped_thread_priority::HighThreadPriorityGuard;
use std::thread::sleep;
use std::time::{Duration, SystemTime};

/*

This is a small library to read DHT11 and DHT22 sensors using memory mapped IO

It's mostly a port of the C library by Adafruit:
  https://github.com/adafruit/Adafruit_Python_DHT/blob/master/source/Raspberry_Pi/pi_dht_read.c

For exact specifications on how the DHT11 communicates, see this DHT11 datasheet by mouser:
  https://www.mouser.com/datasheet/2/758/DHT11-Technical-Data-Sheet-Translated-Version-1143054.pdf

For specifications on the GPIO memory addresses, see the BCM2711 ARM Peripherals datasheet (PI 4B)
  https://datasheets.raspberrypi.com/bcm2711/bcm2711-peripherals.pdf
The registers used are already specified in the PI 1 and PI zero, see the BCM2835 datasheet:
  https://www.raspberrypi.org/app/uploads/2012/02/BCM2835-ARM-Peripherals.pdf

GPIO pin numbers are in the range 0..53 for the PI 1, but were increased up to 0..57 for the PI 4.
Since the registers used were already reserved on the PI 1, this library allows writing to those higher
pin numbers, even though they don't do anything.
At this time, the PIs all only have 40 GPIO pins (0..39).

*/

/// Maximum "ticks" / volatile reads attempted before a read is considered a timeout. Might need
/// to be increased for faster devices.
const DHT_MAXCOUNT: u32 = 32000;

/// Number of "bits" transmitted by the DHT for a single reading
const DHT_PULSES: usize = 41;

/// The type of Dht used. Specifying the wrong type results in absurd values, but does not pose
/// any other risk, as only data interpretation is affected.
pub enum DhtType {
    Dht11,
    Dht22,
}

/// Handle on a Dht device, can be used to read sensor data.
/// The internal memmap reference is automatically cleared on drop.
pub struct Dht {
    gpio: GpioMmapAccess,
    pin: usize,
    typ: DhtType,
}

/// Reason that reading from the Dht failed. See individual errors for explanations.
/// All of these can generally be retried.
#[derive(Copy, Clone, PartialEq, Debug)]
pub enum DhtError {
    /// A timeout happened before data transmission started
    ReadTimeoutEarly,
    /// A timeout happened during data transmission
    ReadTimeoutData,
    /// The data received does not match the checksum - there likely was some transmission error
    ChecksumMismatch,
}

impl Dht {
    /// Create a new `Dht` struct, which can be used to read sensor data.
    /// Note: Specifying the wrong DhtType only results in absurd readings, but does not have any other risks.
    /// The `gpio_pin` is the GPIO number, not the pin number. You can find annotated visual charts for your PI's layout online.
    /// Make sure you have specified the correct pin number.
    /// Pin numbers from 0..57 are allowed here, but only pins 0..39 are actually available on current PI hardware.
    pub fn new(typ: DhtType, gpio_pin: usize) -> Result<Dht, GpioOpenError> {
        if gpio_pin > 57 {
            panic!(
                "Gpio pin number was too large: {}. Aborting to prevent memory overwrites.",
                gpio_pin
            );
        }

        return Ok(Dht {
            gpio: GpioMmapAccess::new()?,
            pin: gpio_pin,
            typ,
        });
    }

    /// Try to get a reading from the DHT sensor.  
    /// **Note** that reading from DHT sensors is inherently unreliable: expect one in three reads to fail, round about.
    /// Additionally, in rare cases individual reads may result in incorrect values.
    /// This is a hardware issue. If you need more reliable reads, read twice
    /// and make sure the results don't deviate much.  
    /// Lastly, if you need *accurate* readings, cross-check your readings to some sensor you know to be correct,
    /// it has been reported online that some DHT modules report values that are offset from the actual value by
    /// some amount.
    pub fn read(&mut self) -> Result<Reading, DhtError> {
        let pin = self.pin;
        let gpio = &mut self.gpio;

        // Number of "reads" is recorded into this array, meaning how many times the gpio
        // state could be polled before the pin read state switched.
        // Every even entry is the length of a "low", every odd entry is the length of a "high"
        let mut pulse_counts = [0u32; DHT_PULSES * 2];

        // Final data will be recorded into this array
        let mut data = [0u8; 5];

        unsafe {
            // Set high scheduling priority
            let _priority_guard = HighThreadPriorityGuard::new();

            // Pull high for 500ms
            gpio.pi_mmio_set_output(pin);
            gpio.pi_mmio_set_high(pin);
            sleep(Duration::from_millis(500));

            // Pull low for 20ms
            gpio.pi_mmio_set_low(pin);
            busy_wait_milliseconds(20);

            // Start reading response
            gpio.pi_mmio_set_input(pin);

            let mut temp: u32 = 0;

            // Spin for a bit. Write volatile so that the compiler does not optimize out.
            for i in 0..500 {
                ((&mut temp) as *mut u32).write_volatile(i);
            }

            // Spin until input is "low"
            let mut count = 0u32;
            while gpio.pi_mmio_input(pin) {
                count += 1;
                if count >= DHT_MAXCOUNT {
                    return Err(DhtError::ReadTimeoutEarly);
                }
            }

            // Record all pulses and their lengths
            for i in (0..(DHT_PULSES * 2)).step_by(2) {
                // Count how long pin is low and store in pulse_counts[i]
                while !gpio.pi_mmio_input(pin) {
                    pulse_counts[i] += 1;
                    if pulse_counts[i] >= DHT_MAXCOUNT {
                        // Timeout waiting for response.
                        return Err(DhtError::ReadTimeoutData);
                    }
                }
                // Count how long pin is high and store in pulse_counts[i+1]
                while gpio.pi_mmio_input(pin) {
                    pulse_counts[i + 1] += 1;
                    if pulse_counts[i + 1] >= DHT_MAXCOUNT {
                        // Timeout waiting for response.
                        return Err(DhtError::ReadTimeoutData);
                    }
                }
            }
        }

        // Compute the average low pulse width to use as a 50 microsecond reference threshold.
        // The "low" parts are expected to be 50 microseconds every time, while the "high" is
        // either 26-26 micros (bit is 0) or 70 micros (bit is 1). The low part timing can thus
        // be used as a threshold for timing.
        // Ignore the first two readings because they are a constant 80 microsecond pulse.
        let mut threshold = 0u32;
        for i_raw in 1..DHT_PULSES {
            let i = i_raw * 2;
            threshold += pulse_counts[i];
        }
        threshold = threshold / (DHT_PULSES as u32 - 1);

        // Interpret each high pulse as a 0 or 1 by comparing it to the 50us reference.
        // If the count is less than 50us it must be a ~28us 0 pulse. If it's higher,
        // then it must be a ~70us 1 pulse.
        for i in (3..(DHT_PULSES * 2)).step_by(2) {
            let mut index = (i - 3) / 16;
            data[index] <<= 1;
            if pulse_counts[i] >= threshold {
                // One bit for long pulse.
                data[index] |= 1;
            }
            // Else zero bit for short pulse.
        }

        // Verify checksum of received data.
        if data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF) {
            return Err(DhtError::ChecksumMismatch);
        }

        return match self.typ {
            DhtType::Dht11 => {
                // This deviates from the Adafruit C code: According to the spec (link at the top of this file),
                // the second and fourth byte are decimal parts for humidity and temperature respectively.
                Ok(Reading {
                    humidity: data[0] as f32 + data[1] as f32 * 0.1,
                    temperature: data[2] as f32 + data[3] as f32 * 0.1,
                })
            }
            DhtType::Dht22 => {
                let humidity = (data[0] as u32 * 256 + data[1] as u32) as f32 / 10.0f32;
                let mut temperature =
                    ((data[2] & 0x7F) as u32 * 256 + data[3] as u32) as f32 / 10.0f32;
                if data[2] & 0x80 != 0 {
                    temperature *= -1.0f32;
                }

                Ok(Reading {
                    humidity,
                    temperature,
                })
            }
        };
    }
}

/// Reading from the DHT sensor.
#[derive(Copy, Clone, PartialEq, Debug)]
pub struct Reading {
    temperature: f32,
    humidity: f32,
}

impl Reading {
    pub fn temperature(&self) -> f32 {
        return self.temperature;
    }

    pub fn humidity(&self) -> f32 {
        return self.humidity;
    }
}

/// Busy-wait `ms` milliseconds, don't yield to the system
fn busy_wait_milliseconds(ms: u64) {
    let before = SystemTime::now();

    loop {
        if before.elapsed().unwrap().as_millis() as u64 >= ms {
            return;
        }
    }
}
