use memmap::MmapMut;
use std::fs::{File, OpenOptions};
use std::os::unix::prelude::OpenOptionsExt;

/*

This file contains a helper struct to access gpio pins directly via memory mapped IO

*/

/// Base memory location in /dev/gpiomem - only used for calculating GPIO_BASE
const BASE: u64 = 0x20000000;

/// Base memory location of the Gpio memmap part if /dev/gpiomem
const GPIO_BASE: u64 = BASE + 0x200000;

/// Size of the GPIO memmapped region
const GPIO_LENGTH: usize = 4096;

/// Direct Gpio access using mmap. Unmaps the mmapped region on drop.
/// The functions for gpio access are "unsafe", because the pin number is used as a memory offset.
pub(crate) struct GpioMmapAccess {
    _file: File,
    _map_struct: MmapMut,
    map: *mut u32,
}

/// Any type of error that happens when attempting to first access GPIO via mmap
#[derive(Debug)]
pub enum GpioOpenError {
    /// In order to function, the file `/dev/gpiomem` needs to be opened. This error means that that did
    /// not work. The most likely reason is that the program is not run as root / has access to the file
    /// or that the program has been run on a device that is not a raspberry pi.
    OpenGpioFileFailed(std::io::Error),
    /// An Error happened in trying to map the already opened File to memory. See contained io::Error
    /// for more information.
    CreateMmapError(std::io::Error),
}

impl GpioMmapAccess {
    /// Create a new `GpioMmapAccess`. Opens the `/dev/gpiomem` linux file and mmaps the region responsible for gpio pins.
    pub fn new() -> Result<Self, GpioOpenError> {
        let gpiomem = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_SYNC)
            .open("/dev/gpiomem")
            .map_err(|err| GpioOpenError::OpenGpioFileFailed(err))?;

        let mut mmap_mut = unsafe {
            memmap::MmapOptions::new()
                .len(GPIO_LENGTH)
                .offset(GPIO_BASE)
                .map_mut(&gpiomem)
                .map_err(|err| GpioOpenError::CreateMmapError(err))?
        };

        let raw_pointer = mmap_mut.as_mut_ptr() as *mut u32;

        return Ok(Self {
            _file: gpiomem,
            _map_struct: mmap_mut,
            map: raw_pointer,
        });
    }
}

impl GpioMmapAccess {
    /// Set the given pin to INPUT mode
    /// Safety: the pin number has to be valid. Invalid pin numbers may result in undefined behavior.
    pub unsafe fn pi_mmio_set_input(&mut self, pin: usize) {
        let address = self.map.add(pin / 10);
        let old_val = address.read_volatile();
        let new_val = old_val & !(7 << ((pin % 10) * 3));

        address.write_volatile(new_val);
    }

    /// Set the given pin to OUTPUT mode
    /// Safety: the pin number has to be valid. Invalid pin numbers may result in undefined behavior.
    pub unsafe fn pi_mmio_set_output(&mut self, pin: usize) {
        self.pi_mmio_set_input(pin);

        let address = self.map.add(pin / 10);
        let old_val = address.read_volatile();
        let new_val = old_val | (1 << ((pin % 10) * 3));
        address.write_volatile(new_val);
    }

    /// Set the given pin to HIGH to output a voltage. Assumes that the pin is in OUTPUT mode.
    /// Safety: the pin number has to be valid. Invalid pin numbers may result in undefined behavior.
    pub unsafe fn pi_mmio_set_high(&mut self, pin: usize) {
        self.map.add(7).write_volatile(1 << pin);
    }

    /// Set the given pin to LOW to output a voltage. Assumes that the pin is in OUTPUT mode.
    /// Safety: the pin number has to be valid. Invalid pin numbers may result in undefined behavior.
    pub unsafe fn pi_mmio_set_low(&mut self, pin: usize) {
        self.map.add(10).write_volatile(1 << pin);
    }

    /// Read the value of the given pin
    /// Safety: the pin number has to be valid. Invalid pin numbers may result in undefined behavior.
    pub unsafe fn pi_mmio_input(&mut self, pin: usize) -> bool {
        (self.map.add(13).read_volatile() & (1 << pin)) != 0
    }
}
