# dht-mmap-rust

## ...is a library to read DHT11/DHT22 sensors on raspberry pi.

This library enables simple reading of DHT11 and DHT22 temperature+humidity sensors using the GPIO pins on a
Raspberry PI.  
In order to achieve sufficiently fast memory access, this library directly accesses the memory registers for pin
control.  
I wrote this library because the other libraries I tried did not work on my PI. This library should work out of the box.

## Usage

The file `/dev/gpiomem` needs to be accessible, so the program either needs to be run as root or have group-based access
to the file.  
_If you want to run your program to run on a user other than root,
[this StackOverflow answer](https://raspberrypi.stackexchange.com/a/40106) has instructions.
This is already configured by default on raspbian._

### Code:

```rust
fn main() {
    // The sensor is a DHT11 connected on pin 23
    let mut dht = Dht::new(DhtType::Dht11, 23).unwrap();

    // Important: DHT sensor reads fail sometimes. In an actual program, if a read fails you should retry multiple times until
    // the read succeeds.
    // For more information, see documentation on `read()`
    let reading = dht.read().unwrap();

    println!(
        "Temperature {} °C, Humidity {}%RH",
        reading.temperature(),
        reading.humidity()
    );
}
```

## Tests

This repo contains two tests,  
one that assumes a DHT11 is connected on GPIO pin 2 and  
one that assumes a DHT22 is connected on GPIO pin 3.