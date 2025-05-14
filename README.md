# ESP32-C6 Sandbox

A Rust-based experimental sandbox for the ESP32-C6 microcontroller (ESP-HAL 1.0.0), currently featuring motor control, RGB LED, rotary encoding, and button input with interrupt handling.

## Features

- 🚀 **Motor Control**: TB6612FNG motor driver integration
- 💡 **Smart LED Support**: RGB LED control with HSV color manipulation
- 🔄 **Rotary Encoder**: Position tracking with clockwise/counterclockwise detection
- 🔘 **Interrupt-Driven Button Input**: Hardware interrupt handling for button presses

## Getting Started

### Prerequisites and hardware

- Rust installed
- The devkit, for example ESP32-C6-DevKitC-1 v1.2
- 

### Building

```bash
cargo build
```

For release builds with optimizations:
```bash
cargo build --release
```

### Flashing

```bash
cargo run
```

## Examples

You can run individual examples like so:
```bash
cargo run --example example
```

## How the main program works

It demonstrates several ESP32-C6 capabilities:

1. **Motor Control**: The TB6612FNG driver controls motor direction and speed through PWM
2. **LED Effects**: RGB LEDs display HSV color effects based on rotary encoder position
3. **Input Handling**: 
   - Rotary encoder tracks position changes
   - Button triggers interrupts for responsive motor control
4. **Interrupt System**: Uses ESP-HAL's interrupt handlers


## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.