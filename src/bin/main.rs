#![no_std]
#![no_main]

use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::time::{Duration, Instant};
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_hal::{
    delay::Delay,
    gpio::{Io, Level, Output, OutputConfig},
    mcpwm::{operator::PwmPinConfig, timer::PwmWorkingMode, PeripheralClockConfig, McPwm},
};
use tb6612fng::{DriveCommand, Motor};

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

extern crate alloc;

#[main]
fn main() -> ! {
    // generator version: 0.3.1

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let mut led_pin_a = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    
    // Motor stuff
    let motorpin1 = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let motorpin2 = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let mut test_pin = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());

    let clock_cfg = PeripheralClockConfig::with_frequency(esp_hal::time::Rate::from_mhz(40)).unwrap();

    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    let pwm_pin = peripherals.GPIO4;

    let pwm_pin_m = mcpwm.operator0.with_pin_a(pwm_pin, PwmPinConfig::UP_ACTIVE_HIGH);

    let mut motor1 = Motor::new(motorpin1, motorpin2, pwm_pin_m).unwrap();

    esp_alloc::heap_allocator!(size: 72 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let _init = esp_wifi::init(
        timg0.timer0,
        esp_hal::rng::Rng::new(peripherals.RNG),
        peripherals.RADIO_CLK,
    )
    .unwrap();

    let delay = Delay::new();

    loop {
        delay.delay_millis(2500);
        led_pin_a.set_level(Level::High);
        println!("Debug: {:?}", led_pin_a.is_set_high());
        motor1.drive(DriveCommand::Forward(100)).expect("could set drive speed");
        delay.delay_millis(2500);
        led_pin_a.set_level(Level::Low);
        led_pin_a.toggle();
        println!("Debug: {:?}", led_pin_a.is_set_high());
        motor1.drive(DriveCommand::Forward(0)).expect("could set drive speed");
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}
