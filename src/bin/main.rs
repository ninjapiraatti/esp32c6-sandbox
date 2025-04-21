#![no_std]
#![no_main]

use esp_hal::clock::CpuClock;
use esp_hal::main;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Output, OutputConfig},
    mcpwm::{operator::PwmPinConfig, PeripheralClockConfig, McPwm, timer::PwmWorkingMode},
    time::Rate,
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

    let mut test_pin = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    //let mut led_pin_a = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());

    // Motor stuff
    let motor_a_pin1 = Output::new(peripherals.GPIO5, Level::Low, OutputConfig::default());
    let motor_a_pin2 = Output::new(peripherals.GPIO6, Level::Low, OutputConfig::default());
    let mut stdby_pin = Output::new(peripherals.GPIO0, Level::Low, OutputConfig::default());

    let clock_cfg = PeripheralClockConfig::with_frequency(Rate::from_mhz(40)).unwrap();
    let motor_a_pwm_pin = peripherals.GPIO4;
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    let motor_a_pwm = mcpwm.operator0.with_pin_a(motor_a_pwm_pin, PwmPinConfig::UP_ACTIVE_HIGH);
    let mut motor_a = Motor::new(motor_a_pin1, motor_a_pin2, motor_a_pwm).unwrap();

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(99, PwmWorkingMode::Increase, Rate::from_khz(20))
        .unwrap();
    mcpwm.timer0.start(timer_clock_cfg);
    stdby_pin.set_high();

    // From the template
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
        test_pin.set_high();
        
        motor_a.drive(DriveCommand::Forward(100)).expect("could set drive speed");
        println!("Debug: {:?}", motor_a.current_drive_command());
        delay.delay_millis(2500);

        motor_a.drive(DriveCommand::Backward(100)).expect("could set drive speed");
        println!("Debug: {:?}", motor_a.current_drive_command());
        delay.delay_millis(2500);
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}
