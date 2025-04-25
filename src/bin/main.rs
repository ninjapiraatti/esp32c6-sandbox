#![no_std]
#![no_main]

use embedded_hal::pwm::SetDutyCycle;
use embedded_hal::digital::OutputPin;
use core::cell::RefCell;
use critical_section::Mutex;
use esp_hal::{clock::CpuClock};
use esp_hal::main;
use esp_hal::timer::timg::TimerGroup;
use esp_println::println;
use esp_hal::{
    delay::Delay,
    gpio::{Level, Input, InputConfig, Event, Output, Io, OutputConfig, Pull},
    mcpwm::{operator::PwmPinConfig, PeripheralClockConfig, McPwm, timer::PwmWorkingMode},
    time::Rate,
    rmt::Rmt,
    handler,
    ram,
};
use tb6612fng::{DriveCommand, Motor};
use esp_hal_smartled::{smartLedBuffer, SmartLedsAdapter};
use smart_leds::{
    brightness, gamma,
    hsv::{hsv2rgb, Hsv},
    SmartLedsWrite,
};
use rotary_encoder_hal::{Direction, Rotary};

static BUTTON: Mutex<RefCell<Option<Input>>> = Mutex::new(RefCell::new(None));

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
    //let peripherals = esp_hal::init(esp_hal::Config::default());
    let mut io = Io::new(peripherals.IO_MUX);
    
    // Interrupt stuff
    let config = InputConfig::default().with_pull(Pull::Up);
    let switch = peripherals.GPIO20;
    io.set_interrupt_handler(interrupt_handler);
    let mut button = Input::new(switch, config);
    critical_section::with(|cs| {
        button.listen(Event::FallingEdge);
        BUTTON.borrow_ref_mut(cs).replace(button)
    });

    // Rotary encoder
    let clk = Input::new(peripherals.GPIO22, config);
    let dt = Input::new(peripherals.GPIO21, config);
    let mut encoder = Rotary::new(clk, dt);

    //let mut test_pin = Output::new(peripherals.GPIO7, Level::Low, OutputConfig::default());
    //let mut led_pin_a = Output::new(peripherals.GPIO8, Level::Low, OutputConfig::default());
    
    // LED stuff
    let led_pin = peripherals.GPIO8;
    let led_freq = Rate::from_mhz(80);
    let rmt = Rmt::new(peripherals.RMT, led_freq).unwrap();
    let rmt_buffer = smartLedBuffer!(1);
    let mut led = SmartLedsAdapter::new(rmt.channel0, led_pin, rmt_buffer);
    let mut color = Hsv {
        hue: 0,
        sat: 255,
        val: 255,
    };
    let mut data;

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
    let mut pos: isize = 0;

    loop {
        color.hue = pos as u8;
        data = [hsv2rgb(color)];
        led.write(brightness(gamma(data.iter().cloned()), 7))
                .unwrap();
            
        println!("Pos: {:?}", pos);
        match encoder.update().unwrap() {
            Direction::Clockwise => {
                pos += 1;
            }
            Direction::CounterClockwise => {
                pos -= 1;
            }
            Direction::None => {}
        }
        if pos > 10 && pos < 40 {
            run_motor(&mut motor_a);
        }
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.0.0-beta.0/examples/src/bin
}

#[handler]
#[ram]
fn interrupt_handler() {
    println!("Button pressed");
    cfg_if::cfg_if! {
        if #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))] {
            esp_println::println!(
                "GPIO Interrupt with priority {}",
                esp_hal::xtensa_lx::interrupt::get_level()
            );
        } else {
            println!("GPIO Interrupt");
        }
    }

    if critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .is_interrupt_set()
    }) {
        println!("Button pressed");
    } else {
        println!("Button NOT pressed");
    }

    critical_section::with(|cs| {
        BUTTON
            .borrow_ref_mut(cs)
            .as_mut()
            .unwrap()
            .clear_interrupt()
    });
}

fn run_motor<T, U, V>(motor: &mut Motor<T, U, V>) 
where
    T: OutputPin,
    U: OutputPin,
    V: SetDutyCycle,
{
    let delay = Delay::new();
    motor.drive(DriveCommand::Forward(100)).expect("could set drive speed");
    println!("Debug: {:?}", motor.current_drive_command());
    delay.delay_millis(100);

    motor.drive(DriveCommand::Backward(100)).expect("could set drive speed");
    println!("Debug: {:?}", motor.current_drive_command());
    delay.delay_millis(100);
    motor.drive(DriveCommand::Stop).expect("could set drive speed");
}