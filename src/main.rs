#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    Config,
};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

type FibonacciOutput = u32;

pub fn fib(x: usize) -> FibonacciOutput {
    if x > 2 {
        fib(x - 1) + fib(x - 2)
    } else {
        1.0 as FibonacciOutput
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    let config = Config::default();
    let p = embassy_stm32::init(config);

    let mut led = Output::new(p.PC5, Level::High, Speed::Low);

    loop {
        // Light LED for one second
        led.set_low();
        Timer::after_secs(1).await;
        led.set_high();

        // Compute fibonacci numbers
        let _x = fib(32);
    }
}
