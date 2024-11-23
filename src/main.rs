#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    time::Hertz,
    Config,
};
use embassy_time::{Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Hello World!");

    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        config.rcc.hsi = false;
        config.rcc.hse = Some(Hse {
            freq: Hertz(24_000_000),
            mode: HseMode::Bypass,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV12,
            mul: PllMul::MUL168,
            divp: Some(PllPDiv::DIV2), // 24mhz / 12 * 168 / 2 = 168Mhz.
            divq: Some(PllQDiv::DIV7), // 24mhz / 12 * 168 / 7 = 48Mhz.
            divr: None,
        });
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV4;
        config.rcc.apb2_pre = APBPrescaler::DIV2;
        config.rcc.ls.rtc = RtcClockSource::LSI;
        config.rcc.ls.lsi = true;
        config.rcc.ls.lse = None;
        config.rcc.sys = Sysclk::PLL1_P;
    }
    let p = embassy_stm32::init(config);

    let mut led = Output::new(p.PC5, Level::High, Speed::Low);

    loop {
        fib_test::<u32>(34, &mut led).await;
        fib_test::<u64>(34, &mut led).await;
        fib_test::<f32>(32, &mut led).await;
        fib_test::<f64>(32, &mut led).await;
    }
}

async fn fib_test<'d, T>(n: usize, led: &mut Output<'d>)
where
    T: core::ops::Add<T, Output = T> + From<u8>,
{
    // Light LED for one second
    led.set_low();
    Timer::after_secs(1).await;
    led.set_high();

    // Compute fibonacci numbers
    let before = Instant::now();
    let _x = fib::<T>(n);

    // Report results to the debug probe
    info!(
        "{} {} -> {} ms",
        core::any::type_name::<T>(),
        n,
        before.elapsed().as_millis()
    );
}

fn fib<T>(n: usize) -> T
where
    T: core::ops::Add<T, Output = T> + From<u8>,
{
    if n > 2 {
        fib::<T>(n - 1) + fib::<T>(n - 2)
    } else {
        1_u8.into()
    }
}
