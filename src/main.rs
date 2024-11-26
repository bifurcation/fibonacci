#![no_std]
#![no_main]
#![allow(dead_code)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    mode::Async,
    spi::{self, Spi},
    time::Hertz,
    Config,
};
use embassy_time::{Instant, Timer};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Configure the device and get a handle to its capabilities
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

    // Configure the LED
    let mut led = Output::new(p.PC5, Level::High, Speed::Low);

    // Configure the screen
    let spi1 = {
        use embassy_stm32::spi::*;
        let mut config = Config::default();
        config.mode.polarity = Polarity::IdleLow;
        config.mode.phase = Phase::CaptureOnFirstTransition;
        config.bit_order = BitOrder::MsbFirst;
        config.frequency = Hertz(42_000_000);

        Spi::new_txonly(
            p.SPI1,     // SPI peripheral
            p.PA5,      // Clock pin
            p.PA7,      // MOSI pin
            p.DMA2_CH3, // DMA peripheral
            config,
        )
    };

    let mut screen = {
        let cs = Output::new(p.PC14, Level::Low, Speed::Low);
        let rst = Output::new(p.PB9, Level::Low, Speed::Low);
        let dc = Output::new(p.PC13, Level::Low, Speed::Low);
        let bl = Output::new(p.PB8, Level::Low, Speed::Low);
        Screen::new(spi1, cs, rst, dc, bl).await.unwrap()
    };

    screen.set_orientation(Orientation::Portrait).unwrap();

    // Do stuff
    const X: usize = 10;
    const Y: usize = 10;
    const WIDTH: usize = 10;
    const HEIGHT: usize = 10;
    let mut color = 0_u16;
    let mut pixel_buf = [0_u16; WIDTH * HEIGHT];

    screen.enable_backlight();
    loop {
        color = (color + 1) % 0x0fff;
        pixel_buf.fill(color);
        screen
            .update_area(X, Y, X + WIDTH, Y + HEIGHT, &pixel_buf)
            .await
            .unwrap();

        led.set_low();
        Timer::after_millis(500).await;

        led.set_high();
        Timer::after_millis(500).await;

        /*
        led.set_low();
        screen.disable_backlight();
        Timer::after_secs(1).await;

        led.set_high();
        screen.enable_backlight();
        Timer::after_secs(1).await;
        */

        /*
        fib_test::<u32>(34, &mut led).await;
        fib_test::<u64>(34, &mut led).await;
        fib_test::<f32>(32, &mut led).await;
        fib_test::<f64>(32, &mut led).await;
        */
    }
}

struct Screen<'d> {
    spi: Spi<'d, Async>,
    cs: Output<'d>,  // Chip select
    rst: Output<'d>, // Reset
    dc: Output<'d>,  // Data / command
    bl: Output<'d>,  // Backlight

    view_width: usize,
    view_height: usize,
}

impl<'d> Screen<'d> {
    async fn new(
        spi: Spi<'d, Async>,
        cs: Output<'d>,
        rst: Output<'d>,
        dc: Output<'d>,
        bl: Output<'d>,
    ) -> Result<Self, spi::Error> {
        const STARTUP_SEQUENCE: &[(u8, &[u8])] = &[
            (POWER_CONTROL_A, &[0x39, 0x2C, 0x00, 0x34, 0x02]),
            (POWER_CONTROL_B, &[0x00, 0xC1, 0x30]),
            (TIMER_CONTROL_A, &[0x85, 0x00, 0x78]),
            (TIMER_CONTROL_B, &[0x00, 0x00]),
            (POWER_ON_SEQUENCE_CONTROL, &[0x64, 0x03, 0x12, 0x81]),
            (PUMP_RATIO_COMMAND, &[0x20]),
            (POWER_CONTROL_VRH, &[0x23]),
            (POWER_CONTROL_SAP_BT, &[0x10]),
            (VCM_CONTROL_1, &[0x3E, 0x28]),
            (VCM_CONTROL_2, &[0x86]),
            (MEMORY_ACCESS_CONTROL, &[0x48]),
            (PIXEL_FORMAT, &[0x55]),
            (FRAME_RATIO_CONTROL, &[0x00, 0x18]),
            (DISPLAY_FUNCTION_CONTROL, &[0x08, 0x82, 0x27]),
            (GAMMA_FUNCTION_DISPLAY, &[0x00]),
            (GAMMA_CURVE_SELECTED, &[0x01]),
            (
                POSITIVE_GAMMA_CORRECTION,
                &[
                    0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E,
                    0x09, 0x00,
                ],
            ),
            (
                NEGATIVE_GAMMA_CORRECTION,
                &[
                    0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31,
                    0x36, 0x0F,
                ],
            ),
        ];

        let mut screen = Self {
            spi,
            cs,
            rst,
            dc,
            bl,
            view_width: 0,
            view_height: 0,
        };

        screen.deselect();
        screen.select();
        screen.reset().await;

        screen.write_command(SOFTWARE_RESET)?;
        Timer::after_millis(5).await;

        for (command, data) in STARTUP_SEQUENCE {
            screen.write_command(*command)?;
            screen.write_data(*data)?;
        }

        screen.write_command(EXIT_SLEEP)?;

        Timer::after_millis(120).await;

        screen.write_command(DISPLAY_ON)?;
        screen.deselect();

        Ok(screen)
    }

    fn deselect(&mut self) {
        self.cs.set_low();
    }

    fn select(&mut self) {
        self.cs.set_high();
    }

    async fn reset(&mut self) {
        self.rst.set_low();
        Timer::after_millis(50).await;
        self.rst.set_high();
    }

    fn write_command(&mut self, command: u8) -> Result<(), spi::Error> {
        self.dc.set_low();
        self.spi.blocking_write(&[command])
    }

    // XXX(RLB) Note select needs to be called prior and deselect following
    fn write_data(&mut self, mut data: &[u8]) -> Result<(), spi::Error> {
        const MAX_CHUNK_SIZE: usize = 1 << 15;
        self.dc.set_high();
        while !data.is_empty() {
            let chunk_size = if data.len() > MAX_CHUNK_SIZE {
                MAX_CHUNK_SIZE
            } else {
                data.len()
            };
            self.spi.blocking_write(&data[..chunk_size])?;
            data = &data[chunk_size..];
        }

        Ok(())
    }

    fn set_orientation(&mut self, orientation: Orientation) -> Result<(), spi::Error> {
        self.write_command(ROTATION_CONTROL)?;
        self.write_data(&[u8::from(orientation)])?;
        self.view_width = orientation.width();
        self.view_height = orientation.height();
        Ok(())
    }

    fn disable_backlight(&mut self) {
        self.bl.set_low();
    }

    fn enable_backlight(&mut self) {
        self.bl.set_high();
    }

    /*
    fn fill_screen(&mut self, color: u16) -> Result<(), spi::Error> {
        self.fill_rectangle_blocking(0, 0, self.view_width, self.view_height, color);
    }

    fn fill_rectangle_blocking(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, color: u16) {
        if x0 >= self.view_width || y0 >= self.view_height || x1 <= x0 || y1 <= y0 {
            // TODO(RLB) make this fallible
            unreachable!();
        }

        let x1 = core::cmp::max(x0, x1);
        let y1 = core::cmp::max(y0, y1);

        self.wait_until_spi_free().await; // TODO

        self.select();
        self.set_writable_pixels(x0, y0, x1 - 1, y1 - 1);

        let width = x1 - x0;
        let height = y1 - y0;
        let total_pixel_bytes = (width as u32) * (height as u32) * 2;

        todo!(); // TODO(RLB) continue
    }
    */

    // TODO(RLB) Have a nicer interface than u16 for pixels, something like `struct
    // PixelBuffer<'a>(&'a [u16])`.
    async fn update_area(
        &mut self,
        x0: usize,
        y0: usize,
        x1: usize,
        y1: usize,
        pixels: &[u16],
    ) -> Result<(), spi::Error> {
        if x0 >= self.view_width || y0 >= self.view_height || x1 <= x0 || y1 <= y0 {
            // TODO(RLB) make this fallible
            info!("nonsensical dimensions");
            unreachable!();
        }

        let x1 = core::cmp::min(x1, self.view_width);
        let y1 = core::cmp::min(y1, self.view_height);

        let width = x1 - x0;
        let height = y1 - y0;
        let total_pixels = (width as usize) * (height as usize);
        if pixels.len() != total_pixels {
            // TODO(RLB) fail
            info!(
                "incorrect pixel buffer size {} != {} = {} * {}",
                pixels.len(),
                total_pixels,
                width,
                height,
            );
            unreachable!();
        }

        let col_data: [u8; 4] = [
            (x0 >> 8) as u8,
            (x0 & 0xff) as u8,
            (x1 >> 8) as u8,
            (x1 & 0xff) as u8,
        ];
        let row_data: [u8; 4] = [
            (y0 >> 8) as u8,
            (y0 & 0xff) as u8,
            (y1 >> 8) as u8,
            (y1 & 0xff) as u8,
        ];

        info!("col_data = {:?}", col_data);
        info!("row_data = {:?}", col_data);
        info!("pixels = {}", pixels.len());

        // XXX(RLB) Create a stack-allocated buffer on the fly
        // XXX(RLB) If this buffer is too big, the stack overflows.
        let mut pixel_buffer = [0_u8; 200];
        for (i, b) in pixels.iter().map(|b| b.to_be_bytes()).flatten().enumerate() {
            pixel_buffer[i] = b;
        }

        let pixel_buffer = &pixel_buffer[..(2 * total_pixels)];

        info!("pixel_buffer {} = {:?}", pixel_buffer.len(), pixel_buffer);

        const MAX_CHUNK_SIZE: usize = 1024;
        self.select();

        // Set the writable area
        self.write_command(COLUMN_ADDRESS_SET)?;
        self.write_data(&col_data)?;

        self.write_command(ROW_ADDRESS_SET)?;
        self.write_data(&row_data)?;

        self.write_command(WRITE_TO_RAM)?;

        info!("writing!");
        self.write_data(&pixel_buffer)?;

        self.deselect();

        Ok(())
    }
}

// TODO(RLB) Change this into an enum?
// TODO(RLB) What is the right size for these?
const SOFTWARE_RESET: u8 = 0x01;
const POWER_CONTROL_A: u8 = 0xCB;
const POWER_CONTROL_B: u8 = 0xCF;
const TIMER_CONTROL_A: u8 = 0xE8;
const TIMER_CONTROL_B: u8 = 0xEA;
const POWER_ON_SEQUENCE_CONTROL: u8 = 0xED;
const PUMP_RATIO_COMMAND: u8 = 0xF7;
const POWER_CONTROL_VRH: u8 = 0xC0;
const POWER_CONTROL_SAP_BT: u8 = 0xC1;
const VCM_CONTROL_1: u8 = 0xC5;
const VCM_CONTROL_2: u8 = 0xC7;
const MEMORY_ACCESS_CONTROL: u8 = 0x36;
const PIXEL_FORMAT: u8 = 0x3A;
const FRAME_RATIO_CONTROL: u8 = 0xB1;
const DISPLAY_FUNCTION_CONTROL: u8 = 0xB6;
const GAMMA_FUNCTION_DISPLAY: u8 = 0xF2;
const GAMMA_CURVE_SELECTED: u8 = 0x26;
const POSITIVE_GAMMA_CORRECTION: u8 = 0xE0;
const NEGATIVE_GAMMA_CORRECTION: u8 = 0xE1;
const EXIT_SLEEP: u8 = 0x11;
const DISPLAY_ON: u8 = 0x29;
const ROTATION_CONTROL: u8 = 0x36;
const COLUMN_ADDRESS_SET: u8 = 0x2a;
const ROW_ADDRESS_SET: u8 = 0x2b;
const WRITE_TO_RAM: u8 = 0x2c;

#[derive(Copy, Clone, Debug, PartialEq)]
enum Orientation {
    Portrait,
    FlippedPortrait,
    LeftLandscape,
    RightLandscape,
}

impl Orientation {
    const WIDTH: usize = 240;
    const HEIGHT: usize = 320;

    // XXX(RLB) These values need more descriptive names
    const MY: u8 = 0x80;
    const MX: u8 = 0x40;
    const MV: u8 = 0x20;
    const BGR: u8 = 0x08;

    const PORTRAIT_MODE: u8 = Self::MY | Self::BGR;
    const FLIPPED_PORTRAIT_MODE: u8 = Self::MX | Self::BGR;
    const LEFT_LANDSCAPE_MODE: u8 = Self::MV | Self::BGR;
    const RIGHT_LANDSCAPE_MODE: u8 = Self::MX | Self::MY | Self::MV | Self::BGR;

    fn width(&self) -> usize {
        match *self {
            Orientation::Portrait | Orientation::FlippedPortrait => Self::WIDTH,
            Orientation::LeftLandscape | Orientation::RightLandscape => Self::HEIGHT,
        }
    }

    fn height(&self) -> usize {
        match *self {
            Orientation::Portrait | Orientation::FlippedPortrait => Self::WIDTH,
            Orientation::LeftLandscape | Orientation::RightLandscape => Self::HEIGHT,
        }
    }
}

impl From<Orientation> for u8 {
    fn from(orientation: Orientation) -> u8 {
        match orientation {
            Orientation::Portrait => Orientation::PORTRAIT_MODE,
            Orientation::FlippedPortrait => Orientation::FLIPPED_PORTRAIT_MODE,
            Orientation::LeftLandscape => Orientation::LEFT_LANDSCAPE_MODE,
            Orientation::RightLandscape => Orientation::RIGHT_LANDSCAPE_MODE,
        }
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
