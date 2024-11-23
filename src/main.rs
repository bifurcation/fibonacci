#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::{
    gpio::{Level, Output, Speed},
    spi::Spi,
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

    // Configure SPI1
    let spi1 = {
        use embassy_stm32::spi::{Config, Spi};
        let mut config = Config::default(); // TODO

        Spi::new_txonly(
            p.SPI1, // SPI peripheral
            p.PA5,  // Clock pin
            p.PA7,  // MOSI pin
            config,
        )
    };

    let mut screen = Screen {
        spi: spi1,
        cs: p.PC14,
        dc: p.PC13,
        rst: p.PB9,
        bl: p.PB8,
    };

    // Do stuff
    loop {
        fib_test::<u32>(34, &mut led).await;
        fib_test::<u64>(34, &mut led).await;
        fib_test::<f32>(32, &mut led).await;
        fib_test::<f64>(32, &mut led).await;
    }
}

struct Screen<CS, DC, RST, BL>
where
    CS: Pin,  // C 14
    DC: Pin,  // C 13
    RST: Pin, // B 9
    BL: Pin,  // B 8
{
    spi: Spi,
    cs: CS,
    dc: DC,
    rst: RST,
    bl: BL,
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

/*

// stm32f4xx_hal_msp.c
/**
* @brief SPI MSP Initialization
* This function configures the hardware resources used in this example
* @param hspi: SPI handle pointer
* @retval None
*/
void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(hspi->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA5     ------> SPI1_SCK
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = SPI1_CLK_Pin|SPI1_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 DMA Init */
    /* SPI1_TX Init */
    hdma_spi1_tx.Instance = DMA2_Stream3;
    hdma_spi1_tx.Init.Channel = DMA_CHANNEL_3;
    hdma_spi1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_spi1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_spi1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_spi1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_spi1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hspi,hdmatx,hdma_spi1_tx);

    /* SPI1 interrupt Init */
    HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }

}




*/
