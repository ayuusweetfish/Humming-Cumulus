#include <stm32f1xx_hal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define LED_PORT    GPIOC
#define LED_PIN_ACT GPIO_PIN_13

static uint8_t swv_buf[256];
static size_t swv_buf_ptr = 0;
__attribute__ ((noinline, used))
void swv_trap_line()
{
  *(volatile char *)swv_buf;
}
static inline void swv_putchar(uint8_t c)
{
  // ITM_SendChar(c);
  if (c == '\n') {
    swv_buf[swv_buf_ptr >= sizeof swv_buf ?
      (sizeof swv_buf - 1) : swv_buf_ptr] = '\0';
    swv_trap_line();
    swv_buf_ptr = 0;
  } else if (++swv_buf_ptr <= sizeof swv_buf) {
    swv_buf[swv_buf_ptr - 1] = c;
  }
}
static void swv_printf(const char *restrict fmt, ...)
{
  char s[256];
  va_list args;
  va_start(args, fmt);
  int r = vsnprintf(s, sizeof s, fmt, args);
  for (int i = 0; i < r && i < sizeof s - 1; i++) swv_putchar(s[i]);
  if (r >= sizeof s) {
    for (int i = 0; i < 3; i++) swv_putchar('.');
    swv_putchar('\n');
  }
}

SPI_HandleTypeDef spi1 = { 0 };
TIM_HandleTypeDef tim2 = { 0 };

#pragma GCC optimize ("O3")
static inline void i2s_delay_half()
{
  // LRCLK = 48 kHz
  // BCLK = 32 * LRCLK = 1536 kHz
  // Half cycle = 23.4375 HCLK cycles
  // Note: acceptable LRCLK range 30.4 ~ 50.4 kHz
  // -> Half cycle = 22.3 ~ 37.0 cycles
  uint32_t start = DWT->CYCCNT;
  while (DWT->CYCCNT - start < 23) { }
}

#pragma GCC optimize ("O3")
static inline void i2s_dump()
{
  // Theoretically this should yield a sample rate of 48 kHz
  // and an ACT-LED toggle frequency of 1 Hz
  // But this seems to fall in the range of 16 kHz (?)
  uint32_t seed = 0;
  uint32_t phase = 0;
  while (1) {
    // seed = (seed * 1103515245 + 12345) & 0x7fffffff;
    // uint16_t value = ((seed >> 8) & 0xffff) / 4;
    phase = (phase + 1) % 64;  // 48 kHz / 64 = 750 Hz
    // uint16_t value = abs((int32_t)phase - 32) * 512;
    // uint16_t value = phase * 64;
    uint16_t value = (phase < 32) * 4096;
    for (int ch = 0; ch <= 1; ch++) {
      for (int i = 15; i >= 0; i--) {
        if (i == 0) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, !ch);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 0);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, (value >> i) & 1);
        i2s_delay_half();
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, 1);
        i2s_delay_half();
      }
    }
  }
}

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  gpio_init.Pin = LED_PIN_ACT;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_PORT, &gpio_init);
  HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, GPIO_PIN_RESET);

  // ======== Clocks ========
  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc_init.HSEState = RCC_HSE_ON;
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc_init.PLL.PLLMUL = RCC_PLL_MUL9;
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1 |
    RCC_CLOCKTYPE_PCLK2;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
  clk_init.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

  // Cycle count
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0;

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  // ======== SPI ========
  // GPIO ports
  // SPI1_SCK (PA5), SPI1_MOSI (PA7)
  gpio_init.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  // gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);
  // LRCLK (PA4)
  gpio_init.Pin = GPIO_PIN_4;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  __HAL_RCC_SPI1_CLK_ENABLE();
  spi1.Instance = SPI1;
  spi1.Init.Mode = SPI_MODE_MASTER;
  spi1.Init.Direction = SPI_DIRECTION_2LINES;
  spi1.Init.CLKPolarity = SPI_POLARITY_LOW; // CPOL = 0
  spi1.Init.CLKPhase = SPI_PHASE_1EDGE;     // CPHA = 0
  spi1.Init.NSS = SPI_NSS_SOFT;
  spi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  spi1.Init.TIMode = SPI_TIMODE_DISABLE;
  spi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  spi1.Init.DataSize = SPI_DATASIZE_8BIT;
  spi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  HAL_SPI_Init(&spi1);
  __HAL_SPI_ENABLE(&spi1);

  // ======== Timer ========
  __HAL_RCC_TIM2_CLK_ENABLE();
  // One trigger per second (48 kHz * 32 bits)
  tim2 = (TIM_HandleTypeDef){
    .Instance = TIM2,
    .Init = {
      .Prescaler = 48 * 32 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 1000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_Base_Init(&tim2);

  TIM_ClockConfigTypeDef tim2_cfg = {
    .ClockSource = TIM_CLOCKSOURCE_TI1,
    .ClockPolarity = TIM_CLOCKPOLARITY_FALLING,
    .ClockPrescaler = TIM_CLOCKPRESCALER_DIV1,
    .ClockFilter = 0,
  };
  HAL_TIM_ConfigClockSource(&tim2, &tim2_cfg);
  HAL_TIM_ConfigTI1Input(&tim2, TIM_TI1SELECTION_CH1);

  // TIM2_CH1_ETR (PA0)
  gpio_init.Pin = GPIO_PIN_0;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  HAL_TIM_Base_Start_IT(&tim2);
  __HAL_TIM_ENABLE_IT(&tim2, TIM_IT_UPDATE);
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);

  // One trigger (toggle) per second
  uint32_t start = DWT->CYCCNT;
  while (0) {
    GPIOA->BSRR = ((uint32_t)GPIO_PIN_5 << 16);
    while (DWT->CYCCNT - start < 23) { }
    GPIOA->BSRR = GPIO_PIN_5;
    while (DWT->CYCCNT - start < 46) { }
    start += 46;
  }

  i2s_dump();

  while (1) {
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, GPIO_PIN_RESET);
    HAL_Delay(200);
  }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void TIM2_IRQHandler()
{
  HAL_TIM_IRQHandler(&tim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *tim2)
{
  static uint32_t count = 0;
  count ^= 1;
  HAL_GPIO_WritePin(LED_PORT, LED_PIN_ACT, count);
}
