#include <stm32g0xx_hal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define LED_IND_ACT_PORT  GPIOC
#define LED_IND_ACT_PIN   GPIO_PIN_15

#define LED_OUT_R_PORT  GPIOA
#define LED_OUT_R_PIN   GPIO_PIN_4
#define LED_OUT_G_PORT  GPIOB
#define LED_OUT_G_PIN   GPIO_PIN_6
#define LED_OUT_B_PORT  GPIOB
#define LED_OUT_B_PIN   GPIO_PIN_7

TIM_HandleTypeDef tim14;

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

int main()
{
  HAL_Init();

  // ======== GPIO ========
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpio_init;

  gpio_init.Pin = LED_IND_ACT_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_PULLDOWN;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_IND_ACT_PORT, &gpio_init);
  HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, GPIO_PIN_RESET);

  // SWD (PA13, PA14)
  gpio_init.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  gpio_init.Mode = GPIO_MODE_AF_PP; // Pass over control to AF peripheral
  gpio_init.Alternate = GPIO_AF0_SWJ;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  // ======== Clocks ========
  RCC_OscInitTypeDef osc_init = { 0 };
  osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc_init.HSIState = RCC_HSI_ON;
  osc_init.PLL.PLLState = RCC_PLL_OFF;
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  // ======== Timer ========
  gpio_init.Pin = LED_OUT_R_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF4_TIM14;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_OUT_R_PORT, &gpio_init);
  __HAL_RCC_TIM14_CLK_ENABLE();
  // APB1 = 16 MHz
  // period = 1 kHz = 16000 cycles
  tim14 = (TIM_HandleTypeDef){
    .Instance = TIM14,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 16000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_PWM_Init(&tim14);
  TIM_OC_InitTypeDef tim14_ch1_oc_init = {
    .OCMode = TIM_OCMODE_PWM1,
    .Pulse = 0, // to be filled
    .OCPolarity = TIM_OCPOLARITY_LOW,
  };
  HAL_TIM_PWM_ConfigChannel(&tim14, &tim14_ch1_oc_init, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim14, TIM_CHANNEL_1);

  // ======== Main loop ========
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  gpio_init.Pin = LED_OUT_G_PIN; HAL_GPIO_Init(LED_OUT_G_PORT, &gpio_init);
  gpio_init.Pin = LED_OUT_B_PIN; HAL_GPIO_Init(LED_OUT_B_PORT, &gpio_init);
  HAL_GPIO_WritePin(LED_OUT_G_PORT, LED_OUT_G_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_OUT_B_PORT, LED_OUT_B_PIN, GPIO_PIN_SET);

  while (true) {
    for (int i = 0; i <= 16000; i += 10) {
      TIM14->CCR1 = i;
      HAL_Delay(1);
    }
    for (int i = 0; i <= 100; i++) {
      TIM14->CCR1 = i;
      HAL_Delay(10);
    }
    for (int i = 0; i < 8; i++) {
      // HAL_GPIO_WritePin(LED_OUT_R_PORT, LED_OUT_R_PIN, (~i >> 0) & 1);
      HAL_GPIO_WritePin(LED_OUT_G_PORT, LED_OUT_G_PIN, (~i >> 1) & 1);
      HAL_GPIO_WritePin(LED_OUT_B_PORT, LED_OUT_B_PIN, (~i >> 2) & 1);
      HAL_Delay(500);
    }
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, GPIO_PIN_SET);
    HAL_Delay(200);
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, GPIO_PIN_RESET);
    HAL_Delay(200);
  }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}
