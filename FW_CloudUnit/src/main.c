#include <stm32g0xx_hal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define LED_IND_ACT_PORT  GPIOC
#define LED_IND_ACT_PIN   GPIO_PIN_15

#define LED_OUT_R_PORT  GPIOA
#define LED_OUT_R_PIN   GPIO_PIN_4
#define LED_OUT_G_PORT  GPIOB
#define LED_OUT_G_PIN   GPIO_PIN_6
#define LED_OUT_B_PORT  GPIOB
#define LED_OUT_B_PIN   GPIO_PIN_7

#define DRV_EN_PORT   GPIOA
#define DRV_EN_U_PIN  GPIO_PIN_1
#define DRV_EN_V_PIN  GPIO_PIN_2
#define DRV_EN_W_PIN  GPIO_PIN_5
#define DRV_IN_U_PORT GPIOA
#define DRV_IN_U_PIN  GPIO_PIN_6
#define DRV_IN_V_PORT GPIOA
#define DRV_IN_V_PIN  GPIO_PIN_7
#define DRV_IN_W_PORT GPIOB
#define DRV_IN_W_PIN  GPIO_PIN_0

TIM_HandleTypeDef tim14, tim16, tim17;
I2C_HandleTypeDef i2c2;

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
  // APB1 = 16 MHz
  // period = 1 kHz = 16000 cycles

  // LED Red, TIM14
  gpio_init.Pin = LED_OUT_R_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF4_TIM14;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_OUT_R_PORT, &gpio_init);
  __HAL_RCC_TIM14_CLK_ENABLE();
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
    .OCMode = TIM_OCMODE_PWM2,
    .Pulse = 0, // to be filled
    .OCPolarity = TIM_OCPOLARITY_HIGH,
  };
  HAL_TIM_PWM_ConfigChannel(&tim14, &tim14_ch1_oc_init, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim14, TIM_CHANNEL_1);

  // LED Green, TIM16
  gpio_init.Pin = LED_OUT_G_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF2_TIM16;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_OUT_G_PORT, &gpio_init);
  __HAL_RCC_TIM16_CLK_ENABLE();
  tim16 = (TIM_HandleTypeDef){
    .Instance = TIM16,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 16000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_PWM_Init(&tim16);
  TIM_OC_InitTypeDef tim16_ch1_oc_init = {
    .OCMode = TIM_OCMODE_PWM2,
    .Pulse = 0, // to be filled
    .OCNPolarity = TIM_OCNPOLARITY_HIGH,  // Output is TIM16_CH1N
  };
  HAL_TIM_PWM_ConfigChannel(&tim16, &tim16_ch1_oc_init, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&tim16, TIM_CHANNEL_1);

  // LED Blue, TIM17
  gpio_init.Pin = LED_OUT_B_PIN;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF2_TIM17;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_OUT_B_PORT, &gpio_init);
  __HAL_RCC_TIM17_CLK_ENABLE();
  tim17 = (TIM_HandleTypeDef){
    .Instance = TIM17,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = 16000 - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_PWM_Init(&tim17);
  TIM_OC_InitTypeDef tim17_ch1_oc_init = {
    .OCMode = TIM_OCMODE_PWM2,
    .Pulse = 0, // to be filled
    .OCNPolarity = TIM_OCNPOLARITY_HIGH,  // Output is TIM17_CH1N
  };
  HAL_TIM_PWM_ConfigChannel(&tim17, &tim17_ch1_oc_init, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&tim17, TIM_CHANNEL_1);

  // Driver PWMs, TIM3
  // gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio_init.Pin = DRV_IN_U_PIN;
  gpio_init.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(DRV_IN_U_PORT, &gpio_init);
  gpio_init.Pin = DRV_IN_V_PIN;
  gpio_init.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(DRV_IN_V_PORT, &gpio_init);
  gpio_init.Pin = DRV_IN_W_PIN;
  gpio_init.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(DRV_IN_W_PORT, &gpio_init);

  // Also, the driver phase-enable signals
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pin = DRV_EN_U_PIN | DRV_EN_V_PIN | DRV_EN_W_PIN;
  HAL_GPIO_Init(DRV_EN_PORT, &gpio_init);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_U_PIN | DRV_EN_V_PIN | DRV_EN_W_PIN, GPIO_PIN_RESET);

  // ======== I2C ========
  gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Alternate = GPIO_AF6_I2C2;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio_init);

  __HAL_RCC_I2C2_CLK_ENABLE();
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
  i2c2 = (I2C_HandleTypeDef){
    .Instance = I2C2,
    .Init = {
      // RM0454 Rev 5, pp. 711, 726, 738 (examples)
      // APB = 16 MHz, fast mode f_SCL = 100 kHz
      // PRESC = 3, SCLDEL = 0x4, SDADEL = 0x2,
      // SCLH = 0x0F, SCLH = 0x0F, SCLL = 0x13
      .Timing = 0x30420F13,
      .OwnAddress1 = 0x00,
      .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    },
  };
  HAL_I2C_Init(&i2c2);

  while (true) {
    static const uint8_t signals[6][6] = {
      {0, 0, 1, 1, 0, 1},
      {1, 1, 0, 0, 0, 1},
      {1, 1, 0, 1, 0, 0},
      {0, 0, 0, 1, 1, 1},
      {0, 1, 0, 0, 1, 1},
      {0, 1, 1, 1, 0, 0},
    };
    for (int i = 0; i < 6; i++) {
      HAL_GPIO_WritePin(DRV_IN_U_PORT, DRV_IN_U_PIN, signals[i][0]);
      HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_U_PIN, signals[i][1]);
      HAL_GPIO_WritePin(DRV_IN_V_PORT, DRV_IN_V_PIN, signals[i][2]);
      HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_V_PIN, signals[i][3]);
      HAL_GPIO_WritePin(DRV_IN_W_PORT, DRV_IN_W_PIN, signals[i][4]);
      HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_W_PIN, signals[i][5]);
      HAL_Delay(200);
    }
  }

  // Read registers from AS5600
  while (1) {
    uint8_t status = 0, agc = 0;
    uint8_t raw_angle[2];
    HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x0B, I2C_MEMADD_SIZE_8BIT, &status, 1, 1000);
    HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, &agc, 1, 1000);
    HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x0E, I2C_MEMADD_SIZE_8BIT, raw_angle, 2, 1000);
  /*
    swv_printf("status = %02x, AGC = %02x, raw angle = %4u\n",
      status & 0x38, agc, ((uint32_t)raw_angle[0] << 8) | raw_angle[1]);
    HAL_Delay(200);
  */
    if (status & 0x20) {
      // MD: Magnet detected
      if (!(status & 0x18)) {
        // No ML or MH: within recommended magnitude range
        uint32_t a = ((uint32_t)raw_angle[0] << 8) | raw_angle[1];
        TIM14->CCR1 = abs(a - 2048) * 6;
        TIM16->CCR1 = (2048 - abs(a - 2048)) * 5;
        TIM17->CCR1 = 1000;
      } else {
        // Out of range
        TIM14->CCR1 = 0;
        TIM16->CCR1 = 0;
        TIM17->CCR1 = 4000;
      }
    } else {
      TIM14->CCR1 = 0;
      TIM16->CCR1 = 0;
      TIM17->CCR1 = 0;
    }
    HAL_Delay(1);
  }

  // ======== Main loop ========
  while (true) {
    for (int i = 0; i <= 10800; i += 3) {
      uint32_t duty[3] = {0, 0, 0};
      for (int k = 0; k < 3; k++) {
        int dist1 = abs(i - (2700 + 3600 * k));
        int dist2 = abs(i + 10800 - (2700 + 3600 * k));
        int dist = (dist1 < dist2 ? dist1 : dist2);
        if (dist < 2700)
          duty[k] = 16000 - dist * 16000 / 2700;
      }
      TIM14->CCR1 = duty[0];
      TIM16->CCR1 = duty[1];
      TIM17->CCR1 = duty[2];
      HAL_Delay(3);
    }
  }
}

void SysTick_Handler()
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();

  if (HAL_GetTick() % 500 == 0)
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, HAL_GetTick() % 1000 != 0);
}
