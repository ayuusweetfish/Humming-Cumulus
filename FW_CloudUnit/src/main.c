#include <stm32g0xx_hal.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define LED_IND_ACT_PORT  GPIOA
#define LED_IND_ACT_PIN   GPIO_PIN_5

#define LED_OUT_R_PORT  GPIOA
#define LED_OUT_R_PIN   GPIO_PIN_4
#define LED_OUT_G_PORT  GPIOB
#define LED_OUT_G_PIN   GPIO_PIN_6
#define LED_OUT_B_PORT  GPIOB
#define LED_OUT_B_PIN   GPIO_PIN_7

#define DRV_NFAULT_PORT GPIOA
#define DRV_NFAULT_PIN  GPIO_PIN_1
#define DRV_NSLEEP_PORT GPIOA
#define DRV_NSLEEP_PIN  GPIO_PIN_2
#define DRV_IN_U_PORT GPIOA
#define DRV_IN_U_PIN  GPIO_PIN_6
#define DRV_IN_V_PORT GPIOA
#define DRV_IN_V_PIN  GPIO_PIN_7
#define DRV_IN_W_PORT GPIOB
#define DRV_IN_W_PIN  GPIO_PIN_0

TIM_HandleTypeDef tim14, tim16, tim17, tim3;
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

#define PWM_RESOLUTION 1000
// angle: [0, 36000000)
/*
from math import *
N=720
print(', '.join('%d' % round(1000*(1+0.2*sin(i/N*2*pi))/2) for i in range(N)))
*/
const uint16_t sin_lookup[720] = {
500, 501, 502, 503, 503, 504, 505, 506, 507, 508, 509, 510, 510, 511, 512, 513, 514, 515, 516, 517, 517, 518, 519, 520, 521, 522, 522, 523, 524, 525, 526, 527, 528, 528, 529, 530, 531, 532, 533, 533, 534, 535, 536, 537, 537, 538, 539, 540, 541, 541, 542, 543, 544, 545, 545, 546, 547, 548, 548, 549, 550, 551, 552, 552, 553, 554, 554, 555, 556, 557, 557, 558, 559, 559, 560, 561, 562, 562, 563, 564, 564, 565, 566, 566, 567, 568, 568, 569, 569, 570, 571, 571, 572, 573, 573, 574, 574, 575, 575, 576, 577, 577, 578, 578, 579, 579, 580, 580, 581, 581, 582, 582, 583, 583, 584, 584, 585, 585, 586, 586, 587, 587, 587, 588, 588, 589, 589, 589, 590, 590, 591, 591, 591, 592, 592, 592, 593, 593, 593, 594, 594, 594, 595, 595, 595, 595, 596, 596, 596, 596, 597, 597, 597, 597, 597, 598, 598, 598, 598, 598, 598, 599, 599, 599, 599, 599, 599, 599, 599, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 599, 599, 599, 599, 599, 599, 599, 599, 598, 598, 598, 598, 598, 598, 597, 597, 597, 597, 597, 596, 596, 596, 596, 595, 595, 595, 595, 594, 594, 594, 593, 593, 593, 592, 592, 592, 591, 591, 591, 590, 590, 589, 589, 589, 588, 588, 587, 587, 587, 586, 586, 585, 585, 584, 584, 583, 583, 582, 582, 581, 581, 580, 580, 579, 579, 578, 578, 577, 577, 576, 575, 575, 574, 574, 573, 573, 572, 571, 571, 570, 569, 569, 568, 568, 567, 566, 566, 565, 564, 564, 563, 562, 562, 561, 560, 559, 559, 558, 557, 557, 556, 555, 554, 554, 553, 552, 552, 551, 550, 549, 548, 548, 547, 546, 545, 545, 544, 543, 542, 541, 541, 540, 539, 538, 537, 537, 536, 535, 534, 533, 533, 532, 531, 530, 529, 528, 528, 527, 526, 525, 524, 523, 522, 522, 521, 520, 519, 518, 517, 517, 516, 515, 514, 513, 512, 511, 510, 510, 509, 508, 507, 506, 505, 504, 503, 503, 502, 501, 500, 499, 498, 497, 497, 496, 495, 494, 493, 492, 491, 490, 490, 489, 488, 487, 486, 485, 484, 483, 483, 482, 481, 480, 479, 478, 478, 477, 476, 475, 474, 473, 472, 472, 471, 470, 469, 468, 467, 467, 466, 465, 464, 463, 463, 462, 461, 460, 459, 459, 458, 457, 456, 455, 455, 454, 453, 452, 452, 451, 450, 449, 448, 448, 447, 446, 446, 445, 444, 443, 443, 442, 441, 441, 440, 439, 438, 438, 437, 436, 436, 435, 434, 434, 433, 432, 432, 431, 431, 430, 429, 429, 428, 427, 427, 426, 426, 425, 425, 424, 423, 423, 422, 422, 421, 421, 420, 420, 419, 419, 418, 418, 417, 417, 416, 416, 415, 415, 414, 414, 413, 413, 413, 412, 412, 411, 411, 411, 410, 410, 409, 409, 409, 408, 408, 408, 407, 407, 407, 406, 406, 406, 405, 405, 405, 405, 404, 404, 404, 404, 403, 403, 403, 403, 403, 402, 402, 402, 402, 402, 402, 401, 401, 401, 401, 401, 401, 401, 401, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 401, 401, 401, 401, 401, 401, 401, 401, 402, 402, 402, 402, 402, 402, 403, 403, 403, 403, 403, 404, 404, 404, 404, 405, 405, 405, 405, 406, 406, 406, 407, 407, 407, 408, 408, 408, 409, 409, 409, 410, 410, 411, 411, 411, 412, 412, 413, 413, 413, 414, 414, 415, 415, 416, 416, 417, 417, 418, 418, 419, 419, 420, 420, 421, 421, 422, 422, 423, 423, 424, 425, 425, 426, 426, 427, 427, 428, 429, 429, 430, 431, 431, 432, 432, 433, 434, 434, 435, 436, 436, 437, 438, 438, 439, 440, 441, 441, 442, 443, 443, 444, 445, 446, 446, 447, 448, 448, 449, 450, 451, 452, 452, 453, 454, 455, 455, 456, 457, 458, 459, 459, 460, 461, 462, 463, 463, 464, 465, 466, 467, 467, 468, 469, 470, 471, 472, 472, 473, 474, 475, 476, 477, 478, 478, 479, 480, 481, 482, 483, 483, 484, 485, 486, 487, 488, 489, 490, 490, 491, 492, 493, 494, 495, 496, 497, 497, 498, 499
};

static inline void drive_motor(uint32_t angle)
{
  TIM3->CCR1 = sin_lookup[(angle / 50000 +   0) % 720];
  TIM3->CCR2 = sin_lookup[(angle / 50000 + 240) % 720];
  TIM3->CCR3 = sin_lookup[(angle / 50000 + 480) % 720];
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
  HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, GPIO_PIN_SET);

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
  osc_init.PLL.PLLState = RCC_PLL_ON;
  osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc_init.PLL.PLLM = RCC_PLLM_DIV2;
  osc_init.PLL.PLLN = 8;
  osc_init.PLL.PLLP = RCC_PLLP_DIV2;
  osc_init.PLL.PLLR = RCC_PLLR_DIV2;
  HAL_RCC_OscConfig(&osc_init);

  RCC_ClkInitTypeDef clk_init = { 0 };
  clk_init.ClockType =
    RCC_CLOCKTYPE_SYSCLK |
    RCC_CLOCKTYPE_HCLK |
    RCC_CLOCKTYPE_PCLK1;
  clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk_init.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_2);

  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  // ======== Timer ========
  // APB1 = 64 MHz
  // period = 4 kHz = 16000 cycles

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
  gpio_init.Mode = GPIO_MODE_AF_PP;
  // gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
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
  HAL_GPIO_WritePin(DRV_IN_U_PORT, DRV_IN_U_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV_IN_V_PORT, DRV_IN_V_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(DRV_IN_W_PORT, DRV_IN_W_PIN, GPIO_PIN_RESET);

  // Timer
  // APB1 = 64 MHz
  // Period = 64 MHz / 1000 = 64 kHz
  __HAL_RCC_TIM3_CLK_ENABLE();
  tim3 = (TIM_HandleTypeDef){
    .Instance = TIM3,
    .Init = {
      .Prescaler = 1 - 1,
      .CounterMode = TIM_COUNTERMODE_UP,
      .Period = PWM_RESOLUTION - 1,
      .ClockDivision = TIM_CLOCKDIVISION_DIV1,
      .RepetitionCounter = 0,
    },
  };
  HAL_TIM_PWM_Init(&tim3);
  TIM_OC_InitTypeDef tim3_ch1_oc_init = {
    .OCMode = TIM_OCMODE_PWM2,
    .Pulse = 0, // to be filled
    .OCPolarity = TIM_OCPOLARITY_HIGH,
  };
  HAL_TIM_PWM_ConfigChannel(&tim3, &tim3_ch1_oc_init, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&tim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_ConfigChannel(&tim3, &tim3_ch1_oc_init, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&tim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_ConfigChannel(&tim3, &tim3_ch1_oc_init, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&tim3, TIM_CHANNEL_3);

  // Driver nFAULT and nSLEEP pins
  gpio_init.Pin = DRV_NFAULT_PIN;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_PULLUP;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DRV_NFAULT_PORT, &gpio_init);

  gpio_init.Pin = DRV_NSLEEP_PIN;
  gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DRV_NSLEEP_PORT, &gpio_init);
  HAL_GPIO_WritePin(DRV_NSLEEP_PORT, DRV_NSLEEP_PIN, 1);

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
      // RM0454 Rev 5, pp. 711, 726, 738 (examples), 766
      // APB = 64 MHz, fast mode f_SCL = 100 kHz
      // PRESC = 15, SCLDEL = 0x4, SDADEL = 0x2,
      // SCLH = 0x0F, SCLH = 0x0F, SCLL = 0x13
      .Timing = 0xF0420F13,
      .OwnAddress1 = 0x00,
      .AddressingMode = I2C_ADDRESSINGMODE_7BIT,
    },
  };
  HAL_I2C_Init(&i2c2);

  HAL_StatusTypeDef enc_ready = HAL_I2C_IsDeviceReady(&i2c2, 0x36 << 1, 3, 1000);
  swv_printf("encoder ready status: %d\n", (int)enc_ready);

  while (1) {
    static int chroma = 2;
    static TIM_TypeDef *const chroma_timers[3] = {TIM14, TIM16, TIM17};
    chroma = (chroma + 1) % 3;
    TIM14->CCR1 = 0;
    TIM16->CCR1 = 0;
    TIM17->CCR1 = 0;
    for (int i = 0; i < 32400; i += 1) {
      float t = 1 - cosf((float)i / 32400 * 6.2831853f);
      // angle normalized into [0, 36000000)
      int angle = (int)(0.5f + 72000000 + 432000000 * t);
      drive_motor(angle);
      // T
      chroma_timers[chroma]->CCR1 = 4000 * t;
    }
    static int parity = 1;
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, parity ^= 1);

    // Read registers from AS5600
    uint8_t status = 0, agc = 0;
    uint8_t raw_angle[2];
    HAL_StatusTypeDef r1 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x0B, I2C_MEMADD_SIZE_8BIT, &status, 1, 1000);
    HAL_StatusTypeDef r2 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, &agc, 1, 1000);
    HAL_StatusTypeDef r3 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x0E, I2C_MEMADD_SIZE_8BIT, raw_angle, 2, 1000);
    swv_printf("status = %02x, AGC = %02x, raw angle = %4u, returned status = %d %d %d, error = %d\n",
      status & 0x38, agc, ((uint32_t)raw_angle[0] << 8) | raw_angle[1], r1, r2, r3, (int)i2c2.ErrorCode);
    // status bit 5: MD, bit 4: ML, bit 3: MH
    // HAL_I2C_ERROR_TIMEOUT (32) can arise when SCL/SDA pull-ups are not well soldered
    // Otherwise device NACK should be HAL_I2C_ERROR_AF (4)
    HAL_Delay(200);
  }

  // Read registers from AS5600
  while (1) {
    uint8_t status = 0, agc = 0;
    uint8_t raw_angle[2];
    HAL_StatusTypeDef r1 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x0B, I2C_MEMADD_SIZE_8BIT, &status, 1, 1000);
    HAL_StatusTypeDef r2 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, &agc, 1, 1000);
    HAL_StatusTypeDef r3 = HAL_I2C_Mem_Read(&i2c2, 0x36 << 1, 0x0E, I2C_MEMADD_SIZE_8BIT, raw_angle, 2, 1000);
    swv_printf("status = %02x, AGC = %02x, raw angle = %4u, returned status = %d %d %d\n",
      status & 0x38, agc, ((uint32_t)raw_angle[0] << 8) | raw_angle[1], r1, r2, r3);
    // status = 10, AGC = 80, raw angle = xxxx
    // status = 20, AGC = <80, raw angle = xxxx
    // status bit 5: MD, bit 4: ML, bit 3: MH
    HAL_Delay(200);
/*
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
*/
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

  // if (HAL_GetTick() % 500 == 0)
  //   HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, HAL_GetTick() % 1000 == 0);
}
