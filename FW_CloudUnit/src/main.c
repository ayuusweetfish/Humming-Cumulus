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

#define PWM_RESOLUTION 6000
// angle: [0, 36000000)
/*
from math import *
N=720
for phase in range(3):
  print('{' + ', '.join('%d' % round(6000*(1+0.5*sin(i/N*2*pi + phase/3*2*pi))/2) for i in range(N)) + '},')
*/
/*
from math import *
N=720
print(', '.join('%d' % round(6000*(1+0.5*sin(i/N*2*pi))/2) for i in range(N)))
*/
const uint16_t sin_lookup[720] = {
3000, 3013, 3026, 3039, 3052, 3065, 3079, 3092, 3105, 3118, 3131, 3144, 3157, 3170, 3183, 3196, 3209, 3222, 3235, 3248, 3260, 3273, 3286, 3299, 3312, 3325, 3337, 3350, 3363, 3376, 3388, 3401, 3413, 3426, 3439, 3451, 3464, 3476, 3488, 3501, 3513, 3525, 3538, 3550, 3562, 3574, 3586, 3598, 3610, 3622, 3634, 3646, 3658, 3669, 3681, 3693, 3704, 3716, 3727, 3739, 3750, 3761, 3773, 3784, 3795, 3806, 3817, 3828, 3839, 3850, 3860, 3871, 3882, 3892, 3903, 3913, 3923, 3934, 3944, 3954, 3964, 3974, 3984, 3994, 4004, 4013, 4023, 4033, 4042, 4051, 4061, 4070, 4079, 4088, 4097, 4106, 4115, 4123, 4132, 4141, 4149, 4157, 4166, 4174, 4182, 4190, 4198, 4206, 4214, 4221, 4229, 4236, 4244, 4251, 4258, 4265, 4272, 4279, 4286, 4292, 4299, 4306, 4312, 4318, 4324, 4331, 4337, 4342, 4348, 4354, 4359, 4365, 4370, 4376, 4381, 4386, 4391, 4396, 4400, 4405, 4410, 4414, 4418, 4422, 4427, 4431, 4434, 4438, 4442, 4445, 4449, 4452, 4455, 4459, 4462, 4464, 4467, 4470, 4472, 4475, 4477, 4479, 4482, 4484, 4485, 4487, 4489, 4490, 4492, 4493, 4494, 4495, 4496, 4497, 4498, 4499, 4499, 4499, 4500, 4500, 4500, 4500, 4500, 4499, 4499, 4499, 4498, 4497, 4496, 4495, 4494, 4493, 4492, 4490, 4489, 4487, 4485, 4484, 4482, 4479, 4477, 4475, 4472, 4470, 4467, 4464, 4462, 4459, 4455, 4452, 4449, 4445, 4442, 4438, 4434, 4431, 4427, 4422, 4418, 4414, 4410, 4405, 4400, 4396, 4391, 4386, 4381, 4376, 4370, 4365, 4359, 4354, 4348, 4342, 4337, 4331, 4324, 4318, 4312, 4306, 4299, 4292, 4286, 4279, 4272, 4265, 4258, 4251, 4244, 4236, 4229, 4221, 4214, 4206, 4198, 4190, 4182, 4174, 4166, 4157, 4149, 4141, 4132, 4123, 4115, 4106, 4097, 4088, 4079, 4070, 4061, 4051, 4042, 4033, 4023, 4013, 4004, 3994, 3984, 3974, 3964, 3954, 3944, 3934, 3923, 3913, 3903, 3892, 3882, 3871, 3860, 3850, 3839, 3828, 3817, 3806, 3795, 3784, 3773, 3761, 3750, 3739, 3727, 3716, 3704, 3693, 3681, 3669, 3658, 3646, 3634, 3622, 3610, 3598, 3586, 3574, 3562, 3550, 3538, 3525, 3513, 3501, 3488, 3476, 3464, 3451, 3439, 3426, 3413, 3401, 3388, 3376, 3363, 3350, 3337, 3325, 3312, 3299, 3286, 3273, 3260, 3248, 3235, 3222, 3209, 3196, 3183, 3170, 3157, 3144, 3131, 3118, 3105, 3092, 3079, 3065, 3052, 3039, 3026, 3013, 3000, 2987, 2974, 2961, 2948, 2935, 2921, 2908, 2895, 2882, 2869, 2856, 2843, 2830, 2817, 2804, 2791, 2778, 2765, 2752, 2740, 2727, 2714, 2701, 2688, 2675, 2663, 2650, 2637, 2624, 2612, 2599, 2587, 2574, 2561, 2549, 2536, 2524, 2512, 2499, 2487, 2475, 2462, 2450, 2438, 2426, 2414, 2402, 2390, 2378, 2366, 2354, 2342, 2331, 2319, 2307, 2296, 2284, 2273, 2261, 2250, 2239, 2227, 2216, 2205, 2194, 2183, 2172, 2161, 2150, 2140, 2129, 2118, 2108, 2097, 2087, 2077, 2066, 2056, 2046, 2036, 2026, 2016, 2006, 1996, 1987, 1977, 1967, 1958, 1949, 1939, 1930, 1921, 1912, 1903, 1894, 1885, 1877, 1868, 1859, 1851, 1843, 1834, 1826, 1818, 1810, 1802, 1794, 1786, 1779, 1771, 1764, 1756, 1749, 1742, 1735, 1728, 1721, 1714, 1708, 1701, 1694, 1688, 1682, 1676, 1669, 1663, 1658, 1652, 1646, 1641, 1635, 1630, 1624, 1619, 1614, 1609, 1604, 1600, 1595, 1590, 1586, 1582, 1578, 1573, 1569, 1566, 1562, 1558, 1555, 1551, 1548, 1545, 1541, 1538, 1536, 1533, 1530, 1528, 1525, 1523, 1521, 1518, 1516, 1515, 1513, 1511, 1510, 1508, 1507, 1506, 1505, 1504, 1503, 1502, 1501, 1501, 1501, 1500, 1500, 1500, 1500, 1500, 1501, 1501, 1501, 1502, 1503, 1504, 1505, 1506, 1507, 1508, 1510, 1511, 1513, 1515, 1516, 1518, 1521, 1523, 1525, 1528, 1530, 1533, 1536, 1538, 1541, 1545, 1548, 1551, 1555, 1558, 1562, 1566, 1569, 1573, 1578, 1582, 1586, 1590, 1595, 1600, 1604, 1609, 1614, 1619, 1624, 1630, 1635, 1641, 1646, 1652, 1658, 1663, 1669, 1676, 1682, 1688, 1694, 1701, 1708, 1714, 1721, 1728, 1735, 1742, 1749, 1756, 1764, 1771, 1779, 1786, 1794, 1802, 1810, 1818, 1826, 1834, 1843, 1851, 1859, 1868, 1877, 1885, 1894, 1903, 1912, 1921, 1930, 1939, 1949, 1958, 1967, 1977, 1987, 1996, 2006, 2016, 2026, 2036, 2046, 2056, 2066, 2077, 2087, 2097, 2108, 2118, 2129, 2140, 2150, 2161, 2172, 2183, 2194, 2205, 2216, 2227, 2239, 2250, 2261, 2273, 2284, 2296, 2307, 2319, 2331, 2342, 2354, 2366, 2378, 2390, 2402, 2414, 2426, 2438, 2450, 2462, 2475, 2487, 2499, 2512, 2524, 2536, 2549, 2561, 2574, 2587, 2599, 2612, 2624, 2637, 2650, 2663, 2675, 2688, 2701, 2714, 2727, 2740, 2752, 2765, 2778, 2791, 2804, 2817, 2830, 2843, 2856, 2869, 2882, 2895, 2908, 2921, 2935, 2948, 2961, 2974, 2987
};

static inline void drive_motor(uint32_t angle)
{
/*
  int sextant = angle / 6000000;
  int pwm = (angle % 6000000) / (6000000 / PWM_RESOLUTION);
  if (sextant == 0) {
    HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_U_PIN, 0);
    TIM3->CCR1 = 0;
    HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_V_PIN, 1);
    TIM3->CCR2 = progress;
    HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_W_PIN, 1);
    TIM3->CCR3 = 0;
  }
*/
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_U_PIN, 1);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_V_PIN, 1);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_W_PIN, 1);
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
  // APB1 = 16 MHz
  // Period = 16 MHz / 6000 = 2.67 kHz
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

/*
  HAL_Delay(2000);
  HAL_GPIO_WritePin(DRV_IN_U_PORT, DRV_IN_U_PIN, 0);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_U_PIN, 0);
  HAL_GPIO_WritePin(DRV_IN_V_PORT, DRV_IN_V_PIN, 1);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_V_PIN, 1);
  HAL_GPIO_WritePin(DRV_IN_W_PORT, DRV_IN_W_PIN, 0);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_W_PIN, 1);
  for (int i = 0; i < 100; i++) asm volatile ("nop");
  HAL_GPIO_WritePin(DRV_IN_U_PORT, DRV_IN_U_PIN, 0);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_U_PIN, 0);
  HAL_GPIO_WritePin(DRV_IN_V_PORT, DRV_IN_V_PIN, 0);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_V_PIN, 0);
  HAL_GPIO_WritePin(DRV_IN_W_PORT, DRV_IN_W_PIN, 0);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_W_PIN, 0);
  HAL_Delay(2000);

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
*/

if (1) {
/*
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_U_PIN, 1);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_V_PIN, 1);
  HAL_GPIO_WritePin(DRV_EN_PORT, DRV_EN_W_PIN, 1);
  while (1) {
    for (int i = 0; i < 720; i++) {
      TIM3->CCR1 = sin_lookup[0][(i +   0) % 720];
      TIM3->CCR2 = sin_lookup[0][(i + 240) % 720];
      TIM3->CCR3 = sin_lookup[0][(i + 480) % 720];
      for (int j = 0; j < 1000; j++) asm volatile ("nop");
    }
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 1);
    for (int i = 719; i >= 0; i--) {
      TIM3->CCR1 = sin_lookup[0][(i +   0) % 720];
      TIM3->CCR2 = sin_lookup[0][(i + 240) % 720];
      TIM3->CCR3 = sin_lookup[0][(i + 480) % 720];
      for (int j = 0; j < 1000; j++) asm volatile ("nop");
    }
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, 0);
    // static int parity = 1;
    // HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, parity ^= 1);
  }
  while (1) {
    for (int max = 9000000; max <= 36000000; max += 1000000) {
      int i = 0;
      for (; i < max; i += 2500) {
        drive_motor(i);
        for (int j = 0; j < 300; j++) asm volatile ("nop");
      }
      for (; i > 0; i -= 2500) {
        drive_motor(i);
        for (int j = 0; j < 300; j++) asm volatile ("nop");
      }
      static int parity = 1;
      HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, parity ^= 1);
    }
  }
*/
  while (1) {
    for (int i = 0; i < 36000000; i += 10000) {
      drive_motor(i);
      // HAL_Delay(1);
      for (int j = 0; j < 100; j++) asm volatile ("nop");
    }
    static int parity = 1;
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, parity ^= 1);
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

  // if (HAL_GetTick() % 500 == 0)
  //   HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, HAL_GetTick() % 1000 == 0);
}
