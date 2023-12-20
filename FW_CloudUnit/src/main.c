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

#define PWM_RESOLUTION 2000
// angle: [0, 36000000)
/*
from math import *
N=720
print(', '.join('%d' % round(2000*(1+0.2*sin(i/N*2*pi))/2) for i in range(N)))
*/
const uint16_t sin_lookup[720] = {
1000, 1002, 1003, 1005, 1007, 1009, 1010, 1012, 1014, 1016, 1017, 1019, 1021, 1023, 1024, 1026, 1028, 1030, 1031, 1033, 1035, 1036, 1038, 1040, 1042, 1043, 1045, 1047, 1048, 1050, 1052, 1053, 1055, 1057, 1058, 1060, 1062, 1063, 1065, 1067, 1068, 1070, 1072, 1073, 1075, 1077, 1078, 1080, 1081, 1083, 1085, 1086, 1088, 1089, 1091, 1092, 1094, 1095, 1097, 1098, 1100, 1102, 1103, 1104, 1106, 1107, 1109, 1110, 1112, 1113, 1115, 1116, 1118, 1119, 1120, 1122, 1123, 1125, 1126, 1127, 1129, 1130, 1131, 1133, 1134, 1135, 1136, 1138, 1139, 1140, 1141, 1143, 1144, 1145, 1146, 1147, 1149, 1150, 1151, 1152, 1153, 1154, 1155, 1157, 1158, 1159, 1160, 1161, 1162, 1163, 1164, 1165, 1166, 1167, 1168, 1169, 1170, 1171, 1171, 1172, 1173, 1174, 1175, 1176, 1177, 1177, 1178, 1179, 1180, 1181, 1181, 1182, 1183, 1183, 1184, 1185, 1185, 1186, 1187, 1187, 1188, 1189, 1189, 1190, 1190, 1191, 1191, 1192, 1192, 1193, 1193, 1194, 1194, 1194, 1195, 1195, 1196, 1196, 1196, 1197, 1197, 1197, 1198, 1198, 1198, 1198, 1199, 1199, 1199, 1199, 1199, 1199, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200, 1199, 1199, 1199, 1199, 1199, 1199, 1198, 1198, 1198, 1198, 1197, 1197, 1197, 1196, 1196, 1196, 1195, 1195, 1194, 1194, 1194, 1193, 1193, 1192, 1192, 1191, 1191, 1190, 1190, 1189, 1189, 1188, 1187, 1187, 1186, 1185, 1185, 1184, 1183, 1183, 1182, 1181, 1181, 1180, 1179, 1178, 1177, 1177, 1176, 1175, 1174, 1173, 1172, 1171, 1171, 1170, 1169, 1168, 1167, 1166, 1165, 1164, 1163, 1162, 1161, 1160, 1159, 1158, 1157, 1155, 1154, 1153, 1152, 1151, 1150, 1149, 1147, 1146, 1145, 1144, 1143, 1141, 1140, 1139, 1138, 1136, 1135, 1134, 1133, 1131, 1130, 1129, 1127, 1126, 1125, 1123, 1122, 1120, 1119, 1118, 1116, 1115, 1113, 1112, 1110, 1109, 1107, 1106, 1104, 1103, 1102, 1100, 1098, 1097, 1095, 1094, 1092, 1091, 1089, 1088, 1086, 1085, 1083, 1081, 1080, 1078, 1077, 1075, 1073, 1072, 1070, 1068, 1067, 1065, 1063, 1062, 1060, 1058, 1057, 1055, 1053, 1052, 1050, 1048, 1047, 1045, 1043, 1042, 1040, 1038, 1036, 1035, 1033, 1031, 1030, 1028, 1026, 1024, 1023, 1021, 1019, 1017, 1016, 1014, 1012, 1010, 1009, 1007, 1005, 1003, 1002, 1000, 998, 997, 995, 993, 991, 990, 988, 986, 984, 983, 981, 979, 977, 976, 974, 972, 970, 969, 967, 965, 964, 962, 960, 958, 957, 955, 953, 952, 950, 948, 947, 945, 943, 942, 940, 938, 937, 935, 933, 932, 930, 928, 927, 925, 923, 922, 920, 919, 917, 915, 914, 912, 911, 909, 908, 906, 905, 903, 902, 900, 898, 897, 896, 894, 893, 891, 890, 888, 887, 885, 884, 882, 881, 880, 878, 877, 875, 874, 873, 871, 870, 869, 867, 866, 865, 864, 862, 861, 860, 859, 857, 856, 855, 854, 853, 851, 850, 849, 848, 847, 846, 845, 843, 842, 841, 840, 839, 838, 837, 836, 835, 834, 833, 832, 831, 830, 829, 829, 828, 827, 826, 825, 824, 823, 823, 822, 821, 820, 819, 819, 818, 817, 817, 816, 815, 815, 814, 813, 813, 812, 811, 811, 810, 810, 809, 809, 808, 808, 807, 807, 806, 806, 806, 805, 805, 804, 804, 804, 803, 803, 803, 802, 802, 802, 802, 801, 801, 801, 801, 801, 801, 800, 800, 800, 800, 800, 800, 800, 800, 800, 800, 800, 800, 800, 800, 800, 800, 800, 801, 801, 801, 801, 801, 801, 802, 802, 802, 802, 803, 803, 803, 804, 804, 804, 805, 805, 806, 806, 806, 807, 807, 808, 808, 809, 809, 810, 810, 811, 811, 812, 813, 813, 814, 815, 815, 816, 817, 817, 818, 819, 819, 820, 821, 822, 823, 823, 824, 825, 826, 827, 828, 829, 829, 830, 831, 832, 833, 834, 835, 836, 837, 838, 839, 840, 841, 842, 843, 845, 846, 847, 848, 849, 850, 851, 853, 854, 855, 856, 857, 859, 860, 861, 862, 864, 865, 866, 867, 869, 870, 871, 873, 874, 875, 877, 878, 880, 881, 882, 884, 885, 887, 888, 890, 891, 893, 894, 896, 897, 898, 900, 902, 903, 905, 906, 908, 909, 911, 912, 914, 915, 917, 919, 920, 922, 923, 925, 927, 928, 930, 932, 933, 935, 937, 938, 940, 942, 943, 945, 947, 948, 950, 952, 953, 955, 957, 958, 960, 962, 964, 965, 967, 969, 970, 972, 974, 976, 977, 979, 981, 983, 984, 986, 988, 990, 991, 993, 995, 997, 998
};

static inline void drive_motor(uint32_t angle)
{
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
  // Period = 64 MHz / 2000 = 32 kHz
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

if (1) {
  while (1) {
    for (int i = 0; i < 3600; i += 3) {
      // angle normalized into [0, 36000000)
      // int angle = i * 10000;
      int angle = (int)(0.5f + 72000000 + 40000000 * sinf((float)i / 3600 * 6.2831853f));
      drive_motor(angle);
      // for (int j = 0; j < 100; j++) asm volatile ("nop");
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
