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
  print('{' + ', '.join('%d' % round(6000*(1+sin(i/N*2*pi + phase/3*2*pi))/2) for i in range(N)) + '},')
*/
const uint16_t sin_lookup[3][720] = {
{3000, 3026, 3052, 3079, 3105, 3131, 3157, 3183, 3209, 3235, 3261, 3288, 3314, 3340, 3366, 3392, 3418, 3443, 3469, 3495, 3521, 3547, 3572, 3598, 3624, 3649, 3675, 3700, 3726, 3751, 3776, 3802, 3827, 3852, 3877, 3902, 3927, 3952, 3977, 4001, 4026, 4051, 4075, 4100, 4124, 4148, 4172, 4196, 4220, 4244, 4268, 4292, 4315, 4339, 4362, 4385, 4408, 4431, 4454, 4477, 4500, 4523, 4545, 4567, 4590, 4612, 4634, 4656, 4678, 4699, 4721, 4742, 4763, 4784, 4805, 4826, 4847, 4868, 4888, 4908, 4928, 4948, 4968, 4988, 5007, 5027, 5046, 5065, 5084, 5103, 5121, 5140, 5158, 5176, 5194, 5212, 5229, 5247, 5264, 5281, 5298, 5315, 5331, 5348, 5364, 5380, 5396, 5412, 5427, 5442, 5457, 5472, 5487, 5502, 5516, 5530, 5544, 5558, 5572, 5585, 5598, 5611, 5624, 5636, 5649, 5661, 5673, 5685, 5696, 5708, 5719, 5730, 5741, 5751, 5762, 5772, 5782, 5791, 5801, 5810, 5819, 5828, 5837, 5845, 5853, 5861, 5869, 5876, 5884, 5891, 5898, 5904, 5911, 5917, 5923, 5929, 5934, 5940, 5945, 5950, 5954, 5959, 5963, 5967, 5971, 5974, 5978, 5981, 5984, 5986, 5989, 5991, 5993, 5994, 5996, 5997, 5998, 5999, 6000, 6000, 6000, 6000, 6000, 5999, 5998, 5997, 5996, 5994, 5993, 5991, 5989, 5986, 5984, 5981, 5978, 5974, 5971, 5967, 5963, 5959, 5954, 5950, 5945, 5940, 5934, 5929, 5923, 5917, 5911, 5904, 5898, 5891, 5884, 5876, 5869, 5861, 5853, 5845, 5837, 5828, 5819, 5810, 5801, 5791, 5782, 5772, 5762, 5751, 5741, 5730, 5719, 5708, 5696, 5685, 5673, 5661, 5649, 5636, 5624, 5611, 5598, 5585, 5572, 5558, 5544, 5530, 5516, 5502, 5487, 5472, 5457, 5442, 5427, 5412, 5396, 5380, 5364, 5348, 5331, 5315, 5298, 5281, 5264, 5247, 5229, 5212, 5194, 5176, 5158, 5140, 5121, 5103, 5084, 5065, 5046, 5027, 5007, 4988, 4968, 4948, 4928, 4908, 4888, 4868, 4847, 4826, 4805, 4784, 4763, 4742, 4721, 4699, 4678, 4656, 4634, 4612, 4590, 4567, 4545, 4523, 4500, 4477, 4454, 4431, 4408, 4385, 4362, 4339, 4315, 4292, 4268, 4244, 4220, 4196, 4172, 4148, 4124, 4100, 4075, 4051, 4026, 4001, 3977, 3952, 3927, 3902, 3877, 3852, 3827, 3802, 3776, 3751, 3726, 3700, 3675, 3649, 3624, 3598, 3572, 3547, 3521, 3495, 3469, 3443, 3418, 3392, 3366, 3340, 3314, 3288, 3261, 3235, 3209, 3183, 3157, 3131, 3105, 3079, 3052, 3026, 3000, 2974, 2948, 2921, 2895, 2869, 2843, 2817, 2791, 2765, 2739, 2712, 2686, 2660, 2634, 2608, 2582, 2557, 2531, 2505, 2479, 2453, 2428, 2402, 2376, 2351, 2325, 2300, 2274, 2249, 2224, 2198, 2173, 2148, 2123, 2098, 2073, 2048, 2023, 1999, 1974, 1949, 1925, 1900, 1876, 1852, 1828, 1804, 1780, 1756, 1732, 1708, 1685, 1661, 1638, 1615, 1592, 1569, 1546, 1523, 1500, 1477, 1455, 1433, 1410, 1388, 1366, 1344, 1322, 1301, 1279, 1258, 1237, 1216, 1195, 1174, 1153, 1132, 1112, 1092, 1072, 1052, 1032, 1012, 993, 973, 954, 935, 916, 897, 879, 860, 842, 824, 806, 788, 771, 753, 736, 719, 702, 685, 669, 652, 636, 620, 604, 588, 573, 558, 543, 528, 513, 498, 484, 470, 456, 442, 428, 415, 402, 389, 376, 364, 351, 339, 327, 315, 304, 292, 281, 270, 259, 249, 238, 228, 218, 209, 199, 190, 181, 172, 163, 155, 147, 139, 131, 124, 116, 109, 102, 96, 89, 83, 77, 71, 66, 60, 55, 50, 46, 41, 37, 33, 29, 26, 22, 19, 16, 14, 11, 9, 7, 6, 4, 3, 2, 1, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 9, 11, 14, 16, 19, 22, 26, 29, 33, 37, 41, 46, 50, 55, 60, 66, 71, 77, 83, 89, 96, 102, 109, 116, 124, 131, 139, 147, 155, 163, 172, 181, 190, 199, 209, 218, 228, 238, 249, 259, 270, 281, 292, 304, 315, 327, 339, 351, 364, 376, 389, 402, 415, 428, 442, 456, 470, 484, 498, 513, 528, 543, 558, 573, 588, 604, 620, 636, 652, 669, 685, 702, 719, 736, 753, 771, 788, 806, 824, 842, 860, 879, 897, 916, 935, 954, 973, 993, 1012, 1032, 1052, 1072, 1092, 1112, 1132, 1153, 1174, 1195, 1216, 1237, 1258, 1279, 1301, 1322, 1344, 1366, 1388, 1410, 1433, 1455, 1477, 1500, 1523, 1546, 1569, 1592, 1615, 1638, 1661, 1685, 1708, 1732, 1756, 1780, 1804, 1828, 1852, 1876, 1900, 1925, 1949, 1974, 1999, 2023, 2048, 2073, 2098, 2123, 2148, 2173, 2198, 2224, 2249, 2274, 2300, 2325, 2351, 2376, 2402, 2428, 2453, 2479, 2505, 2531, 2557, 2582, 2608, 2634, 2660, 2686, 2712, 2739, 2765, 2791, 2817, 2843, 2869, 2895, 2921, 2948, 2974},
{5598, 5585, 5572, 5558, 5544, 5530, 5516, 5502, 5487, 5472, 5457, 5442, 5427, 5412, 5396, 5380, 5364, 5348, 5331, 5315, 5298, 5281, 5264, 5247, 5229, 5212, 5194, 5176, 5158, 5140, 5121, 5103, 5084, 5065, 5046, 5027, 5007, 4988, 4968, 4948, 4928, 4908, 4888, 4868, 4847, 4826, 4805, 4784, 4763, 4742, 4721, 4699, 4678, 4656, 4634, 4612, 4590, 4567, 4545, 4523, 4500, 4477, 4454, 4431, 4408, 4385, 4362, 4339, 4315, 4292, 4268, 4244, 4220, 4196, 4172, 4148, 4124, 4100, 4075, 4051, 4026, 4001, 3977, 3952, 3927, 3902, 3877, 3852, 3827, 3802, 3776, 3751, 3726, 3700, 3675, 3649, 3624, 3598, 3572, 3547, 3521, 3495, 3469, 3443, 3418, 3392, 3366, 3340, 3314, 3288, 3261, 3235, 3209, 3183, 3157, 3131, 3105, 3079, 3052, 3026, 3000, 2974, 2948, 2921, 2895, 2869, 2843, 2817, 2791, 2765, 2739, 2712, 2686, 2660, 2634, 2608, 2582, 2557, 2531, 2505, 2479, 2453, 2428, 2402, 2376, 2351, 2325, 2300, 2274, 2249, 2224, 2198, 2173, 2148, 2123, 2098, 2073, 2048, 2023, 1999, 1974, 1949, 1925, 1900, 1876, 1852, 1828, 1804, 1780, 1756, 1732, 1708, 1685, 1661, 1638, 1615, 1592, 1569, 1546, 1523, 1500, 1477, 1455, 1433, 1410, 1388, 1366, 1344, 1322, 1301, 1279, 1258, 1237, 1216, 1195, 1174, 1153, 1132, 1112, 1092, 1072, 1052, 1032, 1012, 993, 973, 954, 935, 916, 897, 879, 860, 842, 824, 806, 788, 771, 753, 736, 719, 702, 685, 669, 652, 636, 620, 604, 588, 573, 558, 543, 528, 513, 498, 484, 470, 456, 442, 428, 415, 402, 389, 376, 364, 351, 339, 327, 315, 304, 292, 281, 270, 259, 249, 238, 228, 218, 209, 199, 190, 181, 172, 163, 155, 147, 139, 131, 124, 116, 109, 102, 96, 89, 83, 77, 71, 66, 60, 55, 50, 46, 41, 37, 33, 29, 26, 22, 19, 16, 14, 11, 9, 7, 6, 4, 3, 2, 1, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 9, 11, 14, 16, 19, 22, 26, 29, 33, 37, 41, 46, 50, 55, 60, 66, 71, 77, 83, 89, 96, 102, 109, 116, 124, 131, 139, 147, 155, 163, 172, 181, 190, 199, 209, 218, 228, 238, 249, 259, 270, 281, 292, 304, 315, 327, 339, 351, 364, 376, 389, 402, 415, 428, 442, 456, 470, 484, 498, 513, 528, 543, 558, 573, 588, 604, 620, 636, 652, 669, 685, 702, 719, 736, 753, 771, 788, 806, 824, 842, 860, 879, 897, 916, 935, 954, 973, 993, 1012, 1032, 1052, 1072, 1092, 1112, 1132, 1153, 1174, 1195, 1216, 1237, 1258, 1279, 1301, 1322, 1344, 1366, 1388, 1410, 1433, 1455, 1477, 1500, 1523, 1546, 1569, 1592, 1615, 1638, 1661, 1685, 1708, 1732, 1756, 1780, 1804, 1828, 1852, 1876, 1900, 1925, 1949, 1974, 1999, 2023, 2048, 2073, 2098, 2123, 2148, 2173, 2198, 2224, 2249, 2274, 2300, 2325, 2351, 2376, 2402, 2428, 2453, 2479, 2505, 2531, 2557, 2582, 2608, 2634, 2660, 2686, 2712, 2739, 2765, 2791, 2817, 2843, 2869, 2895, 2921, 2948, 2974, 3000, 3026, 3052, 3079, 3105, 3131, 3157, 3183, 3209, 3235, 3261, 3288, 3314, 3340, 3366, 3392, 3418, 3443, 3469, 3495, 3521, 3547, 3572, 3598, 3624, 3649, 3675, 3700, 3726, 3751, 3776, 3802, 3827, 3852, 3877, 3902, 3927, 3952, 3977, 4001, 4026, 4051, 4075, 4100, 4124, 4148, 4172, 4196, 4220, 4244, 4268, 4292, 4315, 4339, 4362, 4385, 4408, 4431, 4454, 4477, 4500, 4523, 4545, 4567, 4590, 4612, 4634, 4656, 4678, 4699, 4721, 4742, 4763, 4784, 4805, 4826, 4847, 4868, 4888, 4908, 4928, 4948, 4968, 4988, 5007, 5027, 5046, 5065, 5084, 5103, 5121, 5140, 5158, 5176, 5194, 5212, 5229, 5247, 5264, 5281, 5298, 5315, 5331, 5348, 5364, 5380, 5396, 5412, 5427, 5442, 5457, 5472, 5487, 5502, 5516, 5530, 5544, 5558, 5572, 5585, 5598, 5611, 5624, 5636, 5649, 5661, 5673, 5685, 5696, 5708, 5719, 5730, 5741, 5751, 5762, 5772, 5782, 5791, 5801, 5810, 5819, 5828, 5837, 5845, 5853, 5861, 5869, 5876, 5884, 5891, 5898, 5904, 5911, 5917, 5923, 5929, 5934, 5940, 5945, 5950, 5954, 5959, 5963, 5967, 5971, 5974, 5978, 5981, 5984, 5986, 5989, 5991, 5993, 5994, 5996, 5997, 5998, 5999, 6000, 6000, 6000, 6000, 6000, 5999, 5998, 5997, 5996, 5994, 5993, 5991, 5989, 5986, 5984, 5981, 5978, 5974, 5971, 5967, 5963, 5959, 5954, 5950, 5945, 5940, 5934, 5929, 5923, 5917, 5911, 5904, 5898, 5891, 5884, 5876, 5869, 5861, 5853, 5845, 5837, 5828, 5819, 5810, 5801, 5791, 5782, 5772, 5762, 5751, 5741, 5730, 5719, 5708, 5696, 5685, 5673, 5661, 5649, 5636, 5624, 5611},
{402, 389, 376, 364, 351, 339, 327, 315, 304, 292, 281, 270, 259, 249, 238, 228, 218, 209, 199, 190, 181, 172, 163, 155, 147, 139, 131, 124, 116, 109, 102, 96, 89, 83, 77, 71, 66, 60, 55, 50, 46, 41, 37, 33, 29, 26, 22, 19, 16, 14, 11, 9, 7, 6, 4, 3, 2, 1, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 9, 11, 14, 16, 19, 22, 26, 29, 33, 37, 41, 46, 50, 55, 60, 66, 71, 77, 83, 89, 96, 102, 109, 116, 124, 131, 139, 147, 155, 163, 172, 181, 190, 199, 209, 218, 228, 238, 249, 259, 270, 281, 292, 304, 315, 327, 339, 351, 364, 376, 389, 402, 415, 428, 442, 456, 470, 484, 498, 513, 528, 543, 558, 573, 588, 604, 620, 636, 652, 669, 685, 702, 719, 736, 753, 771, 788, 806, 824, 842, 860, 879, 897, 916, 935, 954, 973, 993, 1012, 1032, 1052, 1072, 1092, 1112, 1132, 1153, 1174, 1195, 1216, 1237, 1258, 1279, 1301, 1322, 1344, 1366, 1388, 1410, 1433, 1455, 1477, 1500, 1523, 1546, 1569, 1592, 1615, 1638, 1661, 1685, 1708, 1732, 1756, 1780, 1804, 1828, 1852, 1876, 1900, 1925, 1949, 1974, 1999, 2023, 2048, 2073, 2098, 2123, 2148, 2173, 2198, 2224, 2249, 2274, 2300, 2325, 2351, 2376, 2402, 2428, 2453, 2479, 2505, 2531, 2557, 2582, 2608, 2634, 2660, 2686, 2712, 2739, 2765, 2791, 2817, 2843, 2869, 2895, 2921, 2948, 2974, 3000, 3026, 3052, 3079, 3105, 3131, 3157, 3183, 3209, 3235, 3261, 3288, 3314, 3340, 3366, 3392, 3418, 3443, 3469, 3495, 3521, 3547, 3572, 3598, 3624, 3649, 3675, 3700, 3726, 3751, 3776, 3802, 3827, 3852, 3877, 3902, 3927, 3952, 3977, 4001, 4026, 4051, 4075, 4100, 4124, 4148, 4172, 4196, 4220, 4244, 4268, 4292, 4315, 4339, 4362, 4385, 4408, 4431, 4454, 4477, 4500, 4523, 4545, 4567, 4590, 4612, 4634, 4656, 4678, 4699, 4721, 4742, 4763, 4784, 4805, 4826, 4847, 4868, 4888, 4908, 4928, 4948, 4968, 4988, 5007, 5027, 5046, 5065, 5084, 5103, 5121, 5140, 5158, 5176, 5194, 5212, 5229, 5247, 5264, 5281, 5298, 5315, 5331, 5348, 5364, 5380, 5396, 5412, 5427, 5442, 5457, 5472, 5487, 5502, 5516, 5530, 5544, 5558, 5572, 5585, 5598, 5611, 5624, 5636, 5649, 5661, 5673, 5685, 5696, 5708, 5719, 5730, 5741, 5751, 5762, 5772, 5782, 5791, 5801, 5810, 5819, 5828, 5837, 5845, 5853, 5861, 5869, 5876, 5884, 5891, 5898, 5904, 5911, 5917, 5923, 5929, 5934, 5940, 5945, 5950, 5954, 5959, 5963, 5967, 5971, 5974, 5978, 5981, 5984, 5986, 5989, 5991, 5993, 5994, 5996, 5997, 5998, 5999, 6000, 6000, 6000, 6000, 6000, 5999, 5998, 5997, 5996, 5994, 5993, 5991, 5989, 5986, 5984, 5981, 5978, 5974, 5971, 5967, 5963, 5959, 5954, 5950, 5945, 5940, 5934, 5929, 5923, 5917, 5911, 5904, 5898, 5891, 5884, 5876, 5869, 5861, 5853, 5845, 5837, 5828, 5819, 5810, 5801, 5791, 5782, 5772, 5762, 5751, 5741, 5730, 5719, 5708, 5696, 5685, 5673, 5661, 5649, 5636, 5624, 5611, 5598, 5585, 5572, 5558, 5544, 5530, 5516, 5502, 5487, 5472, 5457, 5442, 5427, 5412, 5396, 5380, 5364, 5348, 5331, 5315, 5298, 5281, 5264, 5247, 5229, 5212, 5194, 5176, 5158, 5140, 5121, 5103, 5084, 5065, 5046, 5027, 5007, 4988, 4968, 4948, 4928, 4908, 4888, 4868, 4847, 4826, 4805, 4784, 4763, 4742, 4721, 4699, 4678, 4656, 4634, 4612, 4590, 4567, 4545, 4523, 4500, 4477, 4454, 4431, 4408, 4385, 4362, 4339, 4315, 4292, 4268, 4244, 4220, 4196, 4172, 4148, 4124, 4100, 4075, 4051, 4026, 4001, 3977, 3952, 3927, 3902, 3877, 3852, 3827, 3802, 3776, 3751, 3726, 3700, 3675, 3649, 3624, 3598, 3572, 3547, 3521, 3495, 3469, 3443, 3418, 3392, 3366, 3340, 3314, 3288, 3261, 3235, 3209, 3183, 3157, 3131, 3105, 3079, 3052, 3026, 3000, 2974, 2948, 2921, 2895, 2869, 2843, 2817, 2791, 2765, 2739, 2712, 2686, 2660, 2634, 2608, 2582, 2557, 2531, 2505, 2479, 2453, 2428, 2402, 2376, 2351, 2325, 2300, 2274, 2249, 2224, 2198, 2173, 2148, 2123, 2098, 2073, 2048, 2023, 1999, 1974, 1949, 1925, 1900, 1876, 1852, 1828, 1804, 1780, 1756, 1732, 1708, 1685, 1661, 1638, 1615, 1592, 1569, 1546, 1523, 1500, 1477, 1455, 1433, 1410, 1388, 1366, 1344, 1322, 1301, 1279, 1258, 1237, 1216, 1195, 1174, 1153, 1132, 1112, 1092, 1072, 1052, 1032, 1012, 993, 973, 954, 935, 916, 897, 879, 860, 842, 824, 806, 788, 771, 753, 736, 719, 702, 685, 669, 652, 636, 620, 604, 588, 573, 558, 543, 528, 513, 498, 484, 470, 456, 442, 428, 415},
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
  TIM3->CCR1 = sin_lookup[0][angle / 50000] / 50;
  TIM3->CCR2 = sin_lookup[1][angle / 50000] / 50;
  TIM3->CCR3 = sin_lookup[2][angle / 50000] / 50;
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

if (0) {
  HAL_Delay(500);
  while (1) {
    for (int i = 0; i < 36000000; i += 10000) {
      drive_motor(i);
      // HAL_Delay(1);
      for (int j = 0; j < 300; j++) asm volatile ("nop");
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

  if (HAL_GetTick() % 500 == 0)
    HAL_GPIO_WritePin(LED_IND_ACT_PORT, LED_IND_ACT_PIN, HAL_GetTick() % 1000 == 0);
}
