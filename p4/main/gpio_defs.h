#ifndef _GPIO_DEFS_H_
#define _GPIO_DEFS_H_

#include "driver/gpio.h"

#define I2S_MCLK_GPIO       GPIO_NUM_1
#define I2S_BCLK_GPIO       GPIO_NUM_2
#define I2S_WS_GPIO         GPIO_NUM_3
#define I2S_DOUT_GPIO       GPIO_NUM_4

#define MAX98357_EN_GPIO    GPIO_NUM_5
#define MAX98357_BCLK_GPIO  GPIO_NUM_21
#define MAX98357_WS_GPIO    GPIO_NUM_22
#define MAX98357_DATA_GPIO  GPIO_NUM_23

#define MAX98357_ENABLE 1
#define MAX98357_DISABLE 0

#endif // _GPIO_DEFS_H_
