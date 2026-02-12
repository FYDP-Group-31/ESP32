#pragma once

#include "driver/gpio.h"

#define I2S_MCLK_GPIO        GPIO_NUM_1
#define I2S_BCLK_GPIO        GPIO_NUM_2
#define I2S_WS_GPIO          GPIO_NUM_3
#define I2S_DOUT_GPIO        GPIO_NUM_4

#define AUDIO_BUF_FULL_GPIO  GPIO_NUM_5
// GPIO_NUM_6
#define DAC_RST_GPIO         GPIO_NUM_22

#define UART0_TX_GPIO        GPIO_NUM_37
#define UART0_RX_GPIO        GPIO_NUM_38
#define UART1_TX_GPIO        GPIO_NUM_20
#define UART1_RX_GPIO        GPIO_NUM_21