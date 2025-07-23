#ifndef __S3_CONFIG_H__
#define __S3_CONFIG_H__

#include "driver/gpio.h"

#define DEBUG_MODE              1
#define DEBUG_TIMER             1
#define DEBUG_LED               0
#define DEBUG_I2S               1
#define DEBUG_UART              1

// Common GPIO mappings
#define I2S_BCLK_GPIO           GPIO_NUM_4
#define I2S_WS_GPIO             GPIO_NUM_5
#define I2S_DATA_GPIO           GPIO_NUM_6
#define I2S_MCLK_GPIO           GPIO_NUM_7
#define UART_RX_GPIO            GPIO_NUM_16
#define UART_TX_GPIO            GPIO_NUM_17

// Slave GPIO mappings
#define SLAVE_WRITE_READY_GPIO  GPIO_NUM_14

// Master GPIO mappings
#define MASTER_WRITE_READY_GPIO GPIO_NUM_15

// UART configs
#define UART_BAUD_RATE          2000000

// I2S configs
#define I2S_MODE_STANDARD       0
#define I2S_MODE_TDM            1
#define OUTPUT_MODE             (I2S_MODE_TDM)

#endif