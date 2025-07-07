#include "driver/gpio.h"

#define UNUSED_GPIO_0           GPIO_NUM_0 // Connected to BOOT button, pulled HIGH by default
#define RESERVED_GPIO_1         GPIO_NUM_1 // TX pin for flashing/debugging
#define GPIO_LED_PIN            GPIO_NUM_2
#define RESERVED_GPIO_3         GPIO_NUM_3 // RX pin for flashing/debugging
#define GPIO_I2S_TX_BCLK        GPIO_NUM_4
#define GPIO_I2S_TX_WS_PIN      GPIO_NUM_5 // Must be HIGH during boot
#define RESERVED_GPIO_6         GPIO_NUM_6 // Flash memory
#define RESERVED_GPIO_7         GPIO_NUM_7 // Flash memory
#define RESERVED_GPIO_8         GPIO_NUM_8 // Flash memory
#define RESERVED_GPIO_9         GPIO_NUM_9 // Flash memory
#define RESERVED_GPIO_10        GPIO_NUM_10 // Flash memory
#define RESERVED_GPIO_11        GPIO_NUM_11 // Flash memory
#define UNUSED_GPIO_12          GPIO_NUM_12 // Must be LOW during boot
#define UNUSED_GPIO_13          GPIO_NUM_13
#define UNUSED_GPIO_14          GPIO_NUM_14
#define UNUSED_GPIO_15          GPIO_NUM_15 // Must be HIGH during boot
#define UNUSED_GPIO_16          GPIO_NUM_16 // RX pin for UART2
#define UNUSED_GPIO_17          GPIO_NUM_17 // TX pin for UART2
#define GPIO_I2S_TX_DATA_OUT    GPIO_NUM_18
#define UNUSED_GPIO_19          GPIO_NUM_19
#define RESERVED_GPIO_20        GPIO_NUM_20 // Not exposed
#define GPIO_I2C_SDA            GPIO_NUM_21
#define GPIO_I2C_SCL            GPIO_NUM_22
#define UNUSED_GPIO_23          GPIO_NUM_23
#define RESERVED_GPIO_24        GPIO_NUM_24 // Not exposed
#define UNUSED_GPIO_25          GPIO_NUM_25
#define UNUSED_GPIO_26          GPIO_NUM_26
#define UNUSED_GPIO_27          GPIO_NUM_27
#define RESERVED_GPIO_28        GPIO_NUM_28 // Not exposed
#define RESERVED_GPIO_29        GPIO_NUM_29 // Not exposed
#define RESERVED_GPIO_30        GPIO_NUM_30 // Not exposed
#define RESERVED_GPIO_31        GPIO_NUM_31 // Not exposed
#define UNUSED_GPIO_32          GPIO_NUM_32
#define UNUSED_GPIO_33          GPIO_NUM_33
#define UNUSED_GPIO_34          GPIO_NUM_34 // Input only
#define UNUSED_GPIO_35          GPIO_NUM_35 // Input only
#define UNUSED_GPIO_36          GPIO_NUM_36 // Input only
#define RESERVED_GPIO_37        GPIO_NUM_37 // Not exposed
#define RESERVED_GPIO_38        GPIO_NUM_38 // Not exposed
#define UNUSED_GPIO_39          GPIO_NUM_39 // Input only
