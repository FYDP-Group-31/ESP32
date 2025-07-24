#include "slave_comm.h"
#include "s3_config.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "driver/gpio.h"

volatile bool slave_ready_status = false;

static void IRAM_ATTR slave_ready_isr(void* args)
{
    int gpio_level = gpio_get_level((gpio_num_t)((int)args));
    if (gpio_level == 0)
    {
        // Falling edge
        slave_ready_status = false;
    }
    else
    {
        // Rising edge
        slave_ready_status = true;
    }
}

void slave_comm_init(void)
{
    const gpio_config_t output_gpio = {
        .pin_bit_mask = ((1ULL << I2S_WRITE_START_GPIO) | (1ULL << MASTER_WRITE_READY_GPIO)),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_gpio));
    // ESP_ERROR_CHECK(gpio_set_drive_capability(I2S_WRITE_START_GPIO, GPIO_DRIVE_CAP_3));
    ESP_ERROR_CHECK(gpio_set_level(I2S_WRITE_START_GPIO, 0UL));
    // ESP_ERROR_CHECK(gpio_set_drive_capability(MASTER_WRITE_READY_GPIO, GPIO_DRIVE_CAP_3));
    ESP_ERROR_CHECK(gpio_set_level(MASTER_WRITE_READY_GPIO, 0UL));

    const gpio_config_t input_gpio = {
        .pin_bit_mask = (1ULL << SLAVE_WRITE_READY_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_gpio));

    // Check initial status of slave
    slave_ready_status = (bool)gpio_get_level(SLAVE_WRITE_READY_GPIO);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(SLAVE_WRITE_READY_GPIO, slave_ready_isr, (void*)SLAVE_WRITE_READY_GPIO));
    ESP_LOGI("Slave comms", "Initialized");
}

void master_set_ready(master_ready_t ready)
{
    gpio_set_level(MASTER_WRITE_READY_GPIO, ready);
    printf("master: %u\n", ready);
}
