#include "master_comm.h"
#include "s3_config.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "driver/gpio.h"

volatile bool master_ready_status = false;

static void IRAM_ATTR master_ready_isr(void* args)
{
    int gpio_level = gpio_get_level((gpio_num_t)((int)args));
    if (gpio_level == 0)
    {
        // Falling edge
        master_ready_status = false;
    }
    else
    {
        // Rising edge
        master_ready_status = true;
    }
}

void master_comm_init(void)
{
    const gpio_config_t input_gpio = {
        .pin_bit_mask = ((1ULL << MASTER_WRITE_READY_GPIO) | (1ULL << I2S_WRITE_START_GPIO)),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_gpio));

    const gpio_config_t output_gpio = {
        .pin_bit_mask = (1ULL << SLAVE_WRITE_READY_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&output_gpio));
    // ESP_ERROR_CHECK(gpio_set_drive_capability(SLAVE_WRITE_READY_GPIO, GPIO_DRIVE_CAP_3));
    ESP_ERROR_CHECK(gpio_set_level(SLAVE_WRITE_READY_GPIO, 0UL));

    // Check initial status of master
    master_ready_status = (bool)gpio_get_level(MASTER_WRITE_READY_GPIO);
    ESP_LOGI("Master comms", "master_ready_status init to %u", master_ready_status);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MASTER_WRITE_READY_GPIO, master_ready_isr, (void*)SLAVE_WRITE_READY_GPIO));
    ESP_LOGI("Master comms", "Initialized");
}

void slave_set_ready(slave_ready_t ready)
{
    gpio_set_level(SLAVE_WRITE_READY_GPIO, ready);
    printf("slave: %u\n", ready);
}


