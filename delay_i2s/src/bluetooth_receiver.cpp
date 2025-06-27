#include "bluetooth_receiver.hpp"
#include "esp_a2dp_api.h"
#include "esp_bt_main.h"
#include "nvs_flash.h"
#include "esp_check.h"

BluetoothReceiver bt_receiver;

// https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/classic_bt/a2dp_sink/README.md
// https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/storage/nvs_flash.html

BluetoothReceiver::BluetoothReceiver()
{

}

bool BluetoothReceiver::init(void)
{
    // // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if ((err == ESP_ERR_NVS_NO_FREE_PAGES) || (err == ESP_ERR_NVS_NEW_VERSION_FOUND))
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
        ESP_ERROR_CHECK(err);
    }

    // if (esp_bluedroid_init() != ESP_OK)
    // {
    //     return false;
    // }
    return true;
}
