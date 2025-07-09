idf_component_register(
    SRCS "esp_it.c"
    REQUIRES esp_driver_i2s
    PRIV_REQUIRES esp_driver_gpio esp_ringbuf bt nvs_flash
    INCLUDE_DIRS "../inc" "."
)