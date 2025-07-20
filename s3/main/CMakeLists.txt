idf_component_register(
    SRCS "main.cpp"
    REQUIRES src
    PRIV_REQUIRES esp_driver_gpio esp_ringbuf esp_driver_i2s esp_timer
    INCLUDE_DIRS "."
)
