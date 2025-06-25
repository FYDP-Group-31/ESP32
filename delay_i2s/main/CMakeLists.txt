idf_component_register(
    SRCS "main.cpp"
    PRIV_REQUIRES esp_driver_gpio esp_driver_i2s esp_ringbuf bt
    INCLUDE_DIRS "."
    REQUIRES src
)
