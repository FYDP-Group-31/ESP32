idf_component_register(
    SRCS "main.cpp"
    REQUIRES src
    PRIV_REQUIRES esp_driver_gpio esp_ringbuf
    INCLUDE_DIRS "."
)
