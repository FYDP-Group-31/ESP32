idf_component_register(
    SRCS "main.cpp"
    PRIV_REQUIRES esp_driver_gpio esp_driver_i2s bt
    INCLUDE_DIRS "."
    REQUIRES src
)
