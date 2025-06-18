idf_component_register(
    SRCS "esp_it.c" "i2s_driver.cpp"
    PRIV_REQUIRES esp_driver_i2s
    INCLUDE_DIRS "../inc" "."
)
