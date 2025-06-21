idf_component_register(
    SRCS "audio_dsp.c" "adau1966a_driver.cpp" "esp_it.c"
    PRIV_REQUIRES esp_driver_i2s esp_driver_gpio
    INCLUDE_DIRS "../inc" "."
)
