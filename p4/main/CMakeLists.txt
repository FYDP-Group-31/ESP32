idf_component_register(
    SRCS
        "main.cpp"
        # "i2s_audio.cpp"
        "adau1966a.cpp"
    PRIV_REQUIRES
        "esp_timer"
        "esp_driver_i2s"
        "esp_driver_gpio"
    INCLUDE_DIRS
        ""
)
