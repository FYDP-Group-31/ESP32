idf_component_register(
    SRCS
        # "hello_world_main.c"
        # "main.c"
        "test_esp_lcd_jd9365.c"
    PRIV_REQUIRES
        "spi_flash"
        "esp_timer"
        "unity"
        "esp_lcd"
    INCLUDE_DIRS
        ""
)
