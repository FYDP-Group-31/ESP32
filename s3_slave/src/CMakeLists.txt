idf_component_register(
    SRCS "master_comm.cpp"
    REQUIRES
    PRIV_REQUIRES
        "s3_common"
        "esp_driver_gpio"
    INCLUDE_DIRS
        "../inc"
)