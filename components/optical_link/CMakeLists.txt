idf_component_register(
    SRCS "optical_link.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_gpio 
             esp_driver_rmt 
             freertos
    PRIV_REQUIRES log
)
