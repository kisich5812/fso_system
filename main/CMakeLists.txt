idf_component_register(SRCS "hello_world_main.c"
                       PRIV_REQUIRES spi_flash
                       INCLUDE_DIRS "~/.espressif/v6.1/esp-idf/components/esp_driver_ledc/include")
