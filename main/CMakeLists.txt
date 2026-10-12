idf_component_register(
  SRCS "main.c"
  INCLUDE_DIRS "."
  PRIV_REQUIRES spi_flash
                esp_driver_i2c
                esp_driver_gpio
                mcp4725)
