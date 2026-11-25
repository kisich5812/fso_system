idf_component_register(
  SRCS "lsm_main.c" 
  INCLUDE_DIRS "."
  PRIV_REQUIRES spi_flash
                esp_driver_rmt
                esp_driver_i2c
                esp_driver_gpio
                vfs
                lwip
                mcp4725
                lsm6ds3
                esp_driver_usb_serial_jtag)
