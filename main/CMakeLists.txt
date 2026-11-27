idf_component_register(
    SRCS "main.cpp" "optical_link.cpp" "rmt_loopback.cpp"
         "I2cBus.cpp"
  INCLUDE_DIRS "."
  PRIV_REQUIRES spi_flash
                esp_driver_rmt
                esp_driver_spi
                esp_driver_i2c
                esp_driver_gpio
                vfs
                lwip
                mcp4725
                galvo_system
                lsm6ds3
                esp_driver_usb_serial_jtag)
