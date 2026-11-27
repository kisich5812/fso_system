idf_component_register(
    SRCS "main.cpp" 
         "rmt_loopback.cpp"
  INCLUDE_DIRS "."
  PRIV_REQUIRES spi_flash
                mcp4725
                galvo_system
                optical_link
                i2c_bus
                lsm6ds3)
