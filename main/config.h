#pragma once

#include "hal/spi_types.h"
#include "soc/gpio_num.h"
#include <cstdint>

#ifdef OLD_MODULE

inline constexpr gpio_num_t TX_GPIO = GPIO_NUM_8; 
inline constexpr gpio_num_t RX_GPIO = GPIO_NUM_18; 

#else

inline constexpr gpio_num_t TX_GPIO = GPIO_NUM_18; 
inline constexpr gpio_num_t RX_GPIO = GPIO_NUM_17; 

#endif

inline constexpr uint8_t DAC_I2C_ADDRESS_HIGHER = 0x61;
inline constexpr uint8_t DAC_I2C_ADDRESS_LOWER  = 0x60; 
inline constexpr uint8_t LSM6DS3_ADDR = 0x6B;

inline constexpr gpio_num_t SDA_GPIO = GPIO_NUM_4;
inline constexpr gpio_num_t SCL_GPIO = GPIO_NUM_5;

/* LORA Module */
inline constexpr spi_host_device_t SPI_HOST_ID = SPI2_HOST;
inline constexpr gpio_num_t PIN_MOSI = GPIO_NUM_11;
inline constexpr gpio_num_t PIN_MISO = GPIO_NUM_12;
inline constexpr gpio_num_t PIN_SCLK = GPIO_NUM_10;
inline constexpr gpio_num_t PIN_CS   = GPIO_NUM_9;
inline constexpr gpio_num_t PIN_RST  = GPIO_NUM_13;
inline constexpr gpio_num_t PIN_BUSY = GPIO_NUM_14;
inline constexpr gpio_num_t PIN_DIO1 = GPIO_NUM_46;
