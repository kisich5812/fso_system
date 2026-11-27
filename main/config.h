#pragma once

#include <cstdint>

#include "hal/spi_types.h"
#include "soc/gpio_num.h"

namespace board {

// Optical link (RMT TX/RX)
#if defined(OLD_MODULE)
inline constexpr gpio_num_t kTxGpio = GPIO_NUM_8;
inline constexpr gpio_num_t kRxGpio = GPIO_NUM_18;
#else
inline constexpr gpio_num_t kTxGpio = GPIO_NUM_18;
inline constexpr gpio_num_t kRxGpio = GPIO_NUM_17;
#endif

// Shared I2C bus (MCP4725 DACs + LSM6DS3)
inline constexpr gpio_num_t kSdaGpio = GPIO_NUM_4;
inline constexpr gpio_num_t kSclGpio = GPIO_NUM_5;
inline constexpr uint8_t kDacAddressHigher = 0x61;
inline constexpr uint8_t kDacAddressLower = 0x60;
inline constexpr uint8_t kLsm6ds3Address = 0x6B;

// SX1262 LoRa module (SPI)
inline constexpr spi_host_device_t kSpiHostId = SPI2_HOST;
inline constexpr gpio_num_t kPinMosi = GPIO_NUM_11;
inline constexpr gpio_num_t kPinMiso = GPIO_NUM_12;
inline constexpr gpio_num_t kPinSclk = GPIO_NUM_10;
inline constexpr gpio_num_t kPinCs = GPIO_NUM_9;
inline constexpr gpio_num_t kPinRst = GPIO_NUM_13;
inline constexpr gpio_num_t kPinBusy = GPIO_NUM_14;
inline constexpr gpio_num_t kPinDio1 = GPIO_NUM_46;

}  // namespace board
