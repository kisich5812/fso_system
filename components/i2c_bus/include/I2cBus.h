// I2cBus.h
#pragma once

#include <cstdint>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

namespace bus {

class I2cBus final {
public:
    struct Config {
        i2c_port_num_t port = I2C_NUM_0;
        gpio_num_t sda_gpio = GPIO_NUM_NC;
        gpio_num_t scl_gpio = GPIO_NUM_NC;
        uint32_t glitch_ignore_cnt = 7;
        bool enable_internal_pullup = true;
    };

    I2cBus() = default;
    ~I2cBus();

    I2cBus(const I2cBus &) = delete;
    I2cBus &operator=(const I2cBus &) = delete;

    [[nodiscard]] esp_err_t init(const Config &cfg);
    void deinit();

    bool is_initialized() const { return handle_ != nullptr; }
    i2c_master_bus_handle_t handle() const { return handle_; }

private:
    i2c_master_bus_handle_t handle_ = nullptr;
};

}  // namespace bus
