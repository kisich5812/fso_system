// I2cBus.cpp
#include "I2cBus.h"

#include "esp_check.h"
#include "esp_log.h"

namespace bus {
namespace {
constexpr const char *TAG = "i2c_bus";
}  // namespace

I2cBus::~I2cBus() {
    deinit();
}

esp_err_t I2cBus::init(const Config &cfg) {
    ESP_RETURN_ON_FALSE(cfg.sda_gpio != GPIO_NUM_NC && cfg.scl_gpio != GPIO_NUM_NC,
                        ESP_ERR_INVALID_ARG, TAG, "Invalid SDA/SCL GPIO");

    deinit();

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = cfg.port;
    bus_cfg.sda_io_num = cfg.sda_gpio;
    bus_cfg.scl_io_num = cfg.scl_gpio;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = cfg.glitch_ignore_cnt;
    bus_cfg.flags.enable_internal_pullup = cfg.enable_internal_pullup;

    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &handle_),
                        TAG, "Failed to create I2C master bus");
    return ESP_OK;
}

void I2cBus::deinit() {
    if (handle_ != nullptr) {
        i2c_del_master_bus(handle_);
        handle_ = nullptr;
    }
}

}  // namespace bus
