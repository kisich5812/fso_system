#include "GalvoSystem.h"
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"

namespace {

const char *TAG = "galvo_system";
constexpr uint8_t DAC_I2C_ADDRESS_HIGHER = 0x61;
constexpr uint8_t DAC_I2C_ADDRESS_LOWER  = 0x60; 

i2c_master_dev_handle_t initialize_dac(i2c_master_bus_handle_t i2c_bus, 
                                       uint8_t addr);

}

GalvoSystem::GalvoSystem(i2c_master_bus_handle_t bus_) 
    : x_dac{initialize_dac(bus_, DAC_I2C_ADDRESS_HIGHER)}
    , y_dac{initialize_dac(bus_, DAC_I2C_ADDRESS_LOWER)}
{}

namespace {

i2c_master_dev_handle_t initialize_dac(i2c_master_bus_handle_t i2c_bus, 
                                       uint8_t addr)
{
    esp_err_t err = i2c_master_probe(i2c_bus, addr, 100);
    if (err) { 
        ESP_LOGE(TAG, "No DAC ACK at 0x%02X", addr);
        return nullptr;
    } 

    i2c_device_config_t dac_config = {};
    dac_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dac_config.device_address = addr;
    dac_config.scl_speed_hz = 100'000;

    i2c_master_dev_handle_t dac = nullptr;
    err = i2c_master_bus_add_device(i2c_bus, &dac_config, &dac);
    if (err) {
        ESP_LOGE(TAG, "i2c master bus add device failed!");
        return nullptr;
    } 

    return dac;
}

}
