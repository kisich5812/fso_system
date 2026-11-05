#pragma once

#include "driver/i2c_master.h"

// MCP4725 is a 12-bit DAC. The output range is approximately 0..Vref
// (usually the module's VDD); the device itself does not provide a reference.
esp_err_t mcp4725_set_fraction(i2c_master_dev_handle_t dac, float fraction);
esp_err_t mcp4725_set_voltage(i2c_master_dev_handle_t dac, float voltage,
                              float reference_voltage);

// Fast write: updates DAC register only (not EEPROM) 
esp_err_t mcp4725_write(i2c_master_dev_handle_t dac, uint16_t value); // 0..4095

// Read back: status + DAC register + EEPROM static 
esp_err_t mcp4725_read(i2c_master_dev_handle_t dac,
                       uint16_t *dac_val, uint16_t *eeprom_val, 
                       uint8_t *status);
