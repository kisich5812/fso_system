#pragma once

#include "driver/i2c_master.h"

// Fast write: updates DAC register only (not EEPROM) 
esp_err_t mcp4725_write(i2c_master_dev_handle_t dac, uint16_t value); // 0..4095

// Read back: status + DAC register + EEPROM static 
esp_err_t mcp4725_read(i2c_master_dev_handle_t dac,
                       uint16_t *dac_val, uint16_t *eeprom_val, 
                       uint8_t *status);
