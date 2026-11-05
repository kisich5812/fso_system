#include "mcp4725.h"

#include <cmath>

esp_err_t mcp4725_set_fraction(i2c_master_dev_handle_t dac, float fraction) {
    if (!std::isfinite(fraction) || fraction < 0.0f || fraction > 1.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint16_t value = static_cast<uint16_t>(fraction * 4095.0f + 0.5f);
    return mcp4725_write(dac, value);
}

esp_err_t mcp4725_set_voltage(i2c_master_dev_handle_t dac, float voltage,
                              float reference_voltage) {
    if (!std::isfinite(voltage) || !std::isfinite(reference_voltage) ||
        reference_voltage <= 0.0f || voltage < 0.0f || voltage > reference_voltage) {
        return ESP_ERR_INVALID_ARG;
    }
    return mcp4725_set_fraction(dac, voltage / reference_voltage);
}

// Fast write: updates DAC register only (not EEPROM) 
esp_err_t mcp4725_write(i2c_master_dev_handle_t dac, uint16_t value) { // 0..4095
    uint8_t buf[2] = { 
        (uint8_t)((value >> 8) & 0x0F), // 0 0 PD1 PD0 D11..D8 (PD = 00, normal mode) 
        (uint8_t)(value & 0xFF) // D7..D0 
    }; 
    return i2c_master_transmit(dac, buf, sizeof(buf), 100); 
}

// Read back: status + DAC register + EEPROM static 
esp_err_t mcp4725_read(i2c_master_dev_handle_t dac,
                       uint16_t *dac_val, uint16_t *eeprom_val, 
                       uint8_t *status) { 
    uint8_t rx[5]; 
    esp_err_t err = i2c_master_receive(dac, rx, sizeof(rx), 100); 
    if (err != ESP_OK) 
        return err; 
    *status      = rx[0]; // bit7 = RDY/BSY, bit6 = POR 
    *dac_val     = (rx[1] << 4) | (rx[2] >> 4); 
    *eeprom_val  = ((rx[3] & 0x0F) << 8) | rx[4]; 
    return ESP_OK; 
}
