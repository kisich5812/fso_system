// REVIEW AND REFACTORING REQUIRED! This is mostly slop!

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "mcp4725.h"

#define I2C_SDA_GPIO GPIO_NUM_2
#define I2C_SCL_GPIO GPIO_NUM_1
#define MCP4725_ADDR 0x60

void app_main(void) { 
    i2c_master_dev_handle_t dac;
    i2c_master_bus_config_t bus_cfg = { 
        .i2c_port = I2C_NUM_0, 
        .sda_io_num = I2C_SDA_GPIO, 
        .scl_io_num = I2C_SCL_GPIO, 
        .clk_source = I2C_CLK_SRC_DEFAULT, 
        .glitch_ignore_cnt = 7, 
        .flags.enable_internal_pullup = true,  // weak; external pull-ups are better 
    }; 
    i2c_master_bus_handle_t bus; 
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    ESP_ERROR_CHECK(i2c_master_probe(bus, MCP4725_ADDR, 100));  // is it there?

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = MCP4725_ADDR,
        .scl_speed_hz    = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dac));

    for (size_t i = 1; i < (1 << 12) + 1; i <<= 1) {
        mcp4725_write(dac, i - 1);   // ~half of VDD

        uint16_t d, e; 
        uint8_t s;
        if (mcp4725_read(dac, &d, &e, &s) == ESP_OK)
            ESP_LOGI("mcp4725", "DAC=%u EEPROM=%u status=0x%02X", d, e, s);

        vTaskDelay(300);
    }
}
