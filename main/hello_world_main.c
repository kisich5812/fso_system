#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

#define I2C_SDA_GPIO GPIO_NUM_2
#define I2C_SCL_GPIO GPIO_NUM_1

static const char *TAG = "scan";

void app_main(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));
    ESP_LOGI(TAG, "Bus created on SDA=%d SCL=%d", I2C_SDA_GPIO, I2C_SCL_GPIO);

        int found = 0;
        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
                ESP_LOGI(TAG, "Found device at 0x%02X", addr);
                found++;
            }
        }
        ESP_LOGI(TAG, "Scan done: %d device(s)", found);
        vTaskDelay(pdMS_TO_TICKS(2000));
}
