#include <cmath>
#include <cstdint>

#include "FreeRTOSConfig.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "mcp4725.h"
#include "portmacro.h"

namespace {

static const char *TAG = "rmt_loopback";
static constexpr gpio_num_t DAC_SDA_GPIO = GPIO_NUM_4;
static constexpr gpio_num_t DAC_SCL_GPIO = GPIO_NUM_5;
static constexpr uint8_t DAC_I2C_ADDRESS = 0x61; // first DAC is 0x60, 
                                                 // second DAC is 0x61
static constexpr float DAC_REFERENCE_VOLTAGE = 3.3f;
static constexpr float FRAC = 0.5f;

}

void initialize_dac()
{
    i2c_master_bus_config_t i2c_bus_config = {};
    i2c_bus_config.i2c_port = I2C_NUM_0;
    i2c_bus_config.sda_io_num = DAC_SDA_GPIO;
    i2c_bus_config.scl_io_num = DAC_SCL_GPIO;
    i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t i2c_bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));

    uint8_t detected_dac_address = 0;
    esp_err_t probe_err = i2c_master_probe(i2c_bus, DAC_I2C_ADDRESS, 100);
    if (probe_err == ESP_OK) {
        detected_dac_address = DAC_I2C_ADDRESS;
    } else {
        for (uint8_t address = 0x60; address <= 0x67; ++address) {
            if (address == DAC_I2C_ADDRESS) {
                continue;
            }
            if (i2c_master_probe(i2c_bus, address, 100) == ESP_OK) {
                ESP_LOGI(TAG, "Found smth at 0x%02X", address);
                detected_dac_address = address;
                break;
            }
        }
    }

    ESP_LOGI(TAG, "Detected DAC at 0x%02X", detected_dac_address);

    if (detected_dac_address == 0) {
        ESP_LOGE(TAG, "No DAC ACK at 0x%02X or 0x60..0x67; check SDA/SCL, power, ground, pull-ups and address pins",
                 DAC_I2C_ADDRESS);
        return;
    }

    i2c_device_config_t dac_config = {};
    dac_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dac_config.device_address = detected_dac_address;
    dac_config.scl_speed_hz = 100'000;

    i2c_master_dev_handle_t dac = nullptr;
    esp_err_t err = i2c_master_bus_add_device(i2c_bus, &dac_config, &dac);
#if 0
    if (err == ESP_OK) {
        err = mcp4725_set_fraction(dac, FRAC);
    }
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MCP4725 at 0x%02X set to %f of Vref (about %.2f V)",
                 detected_dac_address, FRAC, DAC_REFERENCE_VOLTAGE * FRAC);
    } else {
        ESP_LOGE(TAG, "MCP4725 write failed: %s", esp_err_to_name(err));
    }
#endif
    for (;;) {
    for (size_t i = 0; i < 20; i++) {
        mcp4725_set_fraction(dac, std::abs(sin(2 * M_PI * i / 20)));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    }
}
