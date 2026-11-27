#include "i2c_bus.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "config.h"

i2c_master_bus_handle_t initialize_i2c_bus() {
    i2c_master_bus_config_t i2c_bus_config = {};
    i2c_bus_config.i2c_port = I2C_NUM_0;
    i2c_bus_config.sda_io_num = SDA_GPIO;
    i2c_bus_config.scl_io_num = SCL_GPIO;
    i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t i2c_bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));

    return i2c_bus;
}
