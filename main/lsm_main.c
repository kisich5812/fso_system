#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "lsm6ds3_reg.h"

#define I2C_SDA_PIN     4
#define I2C_SCL_PIN     5
#define I2C_FREQ_HZ     400000
#define LSM6DS3_ADDR    0x6B   // 0x6B if SA0 is tied high
#define I2C_TIMEOUT_MS  100

static const char *TAG = "lsm6ds3";

/* Platform callbacks required by the ST driver */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *data, uint16_t len)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)handle;
    uint8_t buf[33];
    if (len > sizeof(buf) - 1) return -1;

    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(dev, buf, len + 1, I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t)handle;
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_TIMEOUT_MS) == ESP_OK ? 0 : -1;
}

void app_main(void)
{
    /* I2C bus + device setup */
    i2c_master_bus_config_t bus_cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = LSM6DS3_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    i2c_master_dev_handle_t dev;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus, &dev_cfg, &dev));

    /* Hook the ST driver to our I2C callbacks */
    stmdev_ctx_t ctx = {
        .write_reg = platform_write,
        .read_reg  = platform_read,
        .handle    = dev,
    };

    /* Check WHO_AM_I */
    vTaskDelay(pdMS_TO_TICKS(20));
    uint8_t whoami = 0;
    lsm6ds3_device_id_get(&ctx, &whoami);
    if (whoami != LSM6DS3_ID) {
        ESP_LOGE(TAG, "Device not found (WHO_AM_I = 0x%02X)", whoami);
        return;
    }

    /* Software reset */
    uint8_t rst;
    lsm6ds3_reset_set(&ctx, PROPERTY_ENABLE);
    do {
        lsm6ds3_reset_get(&ctx, &rst);
    } while (rst);

    /* Configure accelerometer: +/-2 g, 104 Hz */
    lsm6ds3_block_data_update_set(&ctx, PROPERTY_ENABLE);
    lsm6ds3_xl_full_scale_set(&ctx, LSM6DS3_2g);
    lsm6ds3_xl_data_rate_set(&ctx, LSM6DS3_XL_ODR_104Hz);

    ESP_LOGI(TAG, "LSM6DS3 ready");

    while (1) {
        int16_t raw[3];
        lsm6ds3_acceleration_raw_get(&ctx, raw);

        printf("Accel [mg]: X=%6.2f  Y=%6.2f  Z=%6.2f\n",
               lsm6ds3_from_fs2g_to_mg(raw[0]),
               lsm6ds3_from_fs2g_to_mg(raw[1]),
               lsm6ds3_from_fs2g_to_mg(raw[2]));

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
