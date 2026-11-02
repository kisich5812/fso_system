// REVIEW AND REFACTORING REQUIRED! This is mostly slop!

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "mcp4725.h"
#include <array>
#include <cstdint>
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_check.h"

#define I2C_SDA_GPIO GPIO_NUM_2
#define I2C_SCL_GPIO GPIO_NUM_1
#define MCP4725_ADDR 0x60

static const char *TAG = "rmt_wave";

static constexpr gpio_num_t RMT_GPIO = GPIO_NUM_8;   // avoid your I2C pins
static constexpr uint32_t   RES_HZ   = 1'000'000;    // 1 tick = 1 us

// One RMT symbol = two (level, duration) segments, so one symbol = one full cycle
static rmt_symbol_word_t make_cycle(uint32_t high_ticks, uint32_t low_ticks) {
    rmt_symbol_word_t s = {};
    s.level0    = 1;
    s.duration0 = high_ticks;   // max 32767 ticks per segment
    s.level1    = 0;
    s.duration1 = low_ticks;
    return s;
}

static constexpr uint32_t TICK_DURATION = 10000; 

static void char_to_rmt_words(char c, std::vector<rmt_symbol_word_t>& vec) {
    for (uint8_t i = 0, mask = 0b1; i < 4; i++) {
        rmt_symbol_word_t s = {};
        s.level0    = (c & mask) ? 1 : 0;
        mask <<= 1;
        s.duration0 = TICK_DURATION;   // max 32767 ticks per segment
        s.level0    = (c & mask) ? 1 : 0;
        mask <<= 1;
        s.duration1 = TICK_DURATION;
        vec.push_back(s);
    }
}

extern "C" 
void app_main(void) { 
    // 1. TX channel
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num          = RMT_GPIO;
    tx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz     = RES_HZ;
    tx_cfg.mem_block_symbols = 48;      // S3: 48 symbols per channel without DMA
    tx_cfg.trans_queue_depth = 4;

    rmt_channel_handle_t chan = nullptr;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &chan));

    // 2. Encoder: "copy" just sends the symbols you give it as-is
    rmt_copy_encoder_config_t enc_cfg = {};
    rmt_encoder_handle_t encoder = nullptr;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &encoder));

    // 3. Generate the wave
    //std::array<rmt_symbol_word_t, 1> wave = { make_cycle(5000, 5000) };
    std::vector<rmt_symbol_word_t> wave{};
    for (char c : "hello") {
        char_to_rmt_words(c, wave);
    }

    // 4. Send, looping forever
    rmt_transmit_config_t tx = {};
    tx.loop_count = -1; // -1 = infinite; 0 = once

    ESP_ERROR_CHECK(rmt_enable(chan));
    ESP_ERROR_CHECK(rmt_transmit(chan, encoder, wave.data(),
                                 wave.size() * sizeof(rmt_symbol_word_t), &tx));
    ESP_LOGI(TAG, "Transmitting on GPIO %d", RMT_GPIO);

    vTaskDelay(portMAX_DELAY);

// DAC stuff
#if 0
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
#endif


}
