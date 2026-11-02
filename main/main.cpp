#include <vector>
#include <string_view>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_encoder.h"
#include "esp_log.h"

static const char *TAG = "rmt_text";

static constexpr gpio_num_t RMT_GPIO      = GPIO_NUM_8;
static constexpr uint32_t   RES_HZ        = 1'000'000;  // 1 tick = 1 us
static constexpr uint32_t   BIT_TICKS     = 10'000;     // 10 ms per bit (max 32767)

#include <string>

static std::string bits_of(const std::vector<rmt_symbol_word_t> &v, size_t from)
{
    std::string s;
    for (size_t i = from; i < v.size(); i++) {
        s += v[i].level0 ? '1' : '0';
        s += v[i].level1 ? '1' : '0';
    }
    return s;
}

static void log_runs(const std::vector<rmt_symbol_word_t> &w)
{
    int cur = -1;
    uint32_t ticks = 0;
    auto flush = [&]() {
        if (cur >= 0)
            ESP_LOGI(TAG, "  %s for %u ms", cur ? "HIGH" : "LOW ",
                     (unsigned)(ticks * 1000ULL / RES_HZ));
    };
    auto seg = [&](int level, uint32_t d) {
        if (level == cur) { ticks += d; }
        else { flush(); cur = level; ticks = d; }
    };
    for (const auto &s : w) {
        seg(s.level0, s.duration0);
        seg(s.level1, s.duration1);
    }
    flush();
}

static void char_to_rmt_words(uint8_t c, std::vector<rmt_symbol_word_t> &out)
{
    for (int i = 0; i < 4; i++) {
        rmt_symbol_word_t s = {};
        s.level0    = (c >> (2 * i))     & 1;
        s.duration0 = BIT_TICKS;
        s.level1    = (c >> (2 * i + 1)) & 1;
        s.duration1 = BIT_TICKS;
        out.push_back(s);
    }
}

extern "C" void app_main(void)
{
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num          = RMT_GPIO;
    tx_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz     = RES_HZ;
    tx_cfg.mem_block_symbols = 48;
    tx_cfg.trans_queue_depth = 4;
    rmt_channel_handle_t chan = nullptr;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &chan));

    rmt_copy_encoder_config_t enc_cfg = {};
    rmt_encoder_handle_t encoder = nullptr;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &encoder));

    std::vector<rmt_symbol_word_t> wave;
    for (char ch : std::string_view("hello")) {
        size_t start = wave.size();
        char_to_rmt_words((uint8_t)ch, wave);
        ESP_LOGI(TAG, "'%c' 0x%02X -> bits (sent order) %s",
                ch >= 32 ? ch : '.', (unsigned)(uint8_t)ch,
                bits_of(wave, start).c_str());
    }

    // Idle gap so a receiver can see where the message restarts
    rmt_symbol_word_t gap = {};
    gap.level0 = 0; gap.duration0 = 30'000;
    gap.level1 = 0; gap.duration1 = 30'000;
    wave.push_back(gap);

    ESP_LOGI(TAG, "Waveform runs:");
    log_runs(wave);

    ESP_LOGI(TAG, "%u symbols", (unsigned)wave.size());   // must be <= 48 for looping

    rmt_transmit_config_t tx = {};
    tx.loop_count = -1;

    ESP_ERROR_CHECK(rmt_enable(chan));
    ESP_ERROR_CHECK(rmt_transmit(chan, encoder, wave.data(),
                                 wave.size() * sizeof(rmt_symbol_word_t), &tx));
    ESP_LOGI(TAG, "Transmitting on GPIO %d", (int)RMT_GPIO);

    vTaskDelay(portMAX_DELAY);
}

    
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