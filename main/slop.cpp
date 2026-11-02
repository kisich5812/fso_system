#include <string>
#include <string_view>
#include <vector>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
#include "esp_log.h"

static const char *TAG = "rmt_loop";

static constexpr gpio_num_t TX_GPIO     = GPIO_NUM_8;
static constexpr gpio_num_t RX_GPIO     = GPIO_NUM_5;     // jumper 8 -> 5
static constexpr uint32_t   TX_RES_HZ   = 1'000'000;      // 1 tick = 1 us
static constexpr uint32_t   BIT_US      = 10'000;         // 10 ms per bit
static constexpr uint32_t   BIT_TICKS   = BIT_US;         // valid because TX is 1 MHz
static constexpr uint32_t   RX_RES_HZ   = 400'000;        // lower, so 50 ms idle fits
static constexpr uint32_t   RX_IDLE_NS  = 50'000'000;     // must exceed longest run (40 ms)
static constexpr std::string_view MESSAGE = "hello";

static rmt_symbol_word_t sym(int l0, uint32_t d0, int l1, uint32_t d1)
{
    rmt_symbol_word_t s = {};
    s.level0 = l0; s.duration0 = d0;
    s.level1 = l1; s.duration1 = d1;
    return s;
}

// [preamble "10"] [data, 2 bits per symbol, LSB first] [300 ms low gap]
static std::vector<rmt_symbol_word_t> build_frame(std::string_view msg)
{
    std::vector<rmt_symbol_word_t> w;
    w.push_back(sym(1, BIT_TICKS, 0, BIT_TICKS));
    for (char ch : msg) {
        uint8_t c = (uint8_t)ch;
        for (int i = 0; i < 4; i++) {
            w.push_back(sym((c >> (2 * i)) & 1, BIT_TICKS,
                            (c >> (2 * i + 1)) & 1, BIT_TICKS));
        }
    }
    for (int i = 0; i < 5; i++) w.push_back(sym(0, 30'000, 0, 30'000));
    return w;
}

// ---- RX side ----
static bool IRAM_ATTR on_recv_done(rmt_channel_handle_t, const rmt_rx_done_event_data_t *e, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR((QueueHandle_t)ctx, e, &woken);   // just hand it to the task
    return woken == pdTRUE;
}

// Merge same-level neighbours, convert each run to bits by dividing by the bit time
static std::string decode_bits(const rmt_symbol_word_t *s, size_t n, size_t expected_bits)
{
    std::string bits;
    int cur = -1;
    uint32_t ticks = 0;
    auto flush = [&]() {
        if (cur < 0 || ticks == 0) return;
        uint32_t us    = (uint64_t)ticks * 1'000'000 / RX_RES_HZ;
        uint32_t count = (us + BIT_US / 2) / BIT_US;     // round to nearest bit
        bits.append(count, cur ? '1' : '0');
    };
    auto seg = [&](int level, uint32_t d) {
        if (d == 0) return;                               // unused half of the last symbol
        if (level == cur) ticks += d;
        else { flush(); cur = level; ticks = d; }
    };
    for (size_t i = 0; i < n; i++) {
        seg(s[i].level0, s[i].duration0);
        seg(s[i].level1, s[i].duration1);
    }
    flush();
    // The trailing low run is cut off by the idle threshold, so pad/trim to the known length
    bits.resize(expected_bits, '0');
    return bits;
}

extern "C" void app_main(void)
{
    // ---------- RX channel ----------
    rmt_rx_channel_config_t rx_ch_cfg = {};
    rx_ch_cfg.gpio_num          = RX_GPIO;
    rx_ch_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    rx_ch_cfg.resolution_hz     = RX_RES_HZ;
    rx_ch_cfg.mem_block_symbols = 48;
    rmt_channel_handle_t rx_chan = nullptr;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_ch_cfg, &rx_chan));

    QueueHandle_t rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = on_recv_done;
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(rx_chan, &cbs, rx_queue));
    ESP_ERROR_CHECK(rmt_enable(rx_chan));

    static rmt_symbol_word_t rx_buf[64];
    rmt_receive_config_t rx_cfg = {};
    rx_cfg.signal_range_min_ns = 1000;          // glitch filter
    rx_cfg.signal_range_max_ns = RX_IDLE_NS;    // idle longer than this = end of frame

    // Arm RX BEFORE starting TX so the first frame isn't caught halfway
    ESP_ERROR_CHECK(rmt_receive(rx_chan, rx_buf, sizeof(rx_buf), &rx_cfg));

    // ---------- TX channel ----------
    rmt_tx_channel_config_t tx_ch_cfg = {};
    tx_ch_cfg.gpio_num          = TX_GPIO;
    tx_ch_cfg.clk_src           = RMT_CLK_SRC_DEFAULT;
    tx_ch_cfg.resolution_hz     = TX_RES_HZ;
    tx_ch_cfg.mem_block_symbols = 48;
    tx_ch_cfg.trans_queue_depth = 4;
    rmt_channel_handle_t tx_chan = nullptr;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_ch_cfg, &tx_chan));

    rmt_copy_encoder_config_t enc_cfg = {};
    rmt_encoder_handle_t encoder = nullptr;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&enc_cfg, &encoder));

    auto frame = build_frame(MESSAGE);
    ESP_LOGI(TAG, "TX frame: %u symbols (must be <= 48 to loop)", (unsigned)frame.size());

    rmt_transmit_config_t tx = {};
    tx.loop_count = -1;
    ESP_ERROR_CHECK(rmt_enable(tx_chan));
    ESP_ERROR_CHECK(rmt_transmit(tx_chan, encoder, frame.data(),
                                 frame.size() * sizeof(rmt_symbol_word_t), &tx));

    // ---------- receive loop ----------
    const size_t expected_bits = 2 + 8 * MESSAGE.size();
    while (true) {
        rmt_rx_done_event_data_t ev;
        if (xQueueReceive(rx_queue, &ev, portMAX_DELAY) != pdTRUE) continue;

        std::string bits = decode_bits(ev.received_symbols, ev.num_symbols, expected_bits);
        ESP_LOGI(TAG, "RX %u symbols, bits: %s", (unsigned)ev.num_symbols, bits.c_str());

        if (bits.compare(0, 2, "10") != 0) {
            ESP_LOGW(TAG, "preamble missing -> partial or misaligned frame");
        } else {
            std::string text;
            for (size_t b = 2; b + 8 <= bits.size(); b += 8) {
                uint8_t c = 0;
                for (int k = 0; k < 8; k++)
                    if (bits[b + k] == '1') c |= (1 << k);       // LSB first
                ESP_LOGI(TAG, "  %.8s -> 0x%02X '%c'", bits.c_str() + b, c, c >= 32 ? c : '.');
                text += (char)c;
            }
            ESP_LOGI(TAG, "decoded: \"%s\"", text.c_str());
        }

        // Re-arm well before the next frame's preamble arrives
        ESP_ERROR_CHECK(rmt_receive(rx_chan, rx_buf, sizeof(rx_buf), &rx_cfg));
    }
}