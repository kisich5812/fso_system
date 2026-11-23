#include "optical_link.h"

#include <algorithm>
#include <cstring>

#include "esp_attr.h"
#include "esp_check.h"
#include "esp_log.h"

namespace optical {
namespace {

const char *TAG = "optical";

// RMT source clock on the ESP32-S3 (APB / PLL_F80M). The divider is 8 bit, so
// the resolution is picked from 80 MHz / [2..255].
constexpr uint32_t RMT_CLK_HZ = 80'000'000;
constexpr uint32_t TICKS_PER_BIT_TARGET = 32;  // time resolution = 1/32 of a bit
constexpr uint32_t IDLE_BITS = 20;             // idle for 20 bit times == end of frame
constexpr uint32_t MAX_IDLE_TICKS = 32'000;    // RMT idle threshold is a 15 bit value
constexpr size_t RMT_MEM_SYMBOLS = 48;

constexpr size_t PREAMBLE_BITS = 16;           // 1010...  (used to measure the bit time)
constexpr uint16_t SYNC_WORD = 0x2DD4;         // contains "00" and "11": cannot match the preamble
constexpr int STUFF_RUN = 5;
constexpr size_t PREAMBLE_EDGES = 5;           // rising-edge intervals used for timing recovery

struct PulseRun {
    uint8_t level;
    uint32_t duration;  // ticks
};

// ----------------------------------------------------------------- CRC ------
uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

// ------------------------------------------------------------ TX side -------
struct BitStuffer {
    std::vector<uint8_t> &out;
    int last = -1;
    int count = 0;

    void put(uint8_t bit)
    {
        out.push_back(bit);
        if (bit == last) {
            ++count;
        } else {
            last = bit;
            count = 1;
        }
        if (count == STUFF_RUN) {
            const uint8_t stuffed = bit ? 0 : 1;
            out.push_back(stuffed);
            last = stuffed;
            count = 1;
        }
    }
    void put_byte(uint8_t v)
    {
        for (int i = 7; i >= 0; --i) {
            put((v >> i) & 1U);
        }
    }
};

void build_bits(const uint8_t *data, size_t len, std::vector<uint8_t> &bits)
{
    bits.clear();
    for (size_t i = 0; i < PREAMBLE_BITS; ++i) {
        bits.push_back(i % 2 == 0 ? 1 : 0);
    }
    for (int i = 15; i >= 0; --i) {
        bits.push_back((SYNC_WORD >> i) & 1U);
    }

    uint8_t body[1 + MAX_PAYLOAD];
    body[0] = static_cast<uint8_t>(len);
    std::memcpy(&body[1], data, len);
    const uint16_t crc = crc16_ccitt(body, 1 + len);

    BitStuffer st{bits};
    for (size_t i = 0; i < 1 + len; ++i) {
        st.put_byte(body[i]);
    }
    st.put_byte(static_cast<uint8_t>(crc >> 8));
    st.put_byte(static_cast<uint8_t>(crc & 0xFF));
}

void append_phase(std::vector<PulseRun> &runs, uint8_t level, uint32_t duration)
{
    if (duration == 0) {
        return;
    }
    if (!runs.empty() && runs.back().level == level) {
        runs.back().duration += duration;
    } else {
        runs.push_back({level, duration});
    }
}

void bits_to_symbols(const std::vector<uint8_t> &bits, uint32_t bit_ticks,
                     std::vector<rmt_symbol_word_t> &symbols)
{
    std::vector<PulseRun> runs;
    runs.reserve(bits.size());
    for (uint8_t b : bits) {
        append_phase(runs, b, bit_ticks);
    }
    symbols.clear();
    symbols.reserve(runs.size() / 2 + 1);
    for (size_t i = 0; i < runs.size(); i += 2) {
        rmt_symbol_word_t s = {};
        s.level0 = runs[i].level;
        s.duration0 = runs[i].duration;
        if (i + 1 < runs.size()) {
            s.level1 = runs[i + 1].level;
            s.duration1 = runs[i + 1].duration;
        } else {
            // The frame ends on a '1': the second half is one bit of 'off'.
            // (duration must never be 0, that is the RMT end-marker.)
            s.level1 = 0;
            s.duration1 = bit_ticks;
        }
        symbols.push_back(s);
    }
}

// ------------------------------------------------------------ RX side -------

// Tries to decode a frame whose first preamble edge is run `s`.
bool try_decode_at(const std::vector<PulseRun> &runs, const std::vector<uint32_t> &start,
                   size_t s, float nominal_ticks, Frame &out)
{
    if (s + 2 * PREAMBLE_EDGES >= runs.size()) {
        return false;
    }

    // 1) Measure the bit time from rising edge to rising edge (2 bits apart in
    //    the preamble). Rising-to-rising is immune to rise/fall asymmetry.
    const float T = static_cast<float>(start[s + 2 * PREAMBLE_EDGES] - start[s]) /
                    static_cast<float>(2 * PREAMBLE_EDGES);
    if (T < nominal_ticks * 0.75f || T > nominal_ticks * 1.25f) {
        return false;
    }

    // 2) Measure duty-cycle distortion: how much longer than T a '1' pulse is.
    //    Falling edges are shifted by `skew`; we move the sampling point by half
    //    of it so it stays in the middle of the real bit cell.
    float high_sum = 0;
    for (size_t k = 0; k < PREAMBLE_EDGES; ++k) {
        high_sum += static_cast<float>(runs[s + 2 * k].duration);
    }
    float skew = high_sum / PREAMBLE_EDGES - T;
    skew = std::clamp(skew, -0.25f * T, 0.5f * T);

    // 3) Sample the line in the middle of every bit cell, measured from the
    //    first edge (absolute timeline => errors never accumulate and bits are
    //    never silently dropped or inserted).
    const float t0 = static_cast<float>(start[s]);
    const float offset = 0.5f * T + 0.5f * skew;
    size_t r = s;
    auto sample = [&](size_t n, uint8_t &level) -> bool {
        const float ts = t0 + static_cast<float>(n) * T + offset;
        while (r < runs.size() && ts >= static_cast<float>(start[r] + runs[r].duration)) {
            ++r;
        }
        if (r >= runs.size()) {
            return false;
        }
        level = runs[r].level;
        return true;
    };

    size_t n = PREAMBLE_BITS;
    uint16_t sync = 0;
    for (int i = 0; i < 16; ++i, ++n) {
        uint8_t bit;
        if (!sample(n, bit)) {
            return false;
        }
        sync = static_cast<uint16_t>((sync << 1) | bit);
    }
    if (sync != SYNC_WORD) {
        return false;
    }

    // 4) Un-stuff + collect [len][payload][crc]
    uint8_t body[1 + MAX_PAYLOAD + 2] = {};
    size_t body_bits = 0;
    size_t target_bits = 8;
    size_t len = 0;
    int last = -1;
    int count = 0;
    bool expect_stuffed = false;

    while (body_bits < target_bits) {
        uint8_t bit;
        if (!sample(n++, bit)) {
            return false;
        }
        if (expect_stuffed) {
            if (bit == last) {
                return false;  // stuffing violation
            }
            last = bit;
            count = 1;
            expect_stuffed = false;
            continue;
        }
        if (bit == last) {
            ++count;
        } else {
            last = bit;
            count = 1;
        }
        body[body_bits >> 3] |= static_cast<uint8_t>(bit << (7 - (body_bits & 7)));
        ++body_bits;
        if (count == STUFF_RUN) {
            expect_stuffed = true;
        }
        if (body_bits == 8) {
            len = body[0];
            if (len == 0 || len > MAX_PAYLOAD) {
                return false;
            }
            target_bits = (1 + len + 2) * 8;
        }
    }

    const uint16_t rx_crc = static_cast<uint16_t>((body[1 + len] << 8) | body[2 + len]);
    if (crc16_ccitt(body, 1 + len) != rx_crc) {
        return false;
    }
    out.len = static_cast<uint8_t>(len);
    std::memcpy(out.data, &body[1], len);
    return true;
}

// Knows nothing about the content: finds a preamble + sync, recovers timing,
// checks the CRC. Every rising edge is tried as a possible frame start, so a
// noise pulse in front of the real frame does no harm.
bool decode_frame(const rmt_symbol_word_t *symbols, size_t count, float nominal_ticks, Frame &out)
{
    if (!symbols || count == 0) {
        return false;
    }
    std::vector<PulseRun> runs;
    runs.reserve(count * 2);
    for (size_t i = 0; i < count; ++i) {
        append_phase(runs, symbols[i].level0, symbols[i].duration0);
        append_phase(runs, symbols[i].level1, symbols[i].duration1);
    }
    std::vector<uint32_t> start(runs.size());
    uint32_t t = 0;
    for (size_t i = 0; i < runs.size(); ++i) {
        start[i] = t;
        t += runs[i].duration;
    }
    for (size_t s = 0; s < runs.size(); ++s) {
        if (runs[s].level == 1 && try_decode_at(runs, start, s, nominal_ticks, out)) {
            return true;
        }
    }
    return false;
}

bool IRAM_ATTR on_rx_done(rmt_channel_handle_t, const rmt_rx_done_event_data_t *event, void *user)
{
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(static_cast<QueueHandle_t>(user), event, &woken);
    return woken == pdTRUE;
}

}  // namespace

// ---------------------------------------------------------------- Link ------

esp_err_t Link::begin(const Config &cfg)
{
    ESP_RETURN_ON_FALSE(cfg.bit_rate_hz >= MIN_BIT_RATE_HZ && cfg.bit_rate_hz <= MAX_BIT_RATE_HZ,
                        ESP_ERR_INVALID_ARG, TAG, "bit rate out of range");

    uint32_t div = RMT_CLK_HZ / (cfg.bit_rate_hz * TICKS_PER_BIT_TARGET);
    div = std::clamp<uint32_t>(div, 2, 255);
    resolution_hz_ = RMT_CLK_HZ / div;
    bit_ticks_ = (resolution_hz_ + cfg.bit_rate_hz / 2) / cfg.bit_rate_hz;
    idle_ticks_ = std::min<uint32_t>(IDLE_BITS * bit_ticks_, MAX_IDLE_TICKS);

    const uint64_t bit_ns = 1'000'000'000ULL / cfg.bit_rate_hz;
    rx_cfg_.signal_range_min_ns = static_cast<uint32_t>(std::min<uint64_t>(3000, bit_ns / 8));
    rx_cfg_.signal_range_max_ns =
        static_cast<uint32_t>(static_cast<uint64_t>(idle_ticks_) * 1'000'000'000ULL / resolution_hz_);

    ESP_LOGI(TAG, "bit rate %u Hz: RMT resolution %u Hz, %u ticks/bit, idle %u ticks",
             (unsigned)cfg.bit_rate_hz, (unsigned)resolution_hz_, (unsigned)bit_ticks_,
             (unsigned)idle_ticks_);

    raw_q_ = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    frame_q_ = xQueueCreate(4, sizeof(Frame));
    tx_mutex_ = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(raw_q_ && frame_q_ && tx_mutex_, ESP_ERR_NO_MEM, TAG, "no memory");

    rmt_rx_channel_config_t rx = {};
    rx.gpio_num = cfg.rx_gpio;
    rx.clk_src = RMT_CLK_SRC_DEFAULT;
    rx.resolution_hz = resolution_hz_;
    rx.mem_block_symbols = RMT_MEM_SYMBOLS;
    rx.flags.invert_in = cfg.rx_invert;
    ESP_RETURN_ON_ERROR(rmt_new_rx_channel(&rx, &rx_chan_), TAG, "rx channel");

    rmt_rx_event_callbacks_t cbs = {};
    cbs.on_recv_done = on_rx_done;
    ESP_RETURN_ON_ERROR(rmt_rx_register_event_callbacks(rx_chan_, &cbs, raw_q_), TAG, "rx cb");

    rmt_tx_channel_config_t tx = {};
    tx.gpio_num = cfg.tx_gpio;
    tx.clk_src = RMT_CLK_SRC_DEFAULT;
    tx.resolution_hz = resolution_hz_;
    tx.mem_block_symbols = RMT_MEM_SYMBOLS;
    tx.trans_queue_depth = 2;
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&tx, &tx_chan_), TAG, "tx channel");

    rmt_copy_encoder_config_t enc_cfg = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&enc_cfg, &encoder_), TAG, "encoder");

    tx_bits_.reserve(512);
    tx_symbols_.reserve(256);

    ESP_RETURN_ON_FALSE(xTaskCreate(rx_task_entry, "optical_rx", 6144, this, 10, nullptr) == pdPASS,
                        ESP_ERR_NO_MEM, TAG, "task");

    ESP_RETURN_ON_ERROR(rmt_enable(rx_chan_), TAG, "enable rx");
    ESP_RETURN_ON_ERROR(rmt_enable(tx_chan_), TAG, "enable tx");
    return arm_receiver();
}

esp_err_t Link::arm_receiver()
{
    return rmt_receive(rx_chan_, rx_buf_, sizeof(rx_buf_), &rx_cfg_);
}

esp_err_t Link::send(const uint8_t *data, size_t len, TickType_t timeout)
{
    ESP_RETURN_ON_FALSE(data && len > 0 && len <= MAX_PAYLOAD, ESP_ERR_INVALID_ARG, TAG, "bad payload");
    if (xSemaphoreTake(tx_mutex_, timeout) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    build_bits(data, len, tx_bits_);
    bits_to_symbols(tx_bits_, bit_ticks_, tx_symbols_);

    rmt_transmit_config_t tc = {};  // loop_count = 0, line returns LOW (laser off) at the end
    esp_err_t err = rmt_transmit(tx_chan_, encoder_, tx_symbols_.data(),
                                 tx_symbols_.size() * sizeof(rmt_symbol_word_t), &tc);
    if (err == ESP_OK) {
        // tx_symbols_ must stay alive until the hardware is done with it.
        err = rmt_tx_wait_all_done(tx_chan_, -1);
    }
    xSemaphoreGive(tx_mutex_);
    return err;
}

int Link::receive(uint8_t *out, size_t cap, TickType_t timeout)
{
    Frame f;
    if (xQueueReceive(frame_q_, &f, timeout) != pdTRUE) {
        return -1;
    }
    if (cap < f.len) {
        return -2;
    }
    std::memcpy(out, f.data, f.len);
    return f.len;
}

void Link::rx_task_entry(void *arg)
{
    static_cast<Link *>(arg)->rx_task();
}

void Link::rx_task()
{
    rmt_rx_done_event_data_t ev = {};
    for (;;) {
        if (xQueueReceive(raw_q_, &ev, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        Frame f;
        if (decode_frame(ev.received_symbols, ev.num_symbols, static_cast<float>(bit_ticks_), f)) {
            ++frames_ok_;
            xQueueSend(frame_q_, &f, 0);
        } else {
            ++frames_bad_;
        }
        const esp_err_t err = arm_receiver();
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "rmt_receive failed: %s", esp_err_to_name(err));
        }
    }
}

}  // namespace optical