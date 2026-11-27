#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "driver/gpio.h"
#include "driver/rmt_encoder.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_tx.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

namespace optical {

constexpr size_t MAX_PAYLOAD = 32;
constexpr size_t RX_BUFFER_SYMBOLS = 512;
constexpr uint32_t MIN_BIT_RATE_HZ = 200;
constexpr uint32_t MAX_BIT_RATE_HZ = 500'000;

struct Config {
    gpio_num_t tx_gpio = GPIO_NUM_NC;
    gpio_num_t rx_gpio = GPIO_NUM_NC;
    uint32_t bit_rate_hz = 100'000;
    bool rx_invert = false;  // set if your receiver module outputs LOW when it sees light
};

struct Frame {
    uint8_t len = 0;
    uint8_t data[MAX_PAYLOAD] = {};
};

// One Link = one TX channel + one RX channel. TX and RX are independent, so
// send() and receive() can be used simultaneously from different tasks.
//
// Over-the-air frame (all bits sent MSB first, NRZ, laser ON = 1, idle = 0):
//   [16-bit preamble 1010...][16-bit sync 0x2DD4][len:8][payload][crc16:16]
// Everything after the sync word is bit-stuffed (after 5 equal bits an
// opposite bit is inserted), so the line never stays constant for long and
// the RMT idle timeout can be used reliably as an end-of-frame marker.
class Link final {
public:
    Link() = default;
    ~Link();

    Link(const Link &) = delete;
    Link &operator=(const Link &) = delete;
    Link(Link &&other) noexcept;
    Link &operator=(Link &&other) noexcept;

    [[nodiscard]] esp_err_t begin(const Config &cfg);
    void deinit();

    bool is_initialized() const { return tx_chan_ != nullptr && rx_chan_ != nullptr; }

    // Blocks until the frame has left the transmitter.
    [[nodiscard]] esp_err_t send(const uint8_t *data, size_t len,
                                   TickType_t timeout = portMAX_DELAY);

    // Returns payload length (>0), -1 on timeout, -2 if `cap` is too small.
    int receive(uint8_t *out, size_t cap, TickType_t timeout);

    uint32_t frames_ok() const { return frames_ok_.load(std::memory_order_relaxed); }
    uint32_t frames_bad() const { return frames_bad_.load(std::memory_order_relaxed); }

private:
    static void rx_task_entry(void *arg);
    void rx_task();
    [[nodiscard]] esp_err_t arm_receiver();

    rmt_channel_handle_t tx_chan_ = nullptr;
    rmt_channel_handle_t rx_chan_ = nullptr;
    rmt_encoder_handle_t encoder_ = nullptr;
    TaskHandle_t rx_task_handle_ = nullptr;
    QueueHandle_t raw_q_ = nullptr;    // ISR -> parser task (raw RMT capture)
    QueueHandle_t frame_q_ = nullptr;  // parser task -> application
    SemaphoreHandle_t tx_mutex_ = nullptr;

    uint32_t resolution_hz_ = 0;
    uint32_t bit_ticks_ = 0;
    uint32_t idle_ticks_ = 0;
    rmt_receive_config_t rx_cfg_ = {};
    rmt_symbol_word_t rx_buf_[RX_BUFFER_SYMBOLS] = {};

    std::vector<uint8_t> tx_bits_;
    std::vector<rmt_symbol_word_t> tx_symbols_;

    std::atomic<uint32_t> frames_ok_{0};
    std::atomic<uint32_t> frames_bad_{0};
};

}  // namespace optical
