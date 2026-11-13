#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <sys/select.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
#include "esp_err.h"
#include "esp_log.h"

namespace { //anonymous

const char *TAG = "rmt_loopback";
constexpr gpio_num_t TX_GPIO = GPIO_NUM_4;
constexpr gpio_num_t RX_GPIO = GPIO_NUM_3;
constexpr uint32_t RMT_RESOLUTION_HZ = 1'000'000; // 1 tick = 1 us
constexpr uint32_t DEFAULT_BIT_TICKS = 5'000;
constexpr uint32_t GAP_TICKS = 10'000;
constexpr uint32_t GAP_N     = 100;
constexpr uint32_t RX_STOP_NS = 25'000'000;
constexpr char MESSAGE[] = "hello";
constexpr size_t MESSAGE_LEN = sizeof(MESSAGE) - 1;
constexpr size_t MESSAGE_BITS = MESSAGE_LEN * 8;
constexpr uint8_t SYNC_BITS[] = {1, 0, 1, 0};
constexpr size_t SYNC_LEN = sizeof(SYNC_BITS) / sizeof(SYNC_BITS[0]);
constexpr size_t RMT_MEMORY_SYMBOLS = 48;
constexpr size_t RX_BUFFER_SYMBOLS = 64;
QueueHandle_t s_rx_queue = nullptr;
rmt_channel_handle_t s_rx_channel = nullptr;
rmt_symbol_word_t s_rx_buffer[RX_BUFFER_SYMBOLS] = {};
rmt_receive_config_t s_rx_receive_config = {};
uint32_t s_bit_ticks = DEFAULT_BIT_TICKS;

struct PulseRun {
    uint8_t level;
    uint32_t duration;
};

void append_pair(std::vector<rmt_symbol_word_t> &wave, uint8_t bit0, uint8_t bit1);
void append_character(uint8_t c, std::vector<rmt_symbol_word_t> &wave);
std::vector<rmt_symbol_word_t> make_waveform();
bool rx_done_callback(rmt_channel_handle_t channel,
                      const rmt_rx_done_event_data_t *event_data,
                      void *user_data);
void append_phase_to_runs(std::vector<PulseRun> &runs, uint8_t level, uint32_t duration);
bool decode_message(const rmt_symbol_word_t *symbols,
                    size_t symbol_count,
                    char *output,
                    size_t output_size);
void receiver_task(void *arg);
void initialize_receiver();
rmt_channel_handle_t initialize_transmitter();
uint32_t read_transmit_frequency();

}

void start_rmt_loopback()
{
    s_bit_ticks = read_transmit_frequency();
    initialize_receiver();
    rmt_channel_handle_t tx_channel = initialize_transmitter();
    rmt_copy_encoder_config_t encoder_config = {};
    rmt_encoder_handle_t encoder = nullptr;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &encoder));
    std::vector<rmt_symbol_word_t> waveform = make_waveform();
    ESP_LOGI(TAG, "TX GPIO=%d, RX GPIO=%d, symbols=%u",
             static_cast<int>(TX_GPIO),
             static_cast<int>(RX_GPIO),
             static_cast<unsigned>(waveform.size()));
    BaseType_t task_created = xTaskCreate(receiver_task, "rmt_rx_parser", 4096, nullptr, 10, nullptr);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
    ESP_ERROR_CHECK(rmt_enable(s_rx_channel));
    ESP_ERROR_CHECK(rmt_enable(tx_channel));
    ESP_ERROR_CHECK(rmt_receive(s_rx_channel,
                                s_rx_buffer,
                                sizeof(s_rx_buffer),
                                &s_rx_receive_config));
    rmt_transmit_config_t tx_transmit_config = {};
    tx_transmit_config.loop_count = -1;
    ESP_ERROR_CHECK(rmt_transmit(tx_channel,
                                 encoder,
                                 waveform.data(),
                                 waveform.size() * sizeof(rmt_symbol_word_t),
                                 &tx_transmit_config));
    ESP_LOGI(TAG, "RMT transmitter is looping; connect GPIO8 to GPIO3 for loopback.");
    vTaskDelay(portMAX_DELAY);
}

namespace { //anonymous

void append_pair(std::vector<rmt_symbol_word_t> &wave, uint8_t bit0, uint8_t bit1)
{
    rmt_symbol_word_t symbol = {};
    symbol.level0 = bit0;
    symbol.duration0 = s_bit_ticks;
    symbol.level1 = bit1;
    symbol.duration1 = s_bit_ticks;
    wave.push_back(symbol);
}

void append_character(uint8_t c, std::vector<rmt_symbol_word_t> &wave)
{
    for (int bit = 0; bit < 8; bit += 2) {
        append_pair(wave, (c >> bit) & 1U, (c >> (bit + 1)) & 1U);
    }
}

std::vector<rmt_symbol_word_t> make_waveform()
{
    std::vector<rmt_symbol_word_t> wave;
    wave.reserve(2 + MESSAGE_LEN * 4 + 2);
    append_pair(wave, SYNC_BITS[0], SYNC_BITS[1]);
    append_pair(wave, SYNC_BITS[2], SYNC_BITS[3]);
    for (size_t i = 0; i < MESSAGE_LEN; ++i) {
        append_character(static_cast<uint8_t>(MESSAGE[i]), wave);
    }
    append_pair(wave, 1, 0);
    rmt_symbol_word_t gap = {};
    gap.level0 = 0;
    gap.duration0 = GAP_TICKS;
    gap.level1 = 0;
    gap.duration1 = GAP_TICKS;
    for (size_t i = 0; i < GAP_N; ++i)
        wave.push_back(gap);

    return wave;
}

bool rx_done_callback(rmt_channel_handle_t channel,
                      const rmt_rx_done_event_data_t *event_data,
                      void *user_data)
{
    (void)channel;
    QueueHandle_t queue = static_cast<QueueHandle_t>(user_data);
    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueSendFromISR(queue, event_data, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}


void append_phase_to_runs(std::vector<PulseRun> &runs, uint8_t level, uint32_t duration)
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

bool decode_message(const rmt_symbol_word_t *symbols,
                    size_t symbol_count,
                    char *output,
                    size_t output_size)
{
    if (symbols == nullptr || output == nullptr || output_size < MESSAGE_LEN + 1) {
        return false;
    }
    std::vector<PulseRun> runs;
    runs.reserve(symbol_count * 2);
    for (size_t i = 0; i < symbol_count; ++i) {
        append_phase_to_runs(runs, symbols[i].level0, symbols[i].duration0);
        append_phase_to_runs(runs, symbols[i].level1, symbols[i].duration1);
    }
    std::vector<uint8_t> bits;
    bits.reserve(MESSAGE_BITS + SYNC_LEN + 8);
    for (const PulseRun &run : runs) {
        const uint32_t bit_count = (run.duration + s_bit_ticks / 2) / s_bit_ticks;
        if (bit_count == 0 || bit_count > 16) {
            continue;
        }
        const uint32_t expected_duration = bit_count * s_bit_ticks;
        const uint32_t error = (run.duration > expected_duration)
                                   ? run.duration - expected_duration
                                   : expected_duration - run.duration;
        if (error > s_bit_ticks / 3) {
            continue;
        }
        for (uint32_t n = 0; n < bit_count; ++n) {
            bits.push_back(run.level);
        }
    }
    for (size_t start = 0; start + SYNC_LEN + MESSAGE_BITS <= bits.size(); ++start) {
        bool sync_found = true;
        for (size_t j = 0; j < SYNC_LEN; ++j) {
            if (bits[start + j] != SYNC_BITS[j]) {
                sync_found = false;
                break;
            }
        }
        if (!sync_found) {
            continue;
        }
        const size_t data_start = start + SYNC_LEN;
        for (size_t character = 0; character < MESSAGE_LEN; ++character) {
            uint8_t value = 0;
            for (size_t bit = 0; bit < 8; ++bit) {
                value |= static_cast<uint8_t>(bits[data_start + character * 8 + bit] << bit);
            }
            output[character] = static_cast<char>(value);
        }
        output[MESSAGE_LEN] = '\0';
        return true;
    }
    return false;
}

void receiver_task(void *arg)
{
    (void)arg;
    rmt_rx_done_event_data_t event_data = {};
    while (true) {
        if (xQueueReceive(s_rx_queue, &event_data, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        char received[MESSAGE_LEN + 1] = {};
        const bool decoded = decode_message(event_data.received_symbols,
                                            event_data.num_symbols,
                                            received,
                                            sizeof(received));
        ESP_LOGI(TAG, "Отправлено: %s", MESSAGE);
        if (decoded) {
            ESP_LOGI(TAG, "Принято:   %s", received);
        } else {
            ESP_LOGW(TAG, "Не удалось декодировать кадр (получено RMT-символов: %u)",
                     static_cast<unsigned>(event_data.num_symbols));
        }
        ESP_ERROR_CHECK(rmt_receive(s_rx_channel,
                                    s_rx_buffer,
                                    sizeof(s_rx_buffer),
                                    &s_rx_receive_config));
    }
}

void initialize_receiver()
{
    rmt_rx_channel_config_t rx_config = {};
    rx_config.gpio_num = RX_GPIO;
    rx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_config.resolution_hz = RMT_RESOLUTION_HZ;
    rx_config.mem_block_symbols = RMT_MEMORY_SYMBOLS;
    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_config, &s_rx_channel));
    s_rx_receive_config.signal_range_min_ns = 1'000;
    s_rx_receive_config.signal_range_max_ns = RX_STOP_NS;
    s_rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    ESP_ERROR_CHECK(s_rx_queue != nullptr ? ESP_OK : ESP_ERR_NO_MEM);
    rmt_rx_event_callbacks_t rx_callbacks = {};
    rx_callbacks.on_recv_done = rx_done_callback;
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(s_rx_channel, &rx_callbacks, s_rx_queue));
}

rmt_channel_handle_t initialize_transmitter()
{
    rmt_tx_channel_config_t tx_config = {};
    tx_config.gpio_num = TX_GPIO;
    tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_config.resolution_hz = RMT_RESOLUTION_HZ;
    tx_config.mem_block_symbols = RMT_MEMORY_SYMBOLS;
    tx_config.trans_queue_depth = 4;
    rmt_channel_handle_t tx_channel = nullptr;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &tx_channel));
    return tx_channel;
}

uint32_t read_transmit_frequency()
{
    constexpr uint32_t minimum_frequency =
        (RMT_RESOLUTION_HZ + 32'766) / 32'767;
    constexpr uint32_t maximum_frequency = RMT_RESOLUTION_HZ;
    char input[16] = {};
    size_t length = 0;

    std::printf("Enter transmit frequency in Hz (31-1000000, default 200): ");
    std::fflush(stdout);

    while (length + 1 < sizeof(input)) {
        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(STDIN_FILENO, &read_fds);
        timeval timeout = {};
        timeout.tv_sec = 5;
        const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
        if (ready <= 0) {
            break;
        }

        char character = 0;
        if (read(STDIN_FILENO, &character, 1) != 1) {
            break;
        }
        if (character == '\n' || character == '\r') {
            break;
        }
        if (character >= '0' && character <= '9') {
            input[length++] = character;
        } else if (character != ' ' && character != '\t') {
            length = 0;
            break;
        }
    }

    if (length == 0) {
        std::printf("Using default frequency: %lu Hz\n", 
                    RMT_RESOLUTION_HZ / DEFAULT_BIT_TICKS);
        return DEFAULT_BIT_TICKS;
    }

    input[length] = '\0';
    char *end = nullptr;
    const unsigned long frequency = std::strtoul(input, &end, 10);
    if (*end != '\0' || frequency < minimum_frequency || frequency > maximum_frequency) {
        std::printf("Frequency out of range; using default: %lu Hz\n",
                    RMT_RESOLUTION_HZ / DEFAULT_BIT_TICKS);
        return DEFAULT_BIT_TICKS;
    }

    const uint32_t bit_ticks = RMT_RESOLUTION_HZ / static_cast<uint32_t>(frequency);
    std::printf("Transmitting at %lu Hz (%lu ticks per bit)\n",
                RMT_RESOLUTION_HZ / bit_ticks, bit_ticks);
    return bit_ticks;
}

}
