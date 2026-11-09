#include <cstdint>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#include "driver/rmt_encoder.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "mcp4725.h"

static const char *TAG = "rmt_loopback";

// GPIO8 transmits; connect it to GPIO3 for the first loopback test.
static constexpr gpio_num_t TX_GPIO = GPIO_NUM_4;
static constexpr gpio_num_t RX_GPIO = GPIO_NUM_3;

// Change these to match the board wiring. GPIO8 and GPIO3 remain dedicated to RMT.
static constexpr gpio_num_t DAC_SDA_GPIO = GPIO_NUM_6;
static constexpr gpio_num_t DAC_SCL_GPIO = GPIO_NUM_7;
static constexpr uint8_t DAC_I2C_ADDRESS = 0x60; // A0 low; common MCP4725 address
static constexpr float DAC_REFERENCE_VOLTAGE = 3.3f; // Set to the DAC module's VDD
static constexpr float FRAC = 0.5f; // Fraction og DAC_REF_VOLTAGE that will be DAC's output

// Both RMT channels use the same resolution and clock source.
static constexpr uint32_t RMT_RESOLUTION_HZ = 1'000'000; // 1 tick = 1 us
static constexpr uint32_t BIT_TICKS = 5'000;             // 5 ms per data bit
static constexpr uint32_t GAP_TICKS = 30'000;            // 30 ms + 30 ms = 60 ms LOW gap
static constexpr uint32_t RX_STOP_NS = 25'000'000;       // Stop after a LOW/HIGH run > 25 ms

static constexpr char MESSAGE[] = "hello";
static constexpr size_t MESSAGE_LEN = sizeof(MESSAGE) - 1;
static constexpr size_t MESSAGE_BITS = MESSAGE_LEN * 8;

static constexpr uint8_t SYNC_BITS[] = {1, 0, 1, 0};
static constexpr size_t SYNC_LEN = sizeof(SYNC_BITS) / sizeof(SYNC_BITS[0]);

static constexpr size_t RMT_MEMORY_SYMBOLS = 48;
static constexpr size_t RX_BUFFER_SYMBOLS = 64;

static QueueHandle_t s_rx_queue = nullptr;
static rmt_channel_handle_t s_rx_channel = nullptr;
static rmt_symbol_word_t s_rx_buffer[RX_BUFFER_SYMBOLS] = {};
static rmt_receive_config_t s_rx_receive_config = {};

struct PulseRun {
    uint8_t level;
    uint32_t duration;
};

static void append_pair(std::vector<rmt_symbol_word_t> &wave, uint8_t bit0, uint8_t bit1)
{
    rmt_symbol_word_t symbol = {};
    symbol.level0 = bit0;
    symbol.duration0 = BIT_TICKS;
    symbol.level1 = bit1;
    symbol.duration1 = BIT_TICKS;
    wave.push_back(symbol);
}

static void append_character(uint8_t c, std::vector<rmt_symbol_word_t> &wave)
{
    // Same bit order as the original program: least significant bit first.
    for (int bit = 0; bit < 8; bit += 2) {
        append_pair(wave,
                    (c >> bit) & 1U,
                    (c >> (bit + 1)) & 1U);
    }
}

static std::vector<rmt_symbol_word_t> make_waveform()
{
    std::vector<rmt_symbol_word_t> wave;
    wave.reserve(2 + MESSAGE_LEN * 4 + 2);

    // Preamble 1010: guarantees a first LOW->HIGH transition and gives RX
    // a pattern to find before decoding the 40 message bits.
    append_pair(wave, SYNC_BITS[0], SYNC_BITS[1]);
    append_pair(wave, SYNC_BITS[2], SYNC_BITS[3]);

    for (size_t i = 0; i < MESSAGE_LEN; ++i) {
        append_character(static_cast<uint8_t>(MESSAGE[i]), wave);
    }

    // End marker 10. Its first HIGH edge terminates the final data bit,
    // so the following long LOW gap cannot merge with that last bit.
    append_pair(wave, 1, 0);

    // Idle gap: two LOW phases, 30 ms each.
    rmt_symbol_word_t gap = {};
    gap.level0 = 0;
    gap.duration0 = GAP_TICKS;
    gap.level1 = 0;
    gap.duration1 = GAP_TICKS;
    wave.push_back(gap);

    return wave;
}

static bool rx_done_callback(rmt_channel_handle_t channel,
                             const rmt_rx_done_event_data_t *event_data,
                             void *user_data)
{
    (void)channel;
    QueueHandle_t queue = static_cast<QueueHandle_t>(user_data);
    BaseType_t higher_priority_task_woken = pdFALSE;

    // Copy the event structure into the queue. Its received_symbols pointer
    // points to s_rx_buffer, which we do not reuse until the parser rearms RX.
    xQueueSendFromISR(queue, event_data, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
}

static void append_phase_to_runs(std::vector<PulseRun> &runs,
                                 uint8_t level,
                                 uint32_t duration)
{
    if (duration == 0) {
        return;
    }

    // Adjacent phases with the same level are one continuous pulse.
    if (!runs.empty() && runs.back().level == level) {
        runs.back().duration += duration;
    } else {
        runs.push_back({level, duration});
    }
}

static bool decode_message(const rmt_symbol_word_t *symbols,
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

    // Turn each run duration back into one or more equal-valued data bits.
    // Example: LOW for 15 ms means three consecutive zero bits.
    std::vector<uint8_t> bits;
    bits.reserve(MESSAGE_BITS + SYNC_LEN + 8);

    for (const PulseRun &run : runs) {
        const uint32_t bit_count = (run.duration + BIT_TICKS / 2) / BIT_TICKS;
        if (bit_count == 0 || bit_count > 16) {
            continue;
        }

        const uint32_t expected_duration = bit_count * BIT_TICKS;
        const uint32_t error = (run.duration > expected_duration)
                                   ? run.duration - expected_duration
                                   : expected_duration - run.duration;

        // Permit timing variation up to one third of a bit period.
        if (error > BIT_TICKS / 3) {
            continue;
        }

        for (uint32_t n = 0; n < bit_count; ++n) {
            bits.push_back(run.level);
        }
    }

    // Find the 1010 preamble, then read the following MESSAGE_BITS bits.
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

static void receiver_task(void *arg)
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

        // The previous transaction is complete; the same buffer can now be reused.
        ESP_ERROR_CHECK(rmt_receive(s_rx_channel,
                                    s_rx_buffer,
                                    sizeof(s_rx_buffer),
                                    &s_rx_receive_config));
    }
}

extern "C" void app_main(void)
{
    // ---------- MCP4725 DAC ----------
    i2c_master_bus_config_t i2c_bus_config = {};
    i2c_bus_config.i2c_port = I2C_NUM_0;
    i2c_bus_config.sda_io_num = DAC_SDA_GPIO;
    i2c_bus_config.scl_io_num = DAC_SCL_GPIO;
    i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t i2c_bus = nullptr;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus));

    // Check the configured address first, then the MCP4725's usual address
    // range (0x60..0x67). The address pins/module variant determine the address.
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
                detected_dac_address = address;
                break;
            }
        }
    }

    ESP_LOGI(TAG, "Detected DAC at 0x%02X", detected_dac_address);

    if (detected_dac_address == 0) {
        ESP_LOGE(TAG, "No DAC ACK at 0x%02X or 0x60..0x67; check SDA/SCL, power, ground, pull-ups and address pins",
                 DAC_I2C_ADDRESS);
    } else {
        i2c_device_config_t dac_config = {};
        dac_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dac_config.device_address = detected_dac_address;
        dac_config.scl_speed_hz = 100'000;

        i2c_master_dev_handle_t dac = nullptr;
        esp_err_t err = i2c_master_bus_add_device(i2c_bus, &dac_config, &dac);
        if (err == ESP_OK) {
            // Example output: half of Vref (1.65 V for a 3.3 V module).
            err = mcp4725_set_fraction(dac, FRAC);
        }
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "MCP4725 at 0x%02X set to %f of Vref (about %.2f V)",
                     detected_dac_address, FRAC, DAC_REFERENCE_VOLTAGE * FRAC);
        } else {
            ESP_LOGE(TAG, "MCP4725 write failed: %s", esp_err_to_name(err));
        }
    }
    // ---------- RX channel on GPIO3 ----------
    rmt_rx_channel_config_t rx_config = {};
    rx_config.gpio_num = RX_GPIO;
    rx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rx_config.resolution_hz = RMT_RESOLUTION_HZ;
    rx_config.mem_block_symbols = RMT_MEMORY_SYMBOLS;

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_config, &s_rx_channel));

    s_rx_receive_config.signal_range_min_ns = 1'000;       // Ignore pulses shorter than 1 us
    s_rx_receive_config.signal_range_max_ns = RX_STOP_NS; // End frame on the 60 ms LOW gap

    s_rx_queue = xQueueCreate(1, sizeof(rmt_rx_done_event_data_t));
    ESP_ERROR_CHECK(s_rx_queue != nullptr ? ESP_OK : ESP_ERR_NO_MEM);

    rmt_rx_event_callbacks_t rx_callbacks = {};
    rx_callbacks.on_recv_done = rx_done_callback;
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(s_rx_channel, &rx_callbacks, s_rx_queue));

    // ---------- TX channel on GPIO8 ----------
    rmt_tx_channel_config_t tx_config = {};
    tx_config.gpio_num = TX_GPIO;
    tx_config.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_config.resolution_hz = RMT_RESOLUTION_HZ;
    tx_config.mem_block_symbols = RMT_MEMORY_SYMBOLS;
    tx_config.trans_queue_depth = 4;

    rmt_channel_handle_t tx_channel = nullptr;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_config, &tx_channel));

    rmt_copy_encoder_config_t encoder_config = {};
    rmt_encoder_handle_t encoder = nullptr;
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&encoder_config, &encoder));

    std::vector<rmt_symbol_word_t> waveform = make_waveform();
    ESP_LOGI(TAG, "TX GPIO=%d, RX GPIO=%d, symbols=%u",
             static_cast<int>(TX_GPIO),
             static_cast<int>(RX_GPIO),
             static_cast<unsigned>(waveform.size()));

    // Start the parser task before enabling reception.
    BaseType_t task_created = xTaskCreate(receiver_task, "rmt_rx_parser", 4096, nullptr, 10, nullptr);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);

    ESP_ERROR_CHECK(rmt_enable(s_rx_channel));
    ESP_ERROR_CHECK(rmt_enable(tx_channel));

    // Arm RX first, so it is listening before TX produces the first edge.
    ESP_ERROR_CHECK(rmt_receive(s_rx_channel,
                                s_rx_buffer,
                                sizeof(s_rx_buffer),
                                &s_rx_receive_config));

    rmt_transmit_config_t tx_transmit_config = {};
    tx_transmit_config.loop_count = -1; // Repeat the same frame forever

    ESP_ERROR_CHECK(rmt_transmit(tx_channel,
                                 encoder,
                                 waveform.data(),
                                 waveform.size() * sizeof(rmt_symbol_word_t),
                                 &tx_transmit_config));

    ESP_LOGI(TAG, "RMT transmitter is looping; connect GPIO8 to GPIO3 for loopback.");

    // Keep waveform and encoder alive while the hardware repeats the frame.
    vTaskDelay(portMAX_DELAY);
}
