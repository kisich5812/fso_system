#include "rmt_loopback.h"

#include <cstring>

#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "optical_link.h"

using namespace board;
using namespace optical;

namespace {

constexpr const char *TAG = "rmt_loopback";
constexpr uint32_t BIT_RATE_HZ = 2'000;  // try 2000 first, then work your way up
constexpr const char kTestMsg[] = "UUUUU";

// Scoped to this translation unit (not a public global), living for the program's lifetime.
Link s_link;

void tx_task(void *)
{
    for (unsigned n = 0;; ++n) {
        const size_t len = sizeof(kTestMsg) - 1;
        const esp_err_t err = s_link.send(reinterpret_cast<const uint8_t *>(kTestMsg), len);
        ESP_LOGI(TAG, "TX #%u (%u bytes): %s -> %s",
                 n, static_cast<unsigned>(len), kTestMsg, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void rx_task(void *)
{
    uint8_t buf[MAX_PAYLOAD + 1];
    for (;;) {
        const int len = s_link.receive(buf, MAX_PAYLOAD, pdMS_TO_TICKS(2000));
        if (len < 0) {
            ESP_LOGW(TAG, "no frame for 2 s (ok=%u, bad=%u)",
                     static_cast<unsigned>(s_link.frames_ok()),
                     static_cast<unsigned>(s_link.frames_bad()));
            continue;
        }
        buf[len] = '\0';
        ESP_LOGI(TAG, "RX (%d bytes): %s", len, reinterpret_cast<char *>(buf));
    }
}

}  // namespace

void start_rmt_loopback()
{
    Config cfg;
    cfg.tx_gpio = kTxGpio;
    cfg.rx_gpio = kRxGpio;
    cfg.bit_rate_hz = BIT_RATE_HZ;
    ESP_ERROR_CHECK(s_link.begin(cfg));

    xTaskCreate(rx_task, "app_rx", 4096, nullptr, 5, nullptr);
    xTaskCreate(tx_task, "app_tx", 4096, nullptr, 5, nullptr);
}
