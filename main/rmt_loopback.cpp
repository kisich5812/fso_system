#include "rmt_loopback.h"

#include <cstdio>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "optical_link.h"

namespace {

const char *TAG = "rmt_loopback";
constexpr gpio_num_t TX_GPIO = GPIO_NUM_18;
constexpr gpio_num_t RX_GPIO = GPIO_NUM_3;
constexpr uint32_t BIT_RATE_HZ = 2'000;  // try 2000 first, then work your way up

optical::Link g_link;

// The sender is the only one that knows what it sends.
void tx_task(void *)
{
    for (unsigned n = 0;; ++n) {
        char msg[optical::MAX_PAYLOAD] = "UUUUUUUUUUUUUUUUUUUUUUUUUUUUUU";
        int len; //std::snprintf(msg, sizeof(msg), "Hello! #%u", n);
        //if (len > static_cast<int>(sizeof(msg)) - 1) {
            len = sizeof(msg) - 1;
        //}
        const esp_err_t err = g_link.send(reinterpret_cast<const uint8_t *>(msg), len);
        ESP_LOGI(TAG, "TX #%u (%d bytes): %s -> %s", n, len, msg, esp_err_to_name(err));
        //vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// The receiver has no idea what comes in.
void rx_task(void *)
{
    uint8_t buf[optical::MAX_PAYLOAD + 1];
    for (;;) {
        const int len = g_link.receive(buf, optical::MAX_PAYLOAD, pdMS_TO_TICKS(2000));
        if (len < 0) {
            ESP_LOGW(TAG, "no frame for 2 s (ok=%u, bad=%u)",
                     (unsigned)g_link.frames_ok(), (unsigned)g_link.frames_bad());
            continue;
        }
        buf[len] = '\0';
        ESP_LOGI(TAG, "RX (%d bytes): %s", len, reinterpret_cast<char *>(buf));
    }
}

}  // namespace

void start_rmt_loopback()
{
    optical::Config cfg;
    cfg.tx_gpio = TX_GPIO;
    cfg.rx_gpio = RX_GPIO;
    cfg.bit_rate_hz = BIT_RATE_HZ;
    ESP_ERROR_CHECK(g_link.begin(cfg));

    //xTaskCreate(rx_task, "app_rx", 4096, nullptr, 5, nullptr);
    xTaskCreate(tx_task, "app_tx", 4096, nullptr, 5, nullptr);
}
