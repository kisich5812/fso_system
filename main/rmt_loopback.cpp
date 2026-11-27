#include "rmt_loopback.h"

#include <cmath>
#include <cstdio>

#include "GalvoSystem.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "mcp4725.h"
#include "optical_link.h"

namespace {

const char *TAG = "rmt_loopback";
constexpr gpio_num_t TX_GPIO = GPIO_NUM_18; // 8 or 18
constexpr gpio_num_t RX_GPIO = GPIO_NUM_17; // 18 or 17
constexpr uint32_t BIT_RATE_HZ = 2'000;  // try 2000 first, then work your way up

optical::Link g_link;

// The sender is the only one that knows what it sends.
void tx_task(void *)
{
    //GalvoSystem* galv = (GalvoSystem*)arg;
    i2c_master_bus_handle_t bus = initialize_i2c_bus();
    GalvoSystem galv{bus};


// 1. Reset the pin to a clean default state
    gpio_reset_pin(TX_GPIO);

    // 2. Set the GPIO direction to output
    gpio_set_direction(TX_GPIO, GPIO_MODE_OUTPUT);

    // 3. Set the output level to HIGH (3.3V)
    gpio_set_level(TX_GPIO, 1);

    float r = 0.0f;
    for (unsigned n = 0;; ++n) {
        char msg[optical::MAX_PAYLOAD] = "UUUUU";
        int len; //std::snprintf(msg, sizeof(msg), "Hello! #%u", n);
        //if (len > static_cast<int>(sizeof(msg)) - 1) {
            len = sizeof(msg) - 1;
        //}
        const esp_err_t err = g_link.send(reinterpret_cast<const uint8_t *>(msg), len);
#if 0
        const size_t disc = 360;
        if (r > 1)
            r = 0.0f;
        float frac_1 = 0.5f + r * cos(2 * M_PI * n / disc);
        float frac_2 = 0.5f + r * sin(2 * M_PI * n / disc);
        mcp4725_set_fraction(galv.x_dac, frac_1);
        mcp4725_set_fraction(galv.y_dac, frac_2);
//        ESP_LOGI(TAG, "x = %5.2f, y = %5.2f", frac_1, frac_2);

        if (n % disc == 0)
            r += 0.01f;
#endif
        
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
