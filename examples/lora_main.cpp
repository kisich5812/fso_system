#include <chrono>
#include <cstring>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sx126x.hpp"

using namespace std::chrono_literals;

static const char *TAG = "lora_ping_pong";

// Adjust these GPIOs to match your ESP32 + SX1262 wiring
constexpr spi_host_device_t SPI_HOST_ID = SPI2_HOST;
constexpr gpio_num_t PIN_MOSI = GPIO_NUM_11;
constexpr gpio_num_t PIN_MISO = GPIO_NUM_12;
constexpr gpio_num_t PIN_SCLK = GPIO_NUM_10;
constexpr gpio_num_t PIN_CS   = GPIO_NUM_9;
constexpr gpio_num_t PIN_RST  = GPIO_NUM_13;
constexpr gpio_num_t PIN_BUSY = GPIO_NUM_14;
constexpr gpio_num_t PIN_DIO1 = GPIO_NUM_46;

static TaskHandle_t s_irq_task_handle = nullptr;

// DIO1 ISR: never do SPI inside an ISR—just wake the radio task
static void IRAM_ATTR dio1_isr_handler(void * /*arg*/) {
  BaseType_t higher_priority_woken = pdFALSE;
  if (s_irq_task_handle) {
    vTaskNotifyGiveFromISR(s_irq_task_handle, &higher_priority_woken);
    portYIELD_FROM_ISR(higher_priority_woken);
  }
}

extern "C" void app_main(void) {
  // 1. Configure control GPIOs (NRESET, BUSY, DIO1)
  gpio_config_t out_cfg{};
  out_cfg.pin_bit_mask = (1ULL << PIN_RST);
  out_cfg.mode = GPIO_MODE_OUTPUT;
  gpio_config(&out_cfg);
  gpio_set_level(PIN_RST, 1);

  gpio_config_t in_cfg{};
  in_cfg.pin_bit_mask = (1ULL << PIN_BUSY);
  in_cfg.mode = GPIO_MODE_INPUT;
  gpio_config(&in_cfg);

  gpio_config_t dio1_cfg{};
  dio1_cfg.pin_bit_mask = (1ULL << PIN_DIO1);
  dio1_cfg.mode = GPIO_MODE_INPUT;
  dio1_cfg.intr_type = GPIO_INTR_POSEDGE;
  gpio_config(&dio1_cfg);

  // 2. Initialize ESP-IDF SPI bus and add the SX1262 device
  spi_bus_config_t bus_cfg{};
  bus_cfg.mosi_io_num = PIN_MOSI;
  bus_cfg.miso_io_num = PIN_MISO;
  bus_cfg.sclk_io_num = PIN_SCLK;
  bus_cfg.quadwp_io_num = -1;
  bus_cfg.quadhd_io_num = -1;
  ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_ID, &bus_cfg, SPI_DMA_CH_AUTO));

  spi_device_interface_config_t dev_cfg{};
  dev_cfg.clock_speed_hz = 8 * 1000 * 1000; // 8 MHz (SX1262 supports up to 16 MHz)
  dev_cfg.mode = 0;                         // SPI Mode 0 (CPOL=0, CPHA=0)
  dev_cfg.spics_io_num = PIN_CS;
  dev_cfg.queue_size = 4;

  spi_device_handle_t spi_dev = nullptr;
  ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST_ID, &dev_cfg, &spi_dev));

  // 3. Build SPI lambdas required by espp::Sx126x
  auto spi_write = [spi_dev](const uint8_t *data, size_t len) -> bool {
    if (len == 0) return true;
    spi_transaction_t t{};
    t.length = len * 8;
    t.tx_buffer = data;
    return spi_device_polling_transmit(spi_dev, &t) == ESP_OK;
  };

  // SX1262 requires CS to stay asserted across the command (write) and response (read)
  auto spi_write_then_read = [spi_dev](const uint8_t *tx_data, size_t tx_len,
                                       uint8_t *rx_data, size_t rx_len) -> bool {
    const size_t total_len = tx_len + rx_len;
    if (total_len == 0) return true;

    std::vector<uint8_t> tx_buf(total_len, 0x00);
    std::vector<uint8_t> rx_buf(total_len, 0x00);
    if (tx_len > 0) {
      std::memcpy(tx_buf.data(), tx_data, tx_len);
    }

    spi_transaction_t t{};
    t.length = total_len * 8;
    t.tx_buffer = tx_buf.data();
    t.rx_buffer = rx_buf.data();

    if (spi_device_polling_transmit(spi_dev, &t) != ESP_OK) {
      return false;
    }
    if (rx_len > 0) {
      std::memcpy(rx_data, rx_buf.data() + tx_len, rx_len);
    }
    return true;
  };

  // 4. Configure and initialize the SX1262 driver
  espp::Sx126x::RadioConfig radio_cfg{};
  radio_cfg.frequency_hz = 868'100'000; // EU868 (use e.g. 915'000'000 for US915)
  radio_cfg.spreading_factor = espp::Sx126x::SpreadingFactor::SF9;
  radio_cfg.bandwidth = espp::Sx126x::Bandwidth::BW_125_KHZ;
  radio_cfg.coding_rate = espp::Sx126x::CodingRate::CR_4_5;
  radio_cfg.tx_power_dbm = 14;          // Keep within local ERP limits

  espp::Sx126x radio({
      .variant = espp::Sx126x::Variant::SX1262,
      .write = spi_write,
      .write_then_read = spi_write_then_read,
      .is_busy = []() { return gpio_get_level(PIN_BUSY) != 0; },
      .reset = [](bool level) { gpio_set_level(PIN_RST, level ? 1 : 0); },
      .tcxo_voltage = 1.8f,               // Float in volts (1.6f to 3.3f; 0.0f disables TCXO)
      .use_dio2_as_rf_switch = true,      // Defaults to true
      .radio_config = radio_cfg,
      .auto_init = false,                 // Prevent double-init since you call radio.initialize(ec) below
      .log_level = espp::Logger::Verbosity::INFO,
  });

  std::error_code ec;
  if (!radio.initialize(ec)) {
    ESP_LOGE(TAG, "SX1262 init failed: %s", ec.message().c_str());
    return;
  }

  // 5. Register RX callback (invoked from the task that calls handle_dio1_interrupt)
  radio.set_receive_callback([&radio](const espp::Sx126x::RxPacket &packet) {
    std::string msg(reinterpret_cast<const char *>(packet.data.data()), packet.data.size());
    ESP_LOGI(TAG, "RX: '%s' | RSSI: %.1f dBm | SNR: %.1f dB",
             msg.c_str(), packet.status.rssi, packet.status.snr);

    // If we received a PING, reply with a PONG
    if (msg.rfind("PING", 0) == 0) {
      std::string reply = "PONG" + msg.substr(4);
      std::error_code tx_ec;
      auto bytes = std::span<const uint8_t>(
          reinterpret_cast<const uint8_t *>(reply.data()), reply.size());
      if (radio.transmit(bytes, 2000ms, tx_ec)) {
        ESP_LOGI(TAG, "TX reply: '%s'", reply.c_str());
      } else {
        ESP_LOGE(TAG, "TX reply failed: %s", tx_ec.message().c_str());
      }
    }
  });

  // 6. Hook up DIO1 interrupt and enter continuous receive mode
  s_irq_task_handle = xTaskGetCurrentTaskHandle();
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_DIO1, dio1_isr_handler, nullptr));

  if (!radio.start_receive(ec)) {
    ESP_LOGE(TAG, "Failed to start RX: %s", ec.message().c_str());
    return;
  }
  ESP_LOGI(TAG, "SX1262 ready and listening...");

  // 7. Main loop: service DIO1 interrupts and send a PING every 5 seconds
  uint32_t counter = 0;
  auto last_ping = std::chrono::steady_clock::now();

  while (true) {
    // Wait up to 100 ms for a DIO1 interrupt notification
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) > 0) {
      std::error_code ec;
      radio.handle_dio1_interrupt(ec);
    }

    auto now = std::chrono::steady_clock::now();
    if (now - last_ping >= 5s) {
      last_ping = now;
      std::string ping = "PING #" + std::to_string(++counter);
      auto bytes = std::span<const uint8_t>(
          reinterpret_cast<const uint8_t *>(ping.data()), ping.size());

      if (radio.transmit(bytes, 2000ms, ec)) {
        ESP_LOGI(TAG, "TX: '%s'", ping.c_str());
      } else {
        ESP_LOGE(TAG, "TX failed: %s", ec.message().c_str());
      }
    }
  }
}

