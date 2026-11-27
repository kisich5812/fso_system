#include "Mcp4725.h"

#include <cmath>
#include <utility>
#include "esp_check.h"
#include "esp_log.h"

namespace dac {
namespace {
constexpr const char *TAG = "mcp4725";
}  // namespace

Mcp4725::~Mcp4725() {
    deinit();
}

Mcp4725::Mcp4725(Mcp4725 &&other) noexcept
    : dev_(std::exchange(other.dev_, nullptr)),
      vref_volts_(other.vref_volts_),
      timeout_ms_(other.timeout_ms_),
      last_code_(other.last_code_) {}

Mcp4725 &Mcp4725::operator=(Mcp4725 &&other) noexcept {
    if (this != &other) {
        deinit();
        dev_ = std::exchange(other.dev_, nullptr);
        vref_volts_ = other.vref_volts_;
        timeout_ms_ = other.timeout_ms_;
        last_code_ = other.last_code_;
    }
    return *this;
}

esp_err_t Mcp4725::init(const Config &cfg) {
    ESP_RETURN_ON_FALSE(cfg.bus != nullptr, ESP_ERR_INVALID_ARG, TAG, "I2C bus is null");
    ESP_RETURN_ON_FALSE(std::isfinite(cfg.vref_volts) && cfg.vref_volts > 0.0f,
                        ESP_ERR_INVALID_ARG, TAG, "Invalid vref");

    deinit();

    ESP_RETURN_ON_ERROR(i2c_master_probe(cfg.bus, cfg.address, cfg.timeout_ms),
                        TAG, "No MCP4725 ACK at 0x%02X", cfg.address);

    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = cfg.address;
    dev_cfg.scl_speed_hz = cfg.scl_speed_hz;

    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(cfg.bus, &dev_cfg, &dev_),
                        TAG, "Failed to add MCP4725 at 0x%02X", cfg.address);

    vref_volts_ = cfg.vref_volts;
    timeout_ms_ = cfg.timeout_ms;
    return ESP_OK;
}

void Mcp4725::deinit() {
    if (dev_ != nullptr) {
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
    }
}

esp_err_t Mcp4725::set_code(uint16_t code, PowerMode mode) {
    ESP_RETURN_ON_FALSE(dev_ != nullptr, ESP_ERR_INVALID_STATE, TAG, "DAC not initialized");
    ESP_RETURN_ON_FALSE(code <= kMaxCode, ESP_ERR_INVALID_ARG, TAG, "Code %u > %u", code, kMaxCode);

    const uint8_t pd_bits = static_cast<uint8_t>(mode) & 0x03;
    const uint8_t buf[2] = {
        static_cast<uint8_t>((pd_bits << 4) | ((code >> 8) & 0x0F)),
        static_cast<uint8_t>(code & 0xFF),
    };

    ESP_RETURN_ON_ERROR(i2c_master_transmit(dev_, buf, sizeof(buf), timeout_ms_),
                        TAG, "I2C transmit failed");
    last_code_ = code;
    return ESP_OK;
}

esp_err_t Mcp4725::set_fraction(float fraction, PowerMode mode) {
    ESP_RETURN_ON_FALSE(std::isfinite(fraction) && fraction >= 0.0f && fraction <= 1.0f,
                        ESP_ERR_INVALID_ARG, TAG, "Fraction out of range [0, 1]");
    const auto code = static_cast<uint16_t>(std::lround(fraction * static_cast<float>(kMaxCode)));
    return set_code(code, mode);
}

esp_err_t Mcp4725::set_voltage(float volts, PowerMode mode) {
    ESP_RETURN_ON_FALSE(std::isfinite(volts) && volts >= 0.0f && volts <= vref_volts_,
                        ESP_ERR_INVALID_ARG, TAG, "Voltage out of range [0, %.2f]", vref_volts_);
    return set_fraction(volts / vref_volts_, mode);
}

esp_err_t Mcp4725::read(Readback &out) const {
    ESP_RETURN_ON_FALSE(dev_ != nullptr, ESP_ERR_INVALID_STATE, TAG, "DAC not initialized");

    uint8_t rx[5] = {};
    ESP_RETURN_ON_ERROR(i2c_master_receive(dev_, rx, sizeof(rx), timeout_ms_),
                        TAG, "I2C receive failed");

    out.status = rx[0];
    out.dac_code = static_cast<uint16_t>((rx[1] << 4) | (rx[2] >> 4));
    out.eeprom_code = static_cast<uint16_t>(((rx[3] & 0x0F) << 8) | rx[4]);
    return ESP_OK;
}

}  // namespace dac
