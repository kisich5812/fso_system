#pragma once

#include <cstdint>
#include "driver/i2c_master.h"
#include "esp_err.h"

namespace dac {

class Mcp4725 final {
public:
    static constexpr uint16_t kMaxCode = 4095;
    static constexpr uint16_t kMidCode = 2048;
    static constexpr uint32_t kDefaultClockHz = 400'000;
    static constexpr int kDefaultTimeoutMs = 100;

    enum class PowerMode : uint8_t {
        Normal = 0b00,
        PullDown1k = 0b01,
        PullDown100k = 0b10,
        PullDown500k = 0b11,
    };

    struct Config {
        i2c_master_bus_handle_t bus = nullptr;
        uint8_t address = 0x60;
        float vref_volts = 3.3f;
        uint32_t scl_speed_hz = kDefaultClockHz;
        int timeout_ms = kDefaultTimeoutMs;
    };

    struct Readback {
        uint8_t status = 0;       // bit 7: RDY/BSY, bit 6: POR
        uint16_t dac_code = 0;    // 0..4095
        uint16_t eeprom_code = 0; // 0..4095
    };

    Mcp4725() = default;
    ~Mcp4725();

    Mcp4725(const Mcp4725 &) = delete;
    Mcp4725 &operator=(const Mcp4725 &) = delete;
    Mcp4725(Mcp4725 &&other) noexcept;
    Mcp4725 &operator=(Mcp4725 &&other) noexcept;

    [[nodiscard]] esp_err_t init(const Config &cfg);
    void deinit();

    bool is_ready() const { return dev_ != nullptr; }
    uint16_t last_code() const { return last_code_; }
    float vref_volts() const { return vref_volts_; }

    // Fast write: updates DAC register only (does not wear out EEPROM).
    [[nodiscard]] esp_err_t set_code(uint16_t code, PowerMode mode = PowerMode::Normal);
    [[nodiscard]] esp_err_t set_fraction(float fraction, PowerMode mode = PowerMode::Normal);
    [[nodiscard]] esp_err_t set_voltage(float volts, PowerMode mode = PowerMode::Normal);

    // Reads back status, active DAC register, and EEPROM register.
    [[nodiscard]] esp_err_t read(Readback &out) const;

private:
    i2c_master_dev_handle_t dev_ = nullptr;
    float vref_volts_ = 3.3f;
    int timeout_ms_ = kDefaultTimeoutMs;
    uint16_t last_code_ = 0;
};

}
