#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "I2cBus.h"
#include "Mcp4725.h"

namespace galvo {

struct Position {
    float x = 0.5f;  // Normalized [0.0f, 1.0f], 0.5f = center
    float y = 0.5f;  // Normalized [0.0f, 1.0f], 0.5f = center
};

class GalvoSystem final {
public:
    struct Config {
        uint8_t x_dac_address = 0x61;
        uint8_t y_dac_address = 0x60;
        float vref_volts = 3.3f;
        uint32_t scl_speed_hz = dac::Mcp4725::kDefaultClockHz;
        int timeout_ms = dac::Mcp4725::kDefaultTimeoutMs;
        bool center_on_init = true;
    };

    // Stateful helper for outward spiral sweeps without polluting task loops.
    class SpiralSweep {
    public:
        explicit SpiralSweep(size_t steps_per_rev = 360, float radius_step = 0.01f,
                             float max_radius = 0.5f)
            : steps_per_rev_(steps_per_rev > 0 ? steps_per_rev : 360),
              radius_step_(radius_step),
              max_radius_(max_radius) {}

        Position next();
        void reset();

        float radius() const { return radius_; }
        uint32_t step() const { return step_; }

    private:
        size_t steps_per_rev_;
        float radius_step_;
        float max_radius_;
        float radius_ = 0.0f;
        uint32_t step_ = 0;
    };

    GalvoSystem() = default;
    ~GalvoSystem();

    GalvoSystem(const GalvoSystem &) = delete;
    GalvoSystem &operator=(const GalvoSystem &) = delete;

    [[nodiscard]] esp_err_t init(const bus::I2cBus &i2c_bus, const Config &cfg);
    void deinit();

    bool is_ready() const { return x_dac_.is_ready() && y_dac_.is_ready(); }
    Position position() const { return pos_; }

    [[nodiscard]] esp_err_t center();
    [[nodiscard]] esp_err_t set_position(Position pos);
    [[nodiscard]] esp_err_t set_position(float x, float y) { return set_position(Position{x, y}); }

    // Applies a signed delta (clamped to [0.0f, 1.0f]) — ideal for LoRa alignment nudges.
    [[nodiscard]] esp_err_t nudge(float dx, float dy);

private:
    dac::Mcp4725 x_dac_;
    dac::Mcp4725 y_dac_;
    SemaphoreHandle_t mutex_ = nullptr;
    Position pos_{};
};

}  // namespace galvo
