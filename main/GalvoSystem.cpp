#include "GalvoSystem.h"

#include <algorithm>
#include <cmath>

#include "esp_check.h"
#include "esp_log.h"

namespace galvo {
namespace {
constexpr const char *TAG = "galvo_system";
constexpr float kTwoPi = 6.28318530717958647692f;
}  // namespace

Position GalvoSystem::SpiralSweep::next() {
    if (radius_ > max_radius_) {
        radius_ = 0.0f;
    }

    const float angle = kTwoPi * static_cast<float>(step_) / static_cast<float>(steps_per_rev_);
    const Position p{
        std::clamp(0.5f + radius_ * std::cos(angle), 0.0f, 1.0f),
        std::clamp(0.5f + radius_ * std::sin(angle), 0.0f, 1.0f),
    };

    ++step_;
    if (step_ % steps_per_rev_ == 0) {
        radius_ += radius_step_;
    }
    return p;
}

void GalvoSystem::SpiralSweep::reset() {
    radius_ = 0.0f;
    step_ = 0;
}

GalvoSystem::~GalvoSystem() {
    deinit();
}

esp_err_t GalvoSystem::init(const bus::I2cBus &i2c_bus, const Config &cfg) {
    ESP_RETURN_ON_FALSE(i2c_bus.is_initialized(), ESP_ERR_INVALID_ARG, TAG, "I2C bus not initialized");

    deinit();

    mutex_ = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(mutex_ != nullptr, ESP_ERR_NO_MEM, TAG, "Failed to create mutex");

    dac::Mcp4725::Config x_cfg{
        .bus = i2c_bus.handle(),
        .address = cfg.x_dac_address,
        .vref_volts = cfg.vref_volts,
        .scl_speed_hz = cfg.scl_speed_hz,
        .timeout_ms = cfg.timeout_ms,
    };
    ESP_RETURN_ON_ERROR(x_dac_.init(x_cfg), TAG, "X DAC init failed");

    dac::Mcp4725::Config y_cfg{
        .bus = i2c_bus.handle(),
        .address = cfg.y_dac_address,
        .vref_volts = cfg.vref_volts,
        .scl_speed_hz = cfg.scl_speed_hz,
        .timeout_ms = cfg.timeout_ms,
    };
    esp_err_t err = y_dac_.init(y_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Y DAC init failed: %s", esp_err_to_name(err));
        deinit();
        return err;
    }

    if (cfg.center_on_init) {
        ESP_RETURN_ON_ERROR(center(), TAG, "Failed to center mirrors on init");
    }

    return ESP_OK;
}

void GalvoSystem::deinit() {
    y_dac_.deinit();
    x_dac_.deinit();
    if (mutex_ != nullptr) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

esp_err_t GalvoSystem::center() {
    return set_position(Position{0.5f, 0.5f});
}

esp_err_t GalvoSystem::set_position(Position pos) {
    ESP_RETURN_ON_FALSE(is_ready(), ESP_ERR_INVALID_STATE, TAG, "GalvoSystem not initialized");
    ESP_RETURN_ON_FALSE(std::isfinite(pos.x) && std::isfinite(pos.y) &&
                            pos.x >= 0.0f && pos.x <= 1.0f &&
                            pos.y >= 0.0f && pos.y <= 1.0f,
                        ESP_ERR_INVALID_ARG, TAG, "Position out of range [0, 1]");

    ESP_RETURN_ON_FALSE(xSemaphoreTake(mutex_, pdMS_TO_TICKS(100)) == pdTRUE,
                        ESP_ERR_TIMEOUT, TAG, "Mutex timeout");

    esp_err_t err = x_dac_.set_fraction(pos.x);
    if (err == ESP_OK) {
        err = y_dac_.set_fraction(pos.y);
    }
    if (err == ESP_OK) {
        pos_ = pos;
    }

    xSemaphoreGive(mutex_);
    return err;
}

esp_err_t GalvoSystem::nudge(float dx, float dy) {
    ESP_RETURN_ON_FALSE(std::isfinite(dx) && std::isfinite(dy), ESP_ERR_INVALID_ARG, TAG, "Non-finite delta");
    const Position target{
        std::clamp(pos_.x + dx, 0.0f, 1.0f),
        std::clamp(pos_.y + dy, 0.0f, 1.0f),
    };
    return set_position(target);
}

}  // namespace galvo
