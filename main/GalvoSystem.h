#pragma once

#include "driver/i2c_types.h"

class GalvoSystem final {
    public:
    i2c_master_dev_handle_t x_dac, y_dac;

    GalvoSystem(i2c_master_bus_handle_t bus);
};
