#include "GalvoSystem.h"
#include "driver/gpio.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "i2c_bus.h"
#include "mcp4725.h"
#include "rmt_loopback.h"
#include <cmath>
#include <cstdlib>

extern "C" void app_main(void)
{
    start_rmt_loopback();
}
