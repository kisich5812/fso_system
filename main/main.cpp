#include "dac_setup.h"
#include "rmt_loopback.h"

extern "C" void app_main(void)
{
    //initialize_dac();
    start_rmt_loopback();
}
