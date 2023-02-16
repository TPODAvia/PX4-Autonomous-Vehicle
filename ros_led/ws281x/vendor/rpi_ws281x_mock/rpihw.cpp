#include "rpihw.h"

#define PERIPH_BASE_RPI2                         0x3f000000
#define VIDEOCORE_BASE_RPI2                      0xc0000000

static const rpi_hw_t mock_rpi_hw {
    RPI_HWVER_TYPE_PI2,
    0xa02082,
    PERIPH_BASE_RPI2,
    VIDEOCORE_BASE_RPI2,
    "Mock RPi 3"
};

const rpi_hw_t *rpi_hw_detect(void)
{
    return &mock_rpi_hw;
}
