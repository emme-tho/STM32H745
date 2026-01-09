#ifndef SETUP_UTILS_H_
#define SETUP_UTILS_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"

HAL_StatusTypeDef setup_set_voltage(
    const char *rail,
    uint16_t mv
);

HAL_StatusTypeDef setup_disable_rail(
    const char *rail
);

#endif /* SETUP_UTILS_H_ */
