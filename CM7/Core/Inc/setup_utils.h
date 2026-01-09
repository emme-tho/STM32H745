#ifndef SETUP_UTILS_H_
#define SETUP_UTILS_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"

HAL_StatusTypeDef setup_set_voltage(
    const char *rail,
    uint16_t mv,
    uint16_t min_mv,
    uint16_t max_mv,
    const char *label,        // z.B. "LDO1", "BUCK3"
    uint16_t *applied_out     // optional, kann NULL sein
);

HAL_StatusTypeDef setup_disable_rail(
    const char *rail,
    const char *label
);

#endif /* SETUP_UTILS_H_ */
