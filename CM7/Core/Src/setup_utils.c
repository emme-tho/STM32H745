#include "setup_utils.h"
#include "pmic.h"
#include "cli.h"

HAL_StatusTypeDef setup_set_voltage(
    const char *rail,
    uint16_t mv,
    uint16_t min_mv,
    uint16_t max_mv,
    const char *label,
    uint16_t *applied_out
)
{
    if (!rail) return HAL_ERROR;
    if (!label) label = rail;

    if (mv < min_mv || mv > max_mv) {
        cli_printf("\r\nBereich: %u..%u mV\r\n", (unsigned)min_mv, (unsigned)max_mv);
        return HAL_ERROR;
    }

    uint16_t applied = 0;
    if (PMIC_SetRail_mV(rail, mv, &applied) != HAL_OK) {
        cli_printf("\r\n%s set %umV FEHLER\r\n", label, (unsigned)mv);
        return HAL_ERROR;
    }

    if (PMIC_SetRailEnable(rail, 1u) != HAL_OK) {
        cli_printf("\r\n%s enable FEHLER\r\n", label);
        return HAL_ERROR;
    }

    cli_printf("\r\n%s: request %umV -> applied %umV, EN=1\r\n",
               label, (unsigned)mv, (unsigned)applied);

    if (applied_out) *applied_out = applied;
    return HAL_OK;
}

HAL_StatusTypeDef setup_disable_rail(const char *rail, const char *label)
{
    if (!rail) return HAL_ERROR;
    if (!label) label = rail;

    if (PMIC_SetRailEnable(rail, 0u) != HAL_OK) {
        cli_printf("\r\n%s disable FEHLER\r\n", label);
        return HAL_ERROR;
    }

    cli_printf("\r\n%s: EN=0\r\n", label);
    return HAL_OK;
}
