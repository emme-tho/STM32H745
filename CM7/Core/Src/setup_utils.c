#include "setup_utils.h"
#include "pmic.h"
#include "cli.h"

HAL_StatusTypeDef setup_set_voltage(const char *rail, uint16_t mv)
{
    if (!rail) return HAL_ERROR;

    if (mv < 500u || mv > 3300u) {
           cli_printf("\r\nBereich: 500..3300 mV\r\n");
           return;
       }

    uint16_t applied = 0;
    if (PMIC_SetRail_mV(rail, mv, &applied) != HAL_OK) {
        cli_printf("\r\n%s set %umV FEHLER\r\n", (unsigned)mv);
        return HAL_ERROR;
    }

    if (PMIC_SetRailEnable(rail, 1u) != HAL_OK) {
        cli_printf("\r\n%s enable FEHLER\r\n");
        return HAL_ERROR;
    }


    cli_printf("\r\n%s: request %umV -> applied %umV, EN=1\r\n",
                   rail, (unsigned)mv, (unsigned)applied);
    return HAL_OK;
}

HAL_StatusTypeDef setup_disable_rail(const char *rail)
{
    if (!rail) return HAL_ERROR;

    if (PMIC_SetRailEnable(rail, 0u) != HAL_OK) {
        cli_printf("\r\n%s disable FEHLER\r\n", rail);
        return HAL_ERROR;
    }

    cli_printf("\r\n%s: EN=0\r\n", rail);
    return HAL_OK;
}
