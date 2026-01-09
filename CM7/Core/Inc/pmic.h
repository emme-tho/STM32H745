#ifndef INC_PMIC_H_
#define INC_PMIC_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// ============================================================
// Konfiguration
// ============================================================

// Deine funktionierende 7-bit I2C-Adresse:
#ifndef PMIC_I2C_ADDR_7BIT
#define PMIC_I2C_ADDR_7BIT        (0x48u)
#endif

#define PMIC_I2C_ADDR             ((uint16_t)(PMIC_I2C_ADDR_7BIT << 1))

#ifndef PMIC_I2C_TIMEOUT_MS
#define PMIC_I2C_TIMEOUT_MS       (100u)
#endif

// ============================================================
// Register Offsets (TPS6593-Q1)
// (siehe Register-Map)  :contentReference[oaicite:4]{index=4}
// ============================================================

// BUCK CTRL/CONF
#define PMIC_REG_BUCK1_CTRL       (0x04u)
#define PMIC_REG_BUCK1_CONF       (0x05u)
#define PMIC_REG_BUCK2_CTRL       (0x06u)  // STM32 Versorgung -> NICHT anfassen
#define PMIC_REG_BUCK2_CONF       (0x07u)  // STM32 Versorgung -> NICHT anfassen
#define PMIC_REG_BUCK3_CTRL       (0x08u)
#define PMIC_REG_BUCK3_CONF       (0x09u)
#define PMIC_REG_BUCK4_CTRL       (0x0Au)
#define PMIC_REG_BUCK4_CONF       (0x0Bu)
#define PMIC_REG_BUCK5_CTRL       (0x0Cu)
#define PMIC_REG_BUCK5_CONF       (0x0Du)

// BUCK VOUT (jeweils 8-bit pro Bank)
#define PMIC_REG_BUCK1_VOUT_1     (0x0Eu)
#define PMIC_REG_BUCK1_VOUT_2     (0x0Fu)
#define PMIC_REG_BUCK2_VOUT_1     (0x10u)  // STM32 Versorgung -> NICHT anfassen
#define PMIC_REG_BUCK2_VOUT_2     (0x11u)  // STM32 Versorgung -> NICHT anfassen
#define PMIC_REG_BUCK3_VOUT_1     (0x12u)
#define PMIC_REG_BUCK3_VOUT_2     (0x13u)
#define PMIC_REG_BUCK4_VOUT_1     (0x14u)
#define PMIC_REG_BUCK4_VOUT_2     (0x15u)
#define PMIC_REG_BUCK5_VOUT_1     (0x16u)
#define PMIC_REG_BUCK5_VOUT_2     (0x17u)

// LDO CTRL/VOUT
#define PMIC_REG_LDO1_CTRL        (0x1Du)
#define PMIC_REG_LDO2_CTRL        (0x1Eu)
#define PMIC_REG_LDO3_CTRL        (0x1Fu)
#define PMIC_REG_LDO4_CTRL        (0x20u)

#define PMIC_REG_LDO1_VOUT        (0x23u)
#define PMIC_REG_LDO2_VOUT        (0x24u)
#define PMIC_REG_LDO3_VOUT        (0x25u)
#define PMIC_REG_LDO4_VOUT        (0x26u)

// REGISTER_LOCK :contentReference[oaicite:7]{index=7}
#define PMIC_REG_REGISTER_LOCK    (0xA1u)
#define PMIC_REGISTER_UNLOCK_KEY  (0x9Bu)

// Bitfields (BUCKx_CTRL) :contentReference[oaicite:8]{index=8}
#define PMIC_BUCK_CTRL_EN_BIT     (1u << 0)
#define PMIC_BUCK_CTRL_VSEL_BIT   (1u << 3)

// Bitfields (LDOx_CTRL): EN ist Bit0
#define PMIC_LDO_CTRL_EN_BIT      (1u << 0)

// ============================================================
// API
// ============================================================

void PMIC_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef PMIC_Ping(void);

HAL_StatusTypeDef PMIC_ReadReg(uint8_t reg, uint8_t *value);
HAL_StatusTypeDef PMIC_WriteReg(uint8_t reg, uint8_t value);

// Scan: sammelt gefundene 7-bit Adressen in Liste
HAL_StatusTypeDef PMIC_I2C_Scan(uint8_t *out_addrs, uint32_t max_addrs, uint32_t *out_count);

// Optional: Unlock (falls LOCK aktiv)
HAL_StatusTypeDef PMIC_UnlockUserRegs(void);
HAL_StatusTypeDef PMIC_IsUnlocked(uint8_t *locked);

// High-Level Rails (BUCK2 ist absichtlich nicht enthalten)
HAL_StatusTypeDef PMIC_SetRailEnable(const char *rail, uint8_t enable);
HAL_StatusTypeDef PMIC_SetRail_mV(const char *rail, uint16_t req_mv, uint16_t *applied_mv);
HAL_StatusTypeDef PMIC_GetRailStatus(const char *rail,
                                     uint8_t *en,
                                     uint8_t *vsel,
                                     uint8_t *vout1_code,
                                     uint8_t *vout2_code,
                                     uint16_t *active_mv);

// Komfort: BUCK1
HAL_StatusTypeDef PMIC_Buck1_Set_mV(uint16_t mv, uint16_t *applied_mv);
HAL_StatusTypeDef PMIC_Buck1_Set_500mV(void);
HAL_StatusTypeDef PMIC_Buck1_Set_1000mV(void);


HAL_StatusTypeDef PMIC_ApplyRail_mV(
    const char *rail,
    uint16_t mv,
    uint16_t min_mv,
    uint16_t max_mv,
    uint16_t *applied_mv
);

HAL_StatusTypeDef PMIC_DisableRail(const char *rail);

#endif /* INC_PMIC_H_ */
