#include "pmic.h"
#include <string.h>
#include <ctype.h>

extern I2C_HandleTypeDef hi2c4;   // ggf. anpassen
static I2C_HandleTypeDef *pmic_hi2c = &hi2c4;

// ------------------------------------------------------------
// interne Helpers
// ------------------------------------------------------------
static int str_ieq(const char *a, const char *b)
{
    if (!a || !b) return 0;
    while (*a && *b) {
        if (tolower((unsigned char)*a) != tolower((unsigned char)*b)) return 0;
        a++; b++;
    }
    return (*a == '\0' && *b == '\0');
}

static bool pmic_is_forbidden_write(uint8_t reg)
{
    // BUCK2 darf NIE geschrieben werden (STM32 Versorgung)
    switch (reg)
    {
        case PMIC_REG_BUCK2_CTRL:
        case PMIC_REG_BUCK2_CONF:
        case PMIC_REG_BUCK2_VOUT_1:
        case PMIC_REG_BUCK2_VOUT_2:
            return true;
        default:
            return false;
    }
}

// ---------- BUCK Code <-> mV (TPS6593 Buck Kapitel, stückweise Schritte) ----------
// 0.30..0.60V: 20mV steps  (0x00..0x0F)
// 0.60..1.10V:  5mV steps  (0x10..0x73)
// 1.10..1.66V: 10mV steps  (0x74..0xAB)
// 1.66..3.34V: 20mV steps  (0xAC..0xFF)
// (Grenz-Codes 0x0F, 0x73, 0xAB ergeben sich aus der Tabelle/Level-Definition.)
static bool buck_mv_to_code(uint16_t req_mv, uint8_t *out_code, uint16_t *out_applied_mv)
{
    if (!out_code) return false;

    // User-Wunsch: 500..3300mV. Wir clampen zusätzlich auf Buck-Fähigkeit (300..3340).
    if (req_mv < 300u) req_mv = 300u;
    if (req_mv > 3340u) req_mv = 3340u;

    uint16_t mv = req_mv;
    uint32_t code = 0;
    uint32_t applied = 0;

    if (mv <= 600u) {
        // 300 + code*20
        code = (uint32_t)(mv - 300u + 10u) / 20u;
        if (code > 0x0Fu) code = 0x0Fu;
        applied = 300u + code * 20u;
    } else if (mv <= 1100u) {
        // 600 + (code-0x0F)*5
        uint32_t steps = (uint32_t)(mv - 600u + 2u) / 5u;
        code = 0x0Fu + steps;
        if (code > 0x73u) code = 0x73u;
        applied = 600u + (code - 0x0Fu) * 5u;
    } else if (mv <= 1660u) {
        // 1100 + (code-0x73)*10
        uint32_t steps = (uint32_t)(mv - 1100u + 5u) / 10u;
        code = 0x73u + steps;
        if (code > 0xABu) code = 0xABu;
        applied = 1100u + (code - 0x73u) * 10u;
    } else {
        // 1660 + (code-0xAB)*20
        uint32_t steps = (uint32_t)(mv - 1660u + 10u) / 20u;
        code = 0xABu + steps;
        if (code > 0xFFu) code = 0xFFu;
        applied = 1660u + (code - 0xABu) * 20u;
    }

    *out_code = (uint8_t)code;
    if (out_applied_mv) *out_applied_mv = (uint16_t)applied;
    return true;
}

static uint16_t buck_code_to_mv(uint8_t code)
{
    if (code <= 0x0Fu) {
        return (uint16_t)(300u + (uint16_t)code * 20u);
    } else if (code <= 0x73u) {
        return (uint16_t)(600u + (uint16_t)(code - 0x0Fu) * 5u);
    } else if (code <= 0xABu) {
        return (uint16_t)(1100u + (uint16_t)(code - 0x73u) * 10u);
    } else {
        return (uint16_t)(1660u + (uint16_t)(code - 0xABu) * 20u);
    }
}

// ---------- LDO Code <-> mV ----------
// LDO1..3: VSET liegt in Bits[6:1], BYPASS ist Bit7 (beibehalten!)
// Praxis (AM62A NVM): 0x3A => 3.30V, 0x1C => 1.80V  :contentReference[oaicite:11]{index=11}
// -> lineares Raster 0.60..3.30V in 50mV steps, Codes 0x04..0x3A.
static bool ldo123_mv_to_vset(uint16_t req_mv, uint8_t *out_vset, uint16_t *out_applied_mv)
{
    if (!out_vset) return false;
    if (req_mv < 600u) req_mv = 600u;
    if (req_mv > 3300u) req_mv = 3300u;

    uint32_t steps = (uint32_t)(req_mv - 600u + 25u) / 50u; // runden
    uint32_t vset  = 0x04u + steps;                         // 0x04 => 0.60V
    if (vset > 0x3Au) vset = 0x3Au;

    uint32_t applied = 600u + (vset - 0x04u) * 50u;

    *out_vset = (uint8_t)vset;
    if (out_applied_mv) *out_applied_mv = (uint16_t)applied;
    return true;
}

static uint16_t ldo123_vset_to_mv(uint8_t vset)
{
    if (vset < 0x04u) vset = 0x04u;
    if (vset > 0x3Au) vset = 0x3Au;
    return (uint16_t)(600u + (uint16_t)(vset - 0x04u) * 50u);
}

// LDO4: VSET liegt in Bits[6:0] (kein BYPASS Bit) :contentReference[oaicite:12]{index=12}
// typisches Raster: 1.20..3.30V in 25mV steps, Codes 0x20..0x74.
static bool ldo4_mv_to_vset(uint16_t req_mv, uint8_t *out_vset, uint16_t *out_applied_mv)
{
    if (!out_vset) return false;
    if (req_mv < 1200u) req_mv = 1200u;
    if (req_mv > 3300u) req_mv = 3300u;

    uint32_t steps = (uint32_t)(req_mv - 1200u + 12u) / 25u;
    uint32_t vset  = 0x20u + steps; // 0x20 => 1.20V
    if (vset > 0x74u) vset = 0x74u;

    uint32_t applied = 1200u + (vset - 0x20u) * 25u;

    *out_vset = (uint8_t)vset;
    if (out_applied_mv) *out_applied_mv = (uint16_t)applied;
    return true;
}

static uint16_t ldo4_vset_to_mv(uint8_t vset)
{
    if (vset < 0x20u) vset = 0x20u;
    if (vset > 0x74u) vset = 0x74u;
    return (uint16_t)(1200u + (uint16_t)(vset - 0x20u) * 25u);
}

// ------------------------------------------------------------
// Rail-Descriptor
// ------------------------------------------------------------
typedef struct {
    const char *name;
    uint8_t ctrl_reg;
    uint8_t vout1_reg;   // BUCK: VOUT_1, LDO: VOUT
    uint8_t vout2_reg;   // BUCK: VOUT_2, LDO: =0
    uint8_t is_buck;
    uint8_t is_ldo4;
} pmic_rail_t;

static const pmic_rail_t g_rails[] = {
    { "buck1", PMIC_REG_BUCK1_CTRL, PMIC_REG_BUCK1_VOUT_1, PMIC_REG_BUCK1_VOUT_2, 1, 0 },
    { "buck3", PMIC_REG_BUCK3_CTRL, PMIC_REG_BUCK3_VOUT_1, PMIC_REG_BUCK3_VOUT_2, 1, 0 },
    { "buck4", PMIC_REG_BUCK4_CTRL, PMIC_REG_BUCK4_VOUT_1, PMIC_REG_BUCK4_VOUT_2, 1, 0 },
    { "buck5", PMIC_REG_BUCK5_CTRL, PMIC_REG_BUCK5_VOUT_1, PMIC_REG_BUCK5_VOUT_2, 1, 0 },

    { "ldo1",  PMIC_REG_LDO1_CTRL,  PMIC_REG_LDO1_VOUT,    0,                  0, 0 },
    { "ldo2",  PMIC_REG_LDO2_CTRL,  PMIC_REG_LDO2_VOUT,    0,                  0, 0 },
    { "ldo3",  PMIC_REG_LDO3_CTRL,  PMIC_REG_LDO3_VOUT,    0,                  0, 0 },
    { "ldo4",  PMIC_REG_LDO4_CTRL,  PMIC_REG_LDO4_VOUT,    0,                  0, 1 },
};

static const pmic_rail_t* rail_find(const char *rail)
{
    if (!rail) return NULL;
    for (unsigned i = 0; i < (sizeof(g_rails)/sizeof(g_rails[0])); i++) {
        if (str_ieq(rail, g_rails[i].name)) return &g_rails[i];
    }
    return NULL;
}

// ------------------------------------------------------------
// Low-Level I2C
// ------------------------------------------------------------
void PMIC_Init(I2C_HandleTypeDef *hi2c)
{
    if (hi2c) pmic_hi2c = hi2c;
}

HAL_StatusTypeDef PMIC_Ping(void)
{
    if (!pmic_hi2c) return HAL_ERROR;

    for (uint32_t i = 0; i < 3; i++)
    {
        HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(
            pmic_hi2c, PMIC_I2C_ADDR, 2, PMIC_I2C_TIMEOUT_MS);
        if (st == HAL_OK) return HAL_OK;
        HAL_Delay(5);
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef PMIC_ReadReg(uint8_t reg, uint8_t *value)
{
    if (!pmic_hi2c || !value) return HAL_ERROR;

    return HAL_I2C_Mem_Read(
        pmic_hi2c, PMIC_I2C_ADDR, reg,
        I2C_MEMADD_SIZE_8BIT, value, 1, PMIC_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef PMIC_WriteReg(uint8_t reg, uint8_t value)
{
    if (!pmic_hi2c) return HAL_ERROR;

    if (pmic_is_forbidden_write(reg)) {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Write(
        pmic_hi2c, PMIC_I2C_ADDR, reg,
        I2C_MEMADD_SIZE_8BIT, &value, 1, PMIC_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef PMIC_I2C_Scan(uint8_t *out_addrs, uint32_t max_addrs, uint32_t *out_count)
{
    if (!pmic_hi2c || !out_addrs || !out_count) return HAL_ERROR;

    uint32_t cnt = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++)
    {
        if (HAL_I2C_IsDeviceReady(pmic_hi2c, (uint16_t)(addr << 1), 1, 5) == HAL_OK)
        {
            if (cnt < max_addrs) out_addrs[cnt] = addr;
            cnt++;
        }
    }
    *out_count = cnt;
    return HAL_OK;
}

// ------------------------------------------------------------
// LOCK / UNLOCK :contentReference[oaicite:13]{index=13}
// ------------------------------------------------------------
HAL_StatusTypeDef PMIC_IsUnlocked(uint8_t *locked)
{
    if (!locked) return HAL_ERROR;
    uint8_t st = 0;
    if (PMIC_ReadReg(PMIC_REG_REGISTER_LOCK, &st) != HAL_OK) return HAL_ERROR;
    *locked = (uint8_t)(st & 0x01u);
    return HAL_OK;
}

HAL_StatusTypeDef PMIC_UnlockUserRegs(void)
{
    // write 0x9B => unlock; read bit0 shows status :contentReference[oaicite:14]{index=14}
    if (PMIC_WriteReg(PMIC_REG_REGISTER_LOCK, PMIC_REGISTER_UNLOCK_KEY) != HAL_OK)
        return HAL_ERROR;

    uint8_t locked = 1;
    if (PMIC_IsUnlocked(&locked) != HAL_OK) return HAL_ERROR;

    return (locked == 0u) ? HAL_OK : HAL_ERROR;
}

// ------------------------------------------------------------
// High-Level Rails
// ------------------------------------------------------------
HAL_StatusTypeDef PMIC_SetRailEnable(const char *rail, uint8_t enable)
{
    const pmic_rail_t *r = rail_find(rail);
    if (!r) return HAL_ERROR;

    // optional unlock (wenn bei dir aktiv)
    (void)PMIC_UnlockUserRegs();

    uint8_t ctrl = 0;
    if (PMIC_ReadReg(r->ctrl_reg, &ctrl) != HAL_OK) return HAL_ERROR;

    if (r->is_buck) {
        if (enable) ctrl |= PMIC_BUCK_CTRL_EN_BIT;
        else        ctrl &= (uint8_t)~PMIC_BUCK_CTRL_EN_BIT;
    } else {
        if (enable) ctrl |= PMIC_LDO_CTRL_EN_BIT;
        else        ctrl &= (uint8_t)~PMIC_LDO_CTRL_EN_BIT;
    }

    return PMIC_WriteReg(r->ctrl_reg, ctrl);
}

HAL_StatusTypeDef PMIC_SetRail_mV(const char *rail, uint16_t req_mv, uint16_t *applied_mv)
{
    const pmic_rail_t *r = rail_find(rail);
    if (!r) return HAL_ERROR;

    // BUCK2 ist absichtlich nicht in der Tabelle; rail_find würde NULL liefern.
    // Trotzdem extra Absicherung (falls jemand "buck2" als alias addet).
    if (str_ieq(rail, "buck2")) return HAL_ERROR;

    // optional unlock (wenn bei dir aktiv)
    (void)PMIC_UnlockUserRegs();

    if (r->is_buck) {
        // User-Wunsch: 500..3300
        if (req_mv < 500u || req_mv > 3300u) return HAL_ERROR;

        uint8_t code = 0;
        uint16_t applied = 0;
        if (!buck_mv_to_code(req_mv, &code, &applied)) return HAL_ERROR;

        // Robust: immer beide VOUT-Banken setzen (VSEL egal)
        if (PMIC_WriteReg(r->vout1_reg, code) != HAL_OK) return HAL_ERROR;
        if (PMIC_WriteReg(r->vout2_reg, code) != HAL_OK) return HAL_ERROR;

        if (applied_mv) *applied_mv = applied;
        return HAL_OK;
    }

    // LDOs
    if (req_mv < 500u || req_mv > 3300u) return HAL_ERROR;

    uint8_t regv = 0;
    if (PMIC_ReadReg(r->vout1_reg, &regv) != HAL_OK) return HAL_ERROR;

    if (r->is_ldo4) {
        uint8_t vset = 0;
        uint16_t applied = 0;
        if (!ldo4_mv_to_vset(req_mv, &vset, &applied)) return HAL_ERROR;

        // LDO4_VOUT: bits[6:0] VSET, bit7 reserved :contentReference[oaicite:15]{index=15}
        regv = (uint8_t)((regv & 0x80u) | (vset & 0x7Fu));
        if (PMIC_WriteReg(r->vout1_reg, regv) != HAL_OK) return HAL_ERROR;

        if (applied_mv) *applied_mv = applied;
        return HAL_OK;
    } else {
        uint8_t vset = 0;
        uint16_t applied = 0;
        if (!ldo123_mv_to_vset(req_mv, &vset, &applied)) return HAL_ERROR;

        // LDO1..3_VOUT: bit7=BYPASS, bits[6:1]=VSET, bit0 reserved
        // VSET liegt als "Code" (0x04..0x3A) in bits[6:1] => <<1
        regv = (uint8_t)((regv & 0x80u) | ((vset & 0x3Fu) << 1));
        if (PMIC_WriteReg(r->vout1_reg, regv) != HAL_OK) return HAL_ERROR;

        if (applied_mv) *applied_mv = applied;
        return HAL_OK;
    }
}

HAL_StatusTypeDef PMIC_GetRailStatus(const char *rail,
                                     uint8_t *en,
                                     uint8_t *vsel,
                                     uint8_t *vout1_code,
                                     uint8_t *vout2_code,
                                     uint16_t *active_mv)
{
    const pmic_rail_t *r = rail_find(rail);
    if (!r) return HAL_ERROR;

    uint8_t ctrl = 0;
    if (PMIC_ReadReg(r->ctrl_reg, &ctrl) != HAL_OK) return HAL_ERROR;

    if (r->is_buck) {
        uint8_t c1 = 0, c2 = 0;
        if (PMIC_ReadReg(r->vout1_reg, &c1) != HAL_OK) return HAL_ERROR;
        if (PMIC_ReadReg(r->vout2_reg, &c2) != HAL_OK) return HAL_ERROR;

        if (en)   *en   = (uint8_t)((ctrl & PMIC_BUCK_CTRL_EN_BIT) ? 1u : 0u);
        if (vsel) *vsel = (uint8_t)((ctrl & PMIC_BUCK_CTRL_VSEL_BIT) ? 1u : 0u);
        if (vout1_code) *vout1_code = c1;
        if (vout2_code) *vout2_code = c2;

        uint8_t sel = (uint8_t)((ctrl & PMIC_BUCK_CTRL_VSEL_BIT) ? 1u : 0u);
        uint8_t active_code = sel ? c2 : c1;
        if (active_mv) *active_mv = buck_code_to_mv(active_code);

        return HAL_OK;
    } else {
        uint8_t rv = 0;
        if (PMIC_ReadReg(r->vout1_reg, &rv) != HAL_OK) return HAL_ERROR;

        if (en)   *en   = (uint8_t)((ctrl & PMIC_LDO_CTRL_EN_BIT) ? 1u : 0u);
        if (vsel) *vsel = 0;

        if (r->is_ldo4) {
            uint8_t vset = (uint8_t)(rv & 0x7Fu);
            if (vout1_code) *vout1_code = vset;
            if (vout2_code) *vout2_code = vset;
            if (active_mv)  *active_mv  = ldo4_vset_to_mv(vset);
        } else {
            uint8_t vset = (uint8_t)((rv >> 1) & 0x3Fu);
            if (vout1_code) *vout1_code = vset;
            if (vout2_code) *vout2_code = vset;
            if (active_mv)  *active_mv  = ldo123_vset_to_mv(vset);
        }
        return HAL_OK;
    }
}

// ------------------------------------------------------------
// Komfort BUCK1
// ------------------------------------------------------------
HAL_StatusTypeDef PMIC_Buck1_Set_mV(uint16_t mv, uint16_t *applied_mv)
{
    return PMIC_SetRail_mV("buck1", mv, applied_mv);
}

HAL_StatusTypeDef PMIC_Buck1_Set_500mV(void)
{
    uint16_t dummy = 0;
    return PMIC_Buck1_Set_mV(500u, &dummy);
}

HAL_StatusTypeDef PMIC_Buck1_Set_1000mV(void)
{
    uint16_t dummy = 0;
    return PMIC_Buck1_Set_mV(1000u, &dummy);
}
