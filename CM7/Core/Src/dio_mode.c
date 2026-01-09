#include "dio_mode.h"
#include "cli.h"
#include "hexstream.h"
#include "pmic.h"
#include "usbd_cdc_if.h"   // CDC_Transmit_HS

#include "main.h"
#include "stm32h7xx_hal.h"

#include <string.h>
#include <stdlib.h>

// ============================================================
// DIO MODE (8x Digital Out, 8x Digital In)
//
// Stream (ohne Enter):
//   wp      -> Readback OUT+IN (OUT bleibt unverändert)
//   w<OO>p  -> Set OUT Byte + Readback
//
// Readback: 4 Hex-Zeichen: OUT(2) + IN(2) + \r\n
//
// Setup (s):
//   - BUCK3 Voltage/Enable  (Digital OUT IV Block)
//   - BUCK4 Voltage/Enable  (Digital IN  IV Block)
//   - Buffer OE: DO_BUF_EN_N / DI_BUF_EN_N (active low) optional
// ============================================================

// ---------------- Pin maps (bit0..bit7) ----------------
static GPIO_TypeDef* const g_do_port[8] = {
    Digital_OUT_0_GPIO_Port, Digital_OUT_1_GPIO_Port, Digital_OUT_2_GPIO_Port, Digital_OUT_3_GPIO_Port,
    Digital_OUT_4_GPIO_Port, Digital_OUT_5_GPIO_Port, Digital_OUT_6_GPIO_Port, Digital_OUT_7_GPIO_Port,
};
static const uint16_t g_do_pin[8] = {
    Digital_OUT_0_Pin, Digital_OUT_1_Pin, Digital_OUT_2_Pin, Digital_OUT_3_Pin,
    Digital_OUT_4_Pin, Digital_OUT_5_Pin, Digital_OUT_6_Pin, Digital_OUT_7_Pin,
};

static GPIO_TypeDef* const g_di_port[8] = {
    Digital_IN_0_GPIO_Port, Digital_IN_1_GPIO_Port, Digital_IN_2_GPIO_Port, Digital_IN_3_GPIO_Port,
    Digital_IN_4_GPIO_Port, Digital_IN_5_GPIO_Port, Digital_IN_6_GPIO_Port, Digital_IN_7_GPIO_Port,
};
static const uint16_t g_di_pin[8] = {
    Digital_IN_0_Pin, Digital_IN_1_Pin, Digital_IN_2_Pin, Digital_IN_3_Pin,
    Digital_IN_4_Pin, Digital_IN_5_Pin, Digital_IN_6_Pin, Digital_IN_7_Pin,
};

// ---------------- Write stream state ----------------
static uint8_t     ws_active = 0;
static hexstream_t ws_hex;

// Cached output byte (spec wants OUT state in high byte)
static uint8_t g_out_state = 0x00;

// ---------------- Setup state ----------------
typedef enum {
    DIO_SETUP_NONE = 0,
    DIO_SETUP_MAIN,
    DIO_SETUP_VOUT,   // BUCK3
    DIO_SETUP_VIN,    // BUCK4
    DIO_SETUP_BUFS,
} dio_setup_state_t;

static dio_setup_state_t g_setup_state = DIO_SETUP_NONE;

// Rail status cache
static uint16_t g_buck3_mv = 0;  // Digital OUT
static uint8_t  g_buck3_en = 0;

static uint16_t g_buck4_mv = 0;  // Digital IN
static uint8_t  g_buck4_en = 0;

// Buffer enables (best effort)
static uint8_t  g_do_buf_en = 1;
static uint8_t  g_di_buf_en = 1;

// ---------------- Helpers ----------------
static void dio_apply_outputs(uint8_t out)
{
    for (uint8_t i = 0; i < 8u; i++) {
        GPIO_PinState st = ((out >> i) & 1u) ? GPIO_PIN_SET : GPIO_PIN_RESET;
        HAL_GPIO_WritePin(g_do_port[i], g_do_pin[i], st);
    }
    g_out_state = out;
}

static uint8_t dio_read_inputs(void)
{
    uint8_t in = 0;
    for (uint8_t i = 0; i < 8u; i++) {
        if (HAL_GPIO_ReadPin(g_di_port[i], g_di_pin[i]) == GPIO_PIN_SET) {
            in |= (uint8_t)(1u << i);
        }
    }
    return in;
}

static uint8_t dio_read_outputs_best_effort(void)
{
    uint8_t out = 0;
    for (uint8_t i = 0; i < 8u; i++) {
        if (HAL_GPIO_ReadPin(g_do_port[i], g_do_pin[i]) == GPIO_PIN_SET) {
            out |= (uint8_t)(1u << i);
        }
    }
    return out;
}

static void dio_print_status(void)
{
    uint8_t out = g_out_state;
    uint8_t in  = dio_read_inputs();
    cli_printf("%02X%02X\r\n", out, in);
}

static void dio_refresh_rails(void)
{
    uint8_t en=0, vsel=0, c1=0, c2=0;
    uint16_t mv=0;

    if (PMIC_GetRailStatus("buck3", &en, &vsel, &c1, &c2, &mv) == HAL_OK) {
        g_buck3_en = en;
        g_buck3_mv = mv;
    } else {
        g_buck3_en = 0;
        g_buck3_mv = 0;
    }

    if (PMIC_GetRailStatus("buck4", &en, &vsel, &c1, &c2, &mv) == HAL_OK) {
        g_buck4_en = en;
        g_buck4_mv = mv;
    } else {
        g_buck4_en = 0;
        g_buck4_mv = 0;
    }
}

// ---------------- Buffer Enable (active low) ----------------
static uint8_t dio_has_do_oe(void)
{
#ifdef DO_BUF_EN_N_Pin
    return 1u;
#else
    return 0u;
#endif
}
static uint8_t dio_has_di_oe(void)
{
#ifdef DI_BUF_EN_N_Pin
    return 1u;
#else
    return 0u;
#endif
}

static uint8_t dio_read_do_buf_en(void)
{
#ifdef DO_BUF_EN_N_Pin
    return (HAL_GPIO_ReadPin(DO_BUF_EN_N_GPIO_Port, DO_BUF_EN_N_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
#else
    return 1u;
#endif
}
static uint8_t dio_read_di_buf_en(void)
{
#ifdef DI_BUF_EN_N_Pin
    return (HAL_GPIO_ReadPin(DI_BUF_EN_N_GPIO_Port, DI_BUF_EN_N_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
#else
    return 1u;
#endif
}

static void dio_set_do_buf_en(uint8_t en)
{
#ifdef DO_BUF_EN_N_Pin
    HAL_GPIO_WritePin(DO_BUF_EN_N_GPIO_Port, DO_BUF_EN_N_Pin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
    g_do_buf_en = en ? 1u : 0u;
#else
    (void)en;
#endif
}
static void dio_set_di_buf_en(uint8_t en)
{
#ifdef DI_BUF_EN_N_Pin
    HAL_GPIO_WritePin(DI_BUF_EN_N_GPIO_Port, DI_BUF_EN_N_Pin, en ? GPIO_PIN_RESET : GPIO_PIN_SET);
    g_di_buf_en = en ? 1u : 0u;
#else
    (void)en;
#endif
}

// ---------------- Setup UI ----------------
static void dio_print_setting_summary(void)
{
    dio_refresh_rails();

    cli_printf("  Digital OUT rail (BUCK3): ");
    if (g_buck3_mv == 0u) cli_printf("unknown");
    else cli_printf("%umV", (unsigned)g_buck3_mv);
    cli_printf("  EN=%u\r\n", (unsigned)g_buck3_en);

    cli_printf("  Digital IN  rail (BUCK4): ");
    if (g_buck4_mv == 0u) cli_printf("unknown");
    else cli_printf("%umV", (unsigned)g_buck4_mv);
    cli_printf("  EN=%u\r\n", (unsigned)g_buck4_en);

    cli_printf("  DO buffer (DO_BUF_EN_N): %s\r\n",
               dio_has_do_oe() ? (g_do_buf_en ? "EN" : "DIS") : "n/a");
    cli_printf("  DI buffer (DI_BUF_EN_N): %s\r\n",
               dio_has_di_oe() ? (g_di_buf_en ? "EN" : "DIS") : "n/a");
}

static void dio_setup_show_main(void)
{
    g_setup_state = DIO_SETUP_MAIN;

    g_do_buf_en = dio_read_do_buf_en();
    g_di_buf_en = dio_read_di_buf_en();

    cli_printf("\r\n[DIO Setup]\r\n");
    dio_print_setting_summary();
    cli_printf("\r\n");
    cli_printf("  1 - Voltage Digital OUT (BUCK3)\r\n");
    cli_printf("  2 - Voltage Digital IN  (BUCK4)\r\n");
    cli_printf("  3 - Buffers (DO/DI OE)\r\n");
    cli_printf("  q - back to DIO\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void dio_setup_show_vout(void)
{
    g_setup_state = DIO_SETUP_VOUT;
    dio_refresh_rails();

    cli_printf("\r\n[DIO Setup] Digital OUT (BUCK3)\r\n");
    cli_printf("Aktuell: ");
    if (g_buck3_mv == 0u) cli_printf("unknown");
    else cli_printf("%umV", (unsigned)g_buck3_mv);
    cli_printf("  EN=%u\r\n\r\n", (unsigned)g_buck3_en);

    cli_printf("  0 - OFF (disable BUCK3)\r\n");
    cli_printf("  1 - 800 mV\r\n");
    cli_printf("  2 - 1800 mV\r\n");
    cli_printf("  3 - 3300 mV\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void dio_setup_show_vin(void)
{
    g_setup_state = DIO_SETUP_VIN;
    dio_refresh_rails();

    cli_printf("\r\n[DIO Setup] Digital IN (BUCK4)\r\n");
    cli_printf("Aktuell: ");
    if (g_buck4_mv == 0u) cli_printf("unknown");
    else cli_printf("%umV", (unsigned)g_buck4_mv);
    cli_printf("  EN=%u\r\n\r\n", (unsigned)g_buck4_en);

    cli_printf("  0 - OFF (disable BUCK4)\r\n");
    cli_printf("  1 - 800 mV\r\n");
    cli_printf("  2 - 1800 mV\r\n");
    cli_printf("  3 - 3300 mV\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void dio_setup_show_bufs(void)
{
    g_setup_state = DIO_SETUP_BUFS;

    g_do_buf_en = dio_read_do_buf_en();
    g_di_buf_en = dio_read_di_buf_en();

    cli_printf("\r\n[DIO Setup] Buffers (active low)\r\n");
    cli_printf("  DO (DO_BUF_EN_N): %s\r\n", dio_has_do_oe() ? (g_do_buf_en ? "EN" : "DIS") : "n/a");
    cli_printf("  DI (DI_BUF_EN_N): %s\r\n\r\n", dio_has_di_oe() ? (g_di_buf_en ? "EN" : "DIS") : "n/a");

    cli_printf("  1 - DO enable\r\n");
    cli_printf("  2 - DO disable\r\n");
    cli_printf("  3 - DI enable\r\n");
    cli_printf("  4 - DI disable\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void dio_set_buck_voltage(const char *buck, uint16_t mv)
{
    uint16_t applied = 0;
    if (PMIC_SetRail_mV(buck, mv, &applied) != HAL_OK) {
        cli_printf("\r\n%s set %umV FEHLER\r\n", buck, (unsigned)mv);
        return;
    }
    if (PMIC_SetRailEnable(buck, 1u) != HAL_OK) {
        cli_printf("\r\n%s enable FEHLER\r\n", buck);
        return;
    }
    cli_printf("\r\n%s: request %umV -> applied %umV, EN=1\r\n",
               buck, (unsigned)mv, (unsigned)applied);
    dio_refresh_rails();
}

static void dio_disable_buck(const char *buck)
{
    if (PMIC_SetRailEnable(buck, 0u) != HAL_OK) {
        cli_printf("\r\n%s disable FEHLER\r\n", buck);
        return;
    }
    cli_printf("\r\n%s: EN=0\r\n", buck);
    dio_refresh_rails();
}

// ---------------- Help ----------------
static void dio_print_help(void)
{
    cli_printf("DIO Mode Befehle:\r\n");
    cli_printf("  s        - Setup\r\n");
    cli_printf("  wp       - Readback (OUT+IN)\r\n");
    cli_printf("  w<OO>p   - Set OUT byte + Readback\r\n");
    cli_printf("             Beispiel: wFFp -> alle OUT high\r\n");
    cli_printf("  help\r\n");
}

// ---------------- Public API ----------------
void DIO_Mode_Enter(void)
{
    ws_active = 0;
    HEXS_Init(&ws_hex);

    g_out_state = dio_read_outputs_best_effort();
    dio_refresh_rails();

    cli_printf("\r\n[DIO Mode]\r\n");
    cli_printf("  s        - Setup\r\n");
    cli_printf("  wp       - Readback (OUT+IN as 4 hex chars)\r\n");
    cli_printf("  w<OO>p   - Set OUT (1 byte) + Readback\r\n");
    cli_printf("  help     - diese Hilfe\r\n");
}

uint8_t DIO_Mode_HandleLine(char *line)
{
    if (!line) return 0;

    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return 1;

    if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
        dio_print_help();
        return 1;
    }

    if (strcmp(line, "s") == 0) {
        dio_setup_show_main();
        return 1;
    }

    // optional convenience "w FF" (mit Enter)
    if (line[0] == 'w' || line[0] == 'W') {
        char *p = line + 1;
        while (*p == ' ' || *p == '\t') p++;
        if (*p == '\0') {
            dio_print_status();
            return 1;
        }
        uint32_t v = strtoul(p, NULL, 16);
        dio_apply_outputs((uint8_t)v);
        dio_print_status();
        return 1;
    }

    return 0;
}

uint8_t DIO_Mode_HandleChar(char ch)
{
    // Setup / menu handling (wenn kein write-capture aktiv)
    if (!ws_active) {

        if (g_setup_state != DIO_SETUP_NONE) {
            if (g_setup_state == DIO_SETUP_MAIN) {
                if (ch == '1') { dio_setup_show_vout(); return 1; }
                if (ch == '2') { dio_setup_show_vin();  return 1; }
                if (ch == '3') { dio_setup_show_bufs(); return 1; }
                if (ch == 'q' || ch == 'Q') {
                    g_setup_state = DIO_SETUP_NONE;
                    cli_printf("\r\n(DIO setup closed)\r\n");
                    CLI_PrintPrompt();
                    return 1;
                }
                return 1;
            }

            if (g_setup_state == DIO_SETUP_VOUT) {
                if (ch == '0') { dio_disable_buck("buck3"); dio_setup_show_vout(); return 1; }
                if (ch == '1') { dio_set_buck_voltage("buck3", 800u);  dio_setup_show_vout(); return 1; }
                if (ch == '2') { dio_set_buck_voltage("buck3", 1800u); dio_setup_show_vout(); return 1; }
                if (ch == '3') { dio_set_buck_voltage("buck3", 3300u); dio_setup_show_vout(); return 1; }
                if (ch == 'q' || ch == 'Q') { dio_setup_show_main(); return 1; }
                return 1;
            }

            if (g_setup_state == DIO_SETUP_VIN) {
                if (ch == '0') { dio_disable_buck("buck4"); dio_setup_show_vin(); return 1; }
                if (ch == '1') { dio_set_buck_voltage("buck4", 800u);  dio_setup_show_vin(); return 1; }
                if (ch == '2') { dio_set_buck_voltage("buck4", 1800u); dio_setup_show_vin(); return 1; }
                if (ch == '3') { dio_set_buck_voltage("buck4", 3300u); dio_setup_show_vin(); return 1; }
                if (ch == 'q' || ch == 'Q') { dio_setup_show_main(); return 1; }
                return 1;
            }

            if (g_setup_state == DIO_SETUP_BUFS) {
                if (ch == '1') { dio_set_do_buf_en(1u); dio_setup_show_bufs(); return 1; }
                if (ch == '2') { dio_set_do_buf_en(0u); dio_setup_show_bufs(); return 1; }
                if (ch == '3') { dio_set_di_buf_en(1u); dio_setup_show_bufs(); return 1; }
                if (ch == '4') { dio_set_di_buf_en(0u); dio_setup_show_bufs(); return 1; }
                if (ch == 'q' || ch == 'Q') { dio_setup_show_main(); return 1; }
                return 1;
            }

            return 1;
        }

        if (ch == 's' || ch == 'S') { dio_setup_show_main(); return 1; }

        if (ch == 'h' || ch == 'H' || ch == '?') { dio_print_help(); return 1; }

        if (ch == 'w' || ch == 'W') {
            ws_active = 1;
            HEXS_Begin(&ws_hex);
            cli_printf("\r\nwrite: ");
            return 1;
        }

        return 0;
    }

    // write-capture aktiv: x -> abort und global wirken lassen
    if (ch == 'x' || ch == 'X') {
        ws_active = 0;
        HEXS_Reset(&ws_hex);
        cli_printf("\r\n(write aborted)\r\n");
        return 0; // wichtig, damit globales x ins Menü geht
    }

    // hex nibble
    if (HEXS_PushNibbleChar(&ws_hex, ch)) {
        (void)CDC_Transmit_HS((uint8_t*)&ch, 1);
        return 1;
    }

    // finalize
    if (ch == 'p' || ch == 'P') {
        if (HEXS_FinalizeSegment(&ws_hex) < 0) {
            cli_printf("\r\nwrite: FEHLER (hex/len)\r\n");
            ws_active = 0;
            HEXS_Reset(&ws_hex);
            return 1;
        }

        uint16_t blen = HEXS_BytesLen(&ws_hex);
        uint8_t *b    = HEXS_Bytes(&ws_hex);

        cli_printf("\r\n");

        if (blen == 0u) {
            // wp
            dio_print_status();
        } else if (blen == 1u) {
            // w<OO>p
            dio_apply_outputs(b[0]);
            dio_print_status();
        } else {
            cli_printf("write: FEHLER (erwartet 0 oder 1 Byte, got %u)\r\n", (unsigned)blen);
        }

        ws_active = 0;
        HEXS_Reset(&ws_hex);
        return 1;
    }

    return 1;
}
