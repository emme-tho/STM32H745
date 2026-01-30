/*
 * can_mode.c
 *
 *  Created on: Jan 21, 2026
 *      Author: emmethsg
 */
#include "can_mode.h"
#include "cli.h"
#include "fdcan.h"
#include "main.h"
#include "pmic.h"
#include "setup_utils.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// ============================================================
// CAN MODE (FDCAN1)
//
// Setup (s):
//   - LDO3 Voltage/Enable
//   - Baudrate (125/250/500 kbit, prescaler 8/4/2)
//   - 120R termination (PG6, high = enabled)
//   - Optical interface disable (PG7, high = enabled)
//
// Listen (l):
//   - start/stop listen output in terminal
// ============================================================

typedef enum {
    CAN_SETUP_NONE = 0,
    CAN_SETUP_MAIN,
    CAN_SETUP_VOLTAGE,
    CAN_SETUP_BAUD,
    CAN_SETUP_120R,
    CAN_SETUP_OPT,
} can_setup_state_t;

static can_setup_state_t g_setup_state = CAN_SETUP_NONE;

static uint16_t g_ldo3_mv = 0;
static uint8_t  g_ldo3_en = 0;

static uint16_t g_can_baud_kbps = 500;
static uint16_t g_can_prescaler = 2;
static uint8_t g_can_listen = 0;

static uint8_t g_can_120r_enabled = 0;
static uint8_t g_can_opt_disabled = 0;

static const char *voltage_can = "ldo3";
#define CAN_WS_MAX 96u

static uint8_t g_can_ws_active = 0;
static char g_can_ws_buf[CAN_WS_MAX];
static size_t g_can_ws_len = 0u;

static int can_hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return (c - '0');
    if (c >= 'a' && c <= 'f') return (c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (c - 'A' + 10);
    return -1;
}

static uint8_t can_has_120r(void)
{
#if defined(CAN_120R_DIS_GPIO_Port) && defined(CAN_120R_DIS_Pin)
    return 1u;
#else
    return 0u;
#endif
}

static uint8_t can_has_opt_disable(void)
{
#if defined(CAN_OPT2_DIS_GPIO_Port) && defined(CAN_OPT2_DIS_Pin)
    return 1u;
#else
    return 0u;
#endif
}

static void can_set_120r(uint8_t enable)
{
#if defined(CAN_120R_DIS_GPIO_Port) && defined(CAN_120R_DIS_Pin)
    HAL_GPIO_WritePin(CAN_120R_DIS_GPIO_Port, CAN_120R_DIS_Pin,
                      enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    g_can_120r_enabled = enable ? 1u : 0u;
#else
    (void)enable;
#endif
}

static void can_set_opt_disable(uint8_t disable)
{
#if defined(CAN_OPT2_DIS_GPIO_Port) && defined(CAN_OPT2_DIS_Pin)
    HAL_GPIO_WritePin(CAN_OPT2_DIS_GPIO_Port, CAN_OPT2_DIS_Pin,
                      disable ? GPIO_PIN_RESET : GPIO_PIN_SET);
    g_can_opt_disabled = disable ? 1u : 0u;
#else
    (void)disable;
#endif
}

static uint8_t can_read_120r(void)
{
#if defined(CAN_120R_DIS_GPIO_Port) && defined(CAN_120R_DIS_Pin)
    return (HAL_GPIO_ReadPin(CAN_120R_DIS_GPIO_Port, CAN_120R_DIS_Pin) == GPIO_PIN_SET) ? 1u : 0u;
#else
    return 0u;
#endif
}

static uint8_t can_read_opt_disable(void)
{
#if defined(CAN_OPT2_DIS_GPIO_Port) && defined(CAN_OPT2_DIS_Pin)
	return (HAL_GPIO_ReadPin(CAN_OPT2_DIS_GPIO_Port, CAN_OPT2_DIS_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
#else
    return 0u;
#endif
}

static void can_refresh_gpio_state(void)
{
    g_can_120r_enabled = can_read_120r();
    g_can_opt_disabled = can_read_opt_disable();
}

static void can_refresh_rail(void)
{
    uint8_t en = 0, vsel = 0, c1 = 0, c2 = 0;
    uint16_t mv = 0;

    if (PMIC_GetRailStatus(voltage_can, &en, &vsel, &c1, &c2, &mv) == HAL_OK) {
        g_ldo3_en = en;
        g_ldo3_mv = mv;
    } else {
        g_ldo3_en = 0;
        g_ldo3_mv = 0;
    }
}

static uint16_t can_prescaler_to_baud(uint16_t prescaler)
{
    switch (prescaler) {
        case 2: return 500;
        case 4: return 250;
        case 8: return 125;
        default: return 0;
    }
}

static uint32_t can_len_to_dlc(uint8_t len)
{
    if (len == 0u) return FDCAN_DLC_BYTES_0;
    if (len == 1u) return FDCAN_DLC_BYTES_1;
    if (len == 2u) return FDCAN_DLC_BYTES_2;
    if (len == 3u) return FDCAN_DLC_BYTES_3;
    if (len == 4u) return FDCAN_DLC_BYTES_4;
    if (len == 5u) return FDCAN_DLC_BYTES_5;
    if (len == 6u) return FDCAN_DLC_BYTES_6;
    if (len == 7u) return FDCAN_DLC_BYTES_7;
    if (len <= 8u) return FDCAN_DLC_BYTES_8;
    if (len <= 12u) return FDCAN_DLC_BYTES_12;
    return FDCAN_DLC_BYTES_16;
}

static uint8_t can_dlc_to_len(uint32_t dlc)
{
    switch (dlc) {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        default: return 0;
    }
}

static void can_apply_baud(uint16_t prescaler)
{
    if (prescaler == 0u) return;

    g_can_prescaler = prescaler;
    g_can_baud_kbps = can_prescaler_to_baud(prescaler);

    hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan1.Init.AutoRetransmission = ENABLE;
    hfdcan1.Init.TransmitPause = DISABLE;
    hfdcan1.Init.ProtocolException = DISABLE;
    hfdcan1.Init.NominalPrescaler = prescaler;
    hfdcan1.Init.NominalSyncJumpWidth = 4;
    hfdcan1.Init.NominalTimeSeg1 = 60;
    hfdcan1.Init.NominalTimeSeg2 = 14;
    hfdcan1.Init.DataPrescaler = 1;
    hfdcan1.Init.DataSyncJumpWidth = 1;
    hfdcan1.Init.DataTimeSeg1 = 1;
    hfdcan1.Init.DataTimeSeg2 = 1;
    hfdcan1.Init.MessageRAMOffset = 0;
    hfdcan1.Init.StdFiltersNbr = 1;
    hfdcan1.Init.ExtFiltersNbr = 0;
    hfdcan1.Init.RxFifo0ElmtsNbr = 8;
    hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxFifo1ElmtsNbr = 0;
    hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.RxBuffersNbr = 0;
    hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    hfdcan1.Init.TxEventsNbr = 0;
    hfdcan1.Init.TxBuffersNbr = 0;
    hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
    hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;


    (void)HAL_FDCAN_DeInit(&hfdcan1);
    if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) {
        cli_printf("\r\nFDCAN re-init FEHLER\r\n");
        return;
    }

    FDCAN_FilterTypeDef filter = {0};
    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = 0x000;
    filter.FilterID2 = 0x000;
    (void)HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

    (void)HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                       FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_ACCEPT_IN_RX_FIFO0,
                                       FDCAN_FILTER_REMOTE,
                                       FDCAN_FILTER_REMOTE);

    (void)HAL_FDCAN_ConfigRxFifoOverwrite(&hfdcan1, FDCAN_RX_FIFO0, FDCAN_RX_FIFO_OVERWRITE);

    if (HAL_FDCAN_Start(&hfdcan1) == HAL_OK) {
        cli_printf("\r\nFDCAN1 re-init OK (%u kbit, prescaler %u)\r\n",
                   (unsigned)g_can_baud_kbps, (unsigned)g_can_prescaler);
    } else {
        cli_printf("\r\nFDCAN1 start FEHLER\r\n");
    }
}

static void can_print_setting_summary(void)
{
    can_refresh_rail();
    can_refresh_gpio_state();

    cli_printf("  Voltage (LDO3): ");
    if (g_ldo3_mv == 0u) cli_printf("unknown");
    else cli_printf("%umV", g_ldo3_mv);
    cli_printf("  EN=%u\r\n", (unsigned)g_ldo3_en);

    cli_printf("  Baudrate: %u kbit (prescaler %u)\r\n",
               (unsigned)g_can_baud_kbps,
               (unsigned)g_can_prescaler);

    cli_printf("  120R Termination: %s\r\n",
               can_has_120r() ? (g_can_120r_enabled ? "ON" : "OFF") : "n/a");
    cli_printf("  Optical IF:       %s\r\n",
               can_has_opt_disable() ? (g_can_opt_disabled ? "DISABLED" : "ENABLED") : "n/a");
}

static void can_setup_show_main(void)
{
    g_setup_state = CAN_SETUP_MAIN;

    cli_printf("\r\n[CAN Setup]\r\n");
    can_print_setting_summary();
    cli_printf("\r\n");
    cli_printf("  1 - Voltage\r\n");
    cli_printf("  2 - Baudrate\r\n");
    cli_printf("  3 - 120R Termination\r\n");
    cli_printf("  4 - Optical IF Disable\r\n");
    cli_printf("  q - back to CAN\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void can_setup_show_voltage(void)
{
    g_setup_state = CAN_SETUP_VOLTAGE;
    can_refresh_rail();

    cli_printf("\r\n[CAN Setup] Voltage (LDO3)\r\n");
    cli_printf("Aktuell: ");
    if (g_ldo3_mv == 0u) cli_printf("unknown");
    else cli_printf("%umV", g_ldo3_mv);
    cli_printf("  EN=%u\r\n\r\n", (unsigned)g_ldo3_en);

    cli_printf("  0 - Disable\r\n");
    cli_printf("  1 - 800 mV\r\n");
    cli_printf("  2 - 1800 mV\r\n");
    cli_printf("  3 - 3300 mV\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void can_setup_show_baud(void)
{
    g_setup_state = CAN_SETUP_BAUD;

    cli_printf("\r\n[CAN Setup] Baudrate\r\n");
    cli_printf("Aktuell: %u kbit (prescaler %u)\r\n\r\n",
               (unsigned)g_can_baud_kbps, (unsigned)g_can_prescaler);

    cli_printf("  1 - 125 kbit (prescaler 8)\r\n");
    cli_printf("  2 - 250 kbit (prescaler 4)\r\n");
    cli_printf("  3 - 500 kbit (prescaler 2)\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void can_setup_show_120r(void)
{
    g_setup_state = CAN_SETUP_120R;
    can_refresh_gpio_state();

    cli_printf("\r\n[CAN Setup] 120R Termination\r\n");
    cli_printf("Aktuell: %s\r\n\r\n", can_has_120r() ? (g_can_120r_enabled ? "ON" : "OFF") : "n/a");

    cli_printf("  0 - OFF\r\n");
    cli_printf("  1 - ON\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void can_setup_show_opt(void)
{
    g_setup_state = CAN_SETUP_OPT;
    can_refresh_gpio_state();

    cli_printf("\r\n[CAN Setup] Optical IF Disable\r\n");
    cli_printf("Aktuell: %s\r\n\r\n", can_has_opt_disable() ? (g_can_opt_disabled ? "DISABLED" : "ENABLED") : "n/a");

    cli_printf("  0 - ENABLED\r\n");
    cli_printf("  1 - DISABLED\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void can_print_help(void)
{
    if (!CLI_IsDebugEnabled()) {
        CLI_PrintDebugRequired();
        return;
    }
    cli_printf("CAN Mode Befehle:\r\n");
    cli_printf("  s           - Setup\r\n");
    cli_printf("  l           - Listen start/stop\r\n");
    cli_printf("  w<ID>#DATAp - Send (HEX), z.B. w123#1122p\r\n");
    cli_printf("  ?           - diese Hilfe\r\n");
}

static void can_print_list_header(void)
{
    cli_printf("\r\n________________");
    for (uint8_t i = 1; i <= 16u; i++) {
        cli_printf("%u__", (unsigned)i);
    }
    cli_printf("\r\n---ID---#-DLC-#----------DATA----------\r\n");
}

static void can_list_toggle(void)
{
    g_can_listen = g_can_listen ? 0u : 1u;
    if (g_can_listen) {
        can_print_list_header();
    } else {
        cli_printf("\r\n(CAN listen stopped)\r\n");
    }
}

static uint8_t can_parse_hex_bytes(const char *hex, uint8_t *out, uint8_t max_len, uint8_t *out_len)
{
    uint8_t len = 0;
    uint8_t have_nibble = 0;
    uint8_t nibble = 0;

    while (*hex) {
        int val = can_hex_nibble(*hex);
        if (val >= 0) {
            if (!have_nibble) {
                nibble = (uint8_t)val;
                have_nibble = 1;
            } else {
                if (len >= max_len) return 0;
                out[len++] = (uint8_t)((nibble << 4) | (uint8_t)val);
                have_nibble = 0;
            }
        }
        hex++;
    }

    if (have_nibble) {
        if (len >= max_len) return 0;
        out[len++] = (uint8_t)(nibble << 4);
    }

    if (out_len) *out_len = len;
    return 1;
}

static void can_ws_reset(void)
{
    g_can_ws_active = 0u;
    g_can_ws_len = 0u;
    g_can_ws_buf[0] = '\0';
}

static void can_send_frame(const char *line)
{
    if (!line || line[0] == '\0') return;

    const char *hash = strchr(line, '#');
    if (!hash) {
        cli_printf("\r\nCAN send: Format w<ID>#DATAp\r\n");
        return;
    }

    char id_buf[9];
    size_t id_len = (size_t)(hash - line);
    if (id_len == 0 || id_len >= sizeof(id_buf)) {
        cli_printf("\r\nCAN send: ungueltige ID\r\n");
        return;
    }

    memcpy(id_buf, line, id_len);
    id_buf[id_len] = '\0';

    char *endptr = NULL;
    uint32_t can_id = strtoul(id_buf, &endptr, 16);
    if (endptr == id_buf || *endptr != '\0') {
        cli_printf("\r\nCAN send: ungueltige ID\r\n");
        return;
    }

    uint8_t payload[16];
    uint8_t payload_len = 0;
    if (!can_parse_hex_bytes(hash + 1, payload, sizeof(payload), &payload_len)) {
        cli_printf("\r\nCAN send: DATA zu lang (max 16 Bytes)\r\n");
        return;
    }
    if (payload_len > 8u) {
        cli_printf("\r\nCAN send: DATA zu lang fuer Classic CAN (max 8 Bytes)\r\n");
        return;
    }

    FDCAN_TxHeaderTypeDef tx = {0};
    if (can_id <= 0x7FFu) {
        tx.IdType = FDCAN_STANDARD_ID;
        tx.Identifier = can_id;
    } else {
        tx.IdType = FDCAN_EXTENDED_ID;
        tx.Identifier = can_id & 0x1FFFFFFFu;
    }
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = can_len_to_dlc(payload_len);
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0;

    if (hfdcan1.Init.TxFifoQueueElmtsNbr == 0u) {
           cli_printf("\r\nCAN TX FEHLER (fifo not configured)\r\n");
           return;
       }

       uint32_t free_level = HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1);
       if (free_level < hfdcan1.Init.TxFifoQueueElmtsNbr) {
           cli_printf("\r\nCAN TX busy (fifo=%lu/%lu)\r\n",
                      (unsigned long)free_level,
                      (unsigned long)hfdcan1.Init.TxFifoQueueElmtsNbr);
           return;
       }


    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, payload) == HAL_OK) {
        cli_printf("\r\nCAN TX OK (ID=0x%lX, DLC=%u)\r\n",
                   (unsigned long)tx.Identifier, (unsigned)payload_len);
        uint32_t start = HAL_GetTick();
        while (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) < hfdcan1.Init.TxFifoQueueElmtsNbr) {
            if ((HAL_GetTick() - start) > 10u) {
                cli_printf("CAN TX WARN: fifo not empty after send\r\n");
                break;
            }
        }
    }
    else {
        uint32_t err = HAL_FDCAN_GetError(&hfdcan1);
        cli_printf("\r\nCAN TX FEHLER (err=0x%08lX)\r\n", (unsigned long)err);
    }
}

void CAN_Mode_Enter(void)
{
    g_setup_state = CAN_SETUP_NONE;
    g_can_listen = 0;
    can_ws_reset();

    if (hfdcan1.Init.NominalPrescaler != 0u) {
        g_can_prescaler = (uint16_t)hfdcan1.Init.NominalPrescaler;
        uint16_t baud = can_prescaler_to_baud(g_can_prescaler);
        if (baud != 0u) g_can_baud_kbps = baud;
    }

    can_refresh_rail();
    can_refresh_gpio_state();
    can_apply_baud(g_can_prescaler);

    if (CLI_IsDebugEnabled()) {
        can_print_help();
    }
}

uint8_t CAN_Mode_HandleLine(char *line)
{
    if (!line) return 0;

    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return 1;

    if (strcmp(line, "l") == 0 || strcmp(line, "L") == 0) {
        can_list_toggle();
        return 1;
    }

    if (strcmp(line, "s") == 0 || strcmp(line, "S") == 0) {
        can_setup_show_main();
        return 1;
    }

    if (strcmp(line, "?") == 0 || strcmp(line, "help") == 0) {
        can_print_help();
        return 1;
    }

    if (line[0] == 'w' || line[0] == 'W') {
        cli_printf("\r\nCAN send: nutze w<ID>#DATAp\r\n");
        return 1;
    }

    return 0;
}

uint8_t CAN_Mode_HandleChar(char ch)
{
    if (g_setup_state != CAN_SETUP_NONE) {
        if (g_setup_state == CAN_SETUP_MAIN) {
            if (ch == '1') { can_setup_show_voltage(); return 1; }
            if (ch == '2') { can_setup_show_baud(); return 1; }
            if (ch == '3') { can_setup_show_120r(); return 1; }
            if (ch == '4') { can_setup_show_opt(); return 1; }
            if (ch == 'q' || ch == 'Q') {
                g_setup_state = CAN_SETUP_NONE;
                cli_printf("\r\n(CAN setup closed)\r\n");
                CLI_PrintPrompt();
                return 1;
            }
            return 1;
        }

        if (g_setup_state == CAN_SETUP_VOLTAGE) {
            if (ch == '0') { setup_disable_rail(voltage_can); can_setup_show_voltage(); return 1; }
            if (ch == '1') { setup_set_voltage(voltage_can, 800u); can_setup_show_voltage(); return 1; }
            if (ch == '2') { setup_set_voltage(voltage_can, 1800u); can_setup_show_voltage(); return 1; }
            if (ch == '3') { setup_set_voltage(voltage_can, 3300u); can_setup_show_voltage(); return 1; }
            if (ch == 'q' || ch == 'Q') { can_setup_show_main(); return 1; }
            return 1;
        }

        if (g_setup_state == CAN_SETUP_BAUD) {
            if (ch == '1') { can_apply_baud(8); can_setup_show_baud(); return 1; }
            if (ch == '2') { can_apply_baud(4); can_setup_show_baud(); return 1; }
            if (ch == '3') { can_apply_baud(2); can_setup_show_baud(); return 1; }
            if (ch == 'q' || ch == 'Q') { can_setup_show_main(); return 1; }
            return 1;
        }

        if (g_setup_state == CAN_SETUP_120R) {
            if (ch == '0') { can_set_120r(0u); can_setup_show_120r(); return 1; }
            if (ch == '1') { can_set_120r(1u); can_setup_show_120r(); return 1; }
            if (ch == 'q' || ch == 'Q') { can_setup_show_main(); return 1; }
            return 1;
        }

        if (g_setup_state == CAN_SETUP_OPT) {
            if (ch == '0') { can_set_opt_disable(0u); can_setup_show_opt(); return 1; }
            if (ch == '1') { can_set_opt_disable(1u); can_setup_show_opt(); return 1; }
            if (ch == 'q' || ch == 'Q') { can_setup_show_main(); return 1; }
            return 1;
        }
    }

    if (ch == 's' || ch == 'S') {
        can_setup_show_main();
        return 1;
    }

    if (ch == 'l' || ch == 'L') {
        can_list_toggle();
        return 1;
    }

    if (ch == '?') {
        can_print_help();
        return 1;
    }

    if (!g_can_ws_active) {
        if (ch == 'w' || ch == 'W') {
            g_can_ws_active = 1u;
            g_can_ws_len = 0u;
            g_can_ws_buf[0] = '\0';
            cli_printf("\r\nsend: ");
            return 1;
        }
        return 0;
    }

    if (ch == 'x' || ch == 'X') {
        can_ws_reset();
        cli_printf("\r\n(send aborted)\r\n");
        return 0;
    }

    if (ch == 'p' || ch == 'P') {
        g_can_ws_buf[g_can_ws_len] = '\0';
        if (g_can_ws_len == 0u) {
            cli_printf("\r\nsend: FEHLER (no data)\r\n");
            can_ws_reset();
            return 1;
        }
        can_send_frame(g_can_ws_buf);
        can_ws_reset();
        return 1;
    }

    if (ch == '\r' || ch == '\n') {
        return 1;
    }

    if (g_can_ws_len < (CAN_WS_MAX - 1u)) {
        g_can_ws_buf[g_can_ws_len++] = ch;
        g_can_ws_buf[g_can_ws_len] = '\0';
        cli_printf("%c", ch);
        return 1;
    }

    cli_printf("\r\nsend: FEHLER (zu lang)\r\n");
    can_ws_reset();
    return 1;
}

void CAN_Mode_Poll(void)
{
    if (!g_can_listen) return;

    uint8_t retry_guard = 0u;

    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u) {
        FDCAN_RxHeaderTypeDef rx = {0};
        uint8_t data[64] = {0};
        if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx, data) != HAL_OK) {
            uint32_t err = HAL_FDCAN_GetError(&hfdcan1);
            if ((err & HAL_FDCAN_ERROR_FIFO_EMPTY) != 0u) {
                break;
            }
            retry_guard++;
            if (retry_guard > 2u) {
                break;
            }
            continue;
        }
        retry_guard = 0u;

        uint8_t len = can_dlc_to_len(rx.DataLength);
        uint32_t id = rx.Identifier;
        char line[128];
        size_t used = 0u;
        int wrote = snprintf(line + used, sizeof(line) - used, "%08lX#-%2u-#",
                             (unsigned long)id,
                             (unsigned)len);
        if (wrote > 0) {
            used += (size_t)wrote;
            if (used >= sizeof(line)) {
                used = sizeof(line) - 1u;
            }
        }

        for (uint8_t i = 0; i < 16u; i++) {
            if (i < len) {
                wrote = snprintf(line + used, sizeof(line) - used, "%02X ", data[i]);
            } else {
                wrote = snprintf(line + used, sizeof(line) - used, "   ");
            }
            if (wrote > 0) {
                used += (size_t)wrote;
                if (used >= sizeof(line)) {
                    used = sizeof(line) - 1u;
                    break;
                }
            }
        }

        if (used < sizeof(line) - 2u) {
            line[used++] = '\r';
            line[used++] = '\n';
            line[used] = '\0';
        } else {
            line[sizeof(line) - 1u] = '\0';
        }

        cli_printf("%s", line);
    }
}




