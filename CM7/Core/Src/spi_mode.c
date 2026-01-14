#include "spi_mode.h"
#include "cli.h"
#include "pmic.h"
#include "setup_utils.h"
#include "hexstream.h"
#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdlib.h>

// ============================================================
// SPI MODE (Master, SPI2)
//
// Setup (s):
//   - BUCK5 Voltage/Enable
//   - SPI Mode 0-3 (CPOL/CPHA)
//   - Clock (MHz, 1..max)
//   - Frame Format (Motorola/TI)
//   - Datasize (4..32 bit)
//   - First Bit (MSB/LSB)
// ============================================================

static const char *voltage_spi = "buck5";




// ---------------- Setup state ----------------
typedef enum {
    SPI_SETUP_NONE = 0,
    SPI_SETUP_MAIN,
    SPI_SETUP_VOLTAGE,
    SPI_SETUP_MODE,
    SPI_SETUP_CLOCK_INPUT,
    SPI_SETUP_FRAME,
    SPI_SETUP_DATASIZE_INPUT,
    SPI_SETUP_FIRSTBIT,
} spi_setup_state_t;

static spi_setup_state_t g_setup_state = SPI_SETUP_NONE;

// Rail status cache
static uint16_t g_buck5_mv = 0;
static uint8_t  g_buck5_en = 0;

// SPI settings
static uint8_t  g_spi_mode = 0;           // 0..3
static uint8_t  g_spi_frame_motorola = 1; // 1=Motorola, 0=TI
static uint8_t  g_spi_firstbit_msb = 1;   // 1=MSB first
static uint8_t  g_spi_datasize_bits = 8;  // 4..32
static uint32_t g_spi_req_mhz = 1;        // requested MHz
static uint32_t g_spi_clk_hz = 0;         // actual Hz
static uint16_t g_spi_prescaler = 2;      // 2..256

#define SPI_TX_TIMEOUT_MS (100u)

// ---------------- Write stream state ----------------
static uint8_t     ws_active = 0;
static hexstream_t ws_hex;
static uint8_t     ws_rx[256];


// ---------------- Helpers ----------------
static void spi_refresh_rail(void)
{
    uint8_t en = 0, vsel = 0, c1 = 0, c2 = 0;
    uint16_t mv = 0;

    if (PMIC_GetRailStatus(voltage_spi, &en, &vsel, &c1, &c2, &mv) == HAL_OK) {
        g_buck5_en = en;
        g_buck5_mv = mv;
    } else {
        g_buck5_en = 0;
        g_buck5_mv = 0;
    }
}

static uint32_t spi_get_pclk_hz(void)
{
    return HAL_RCC_GetPCLK1Freq();
}

static uint32_t spi_get_max_mhz(void)
{
    uint32_t pclk = spi_get_pclk_hz();
    if (pclk == 0u) return 0u;
    return (pclk / 2u) / 1000000u;
}

#ifdef HAL_SPI_MODULE_ENABLED
__attribute__((weak)) SPI_HandleTypeDef hspi2;

static uint32_t spi_prescaler_to_hal(uint16_t prescaler)
{
    switch (prescaler) {
        case 2:   return SPI_BAUDRATEPRESCALER_2;
        case 4:   return SPI_BAUDRATEPRESCALER_4;
        case 8:   return SPI_BAUDRATEPRESCALER_8;
        case 16:  return SPI_BAUDRATEPRESCALER_16;
        case 32:  return SPI_BAUDRATEPRESCALER_32;
        case 64:  return SPI_BAUDRATEPRESCALER_64;
        case 128: return SPI_BAUDRATEPRESCALER_128;
        case 256: return SPI_BAUDRATEPRESCALER_256;
        default:  return SPI_BAUDRATEPRESCALER_256;
    }
}

static uint32_t spi_datasize_to_hal(uint8_t bits)
{
    switch (bits) {
        case 4:  return SPI_DATASIZE_4BIT;
        case 5:  return SPI_DATASIZE_5BIT;
        case 6:  return SPI_DATASIZE_6BIT;
        case 7:  return SPI_DATASIZE_7BIT;
        case 8:  return SPI_DATASIZE_8BIT;
        case 9:  return SPI_DATASIZE_9BIT;
        case 10: return SPI_DATASIZE_10BIT;
        case 11: return SPI_DATASIZE_11BIT;
        case 12: return SPI_DATASIZE_12BIT;
        case 13: return SPI_DATASIZE_13BIT;
        case 14: return SPI_DATASIZE_14BIT;
        case 15: return SPI_DATASIZE_15BIT;
        case 16: return SPI_DATASIZE_16BIT;
        case 17: return SPI_DATASIZE_17BIT;
        case 18: return SPI_DATASIZE_18BIT;
        case 19: return SPI_DATASIZE_19BIT;
        case 20: return SPI_DATASIZE_20BIT;
        case 21: return SPI_DATASIZE_21BIT;
        case 22: return SPI_DATASIZE_22BIT;
        case 23: return SPI_DATASIZE_23BIT;
        case 24: return SPI_DATASIZE_24BIT;
        case 25: return SPI_DATASIZE_25BIT;
        case 26: return SPI_DATASIZE_26BIT;
        case 27: return SPI_DATASIZE_27BIT;
        case 28: return SPI_DATASIZE_28BIT;
        case 29: return SPI_DATASIZE_29BIT;
        case 30: return SPI_DATASIZE_30BIT;
        case 31: return SPI_DATASIZE_31BIT;
        case 32: return SPI_DATASIZE_32BIT;
        default: return SPI_DATASIZE_4BIT;
    }
}

static void spi_apply_settings(void)
{
    if (&hspi2 == NULL) {
        cli_printf("\r\nSPI2 handle fehlt (hspi2 nicht definiert).\r\n");
        return;
    }

    hspi2.Init.Mode = SPI_MODE_MASTER;
    hspi2.Init.CLKPolarity = (g_spi_mode & 0x2u) ? SPI_POLARITY_HIGH : SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase = (g_spi_mode & 0x1u) ? SPI_PHASE_2EDGE : SPI_PHASE_1EDGE;
    hspi2.Init.TIMode = g_spi_frame_motorola ? SPI_TIMODE_DISABLE : SPI_TIMODE_ENABLE;
    hspi2.Init.FirstBit = g_spi_firstbit_msb ? SPI_FIRSTBIT_MSB : SPI_FIRSTBIT_LSB;
    hspi2.Init.DataSize = spi_datasize_to_hal(g_spi_datasize_bits);
    hspi2.Init.BaudRatePrescaler = spi_prescaler_to_hal(g_spi_prescaler);

    (void)HAL_SPI_DeInit(&hspi2);
    HAL_StatusTypeDef st = HAL_SPI_Init(&hspi2);
    if (st == HAL_OK) {
        cli_printf("\r\nSPI2 re-init OK (mode=%u, %lu Hz, %ubit)\r\n",
                   (unsigned)g_spi_mode, (unsigned long)g_spi_clk_hz, (unsigned)g_spi_datasize_bits);
    } else {
        cli_printf("\r\nSPI2 re-init FEHLER\r\n");
    }
}
#else
static void spi_apply_settings(void)
{
    cli_printf("\r\nSPI HAL nicht aktiviert (HAL_SPI_MODULE_ENABLED).\r\n");
}
#endif

static void spi_set_clock_mhz(uint32_t mhz)
{
    uint32_t max_mhz = spi_get_max_mhz();
    if (mhz < 1u) mhz = 1u;
    if (max_mhz > 0u && mhz > max_mhz) mhz = max_mhz;

    g_spi_req_mhz = mhz;

    uint32_t pclk = spi_get_pclk_hz();
    if (pclk == 0u) {
        g_spi_prescaler = 2;
        g_spi_clk_hz = mhz * 1000000u;
        spi_apply_settings();
        return;
    }

    static const uint16_t prescalers[] = {2, 4, 8, 16, 32, 64, 128, 256};
    uint32_t target_hz = mhz * 1000000u;

    g_spi_prescaler = 256;
    g_spi_clk_hz = pclk / 256u;

    for (size_t i = 0; i < (sizeof(prescalers) / sizeof(prescalers[0])); i++) {
        uint16_t presc = prescalers[i];
        uint32_t freq = pclk / (uint32_t)presc;
        if (freq <= target_hz) {
            g_spi_prescaler = presc;
            g_spi_clk_hz = freq;
            break;
        }
    }

    spi_apply_settings();
}

static void spi_set_mode(uint8_t mode)
{
    if (mode > 3u) return;
    g_spi_mode = mode;
    spi_apply_settings();
}

static void spi_set_frame_motorola(uint8_t motorola)
{
    g_spi_frame_motorola = motorola ? 1u : 0u;
    spi_apply_settings();
}

static void spi_set_firstbit_msb(uint8_t msb)
{
    g_spi_firstbit_msb = msb ? 1u : 0u;
    spi_apply_settings();
}

static void spi_set_datasize(uint8_t bits)
{
    if (bits < 4u || bits > 32u) return;
    g_spi_datasize_bits = bits;
    spi_apply_settings();
}


static void spi_ws_reset(void)
{
    ws_active = 0;
    HEXS_Reset(&ws_hex);
}

static void spi_print_bytes(const uint8_t *b, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        cli_printf("%02X", b[i]);
        if (i + 1u < n) cli_printf(" ");
    }
}
// ---------------- Setup UI ----------------
static void spi_print_setting_summary(void)
{
    spi_refresh_rail();

    cli_printf("  Voltage (BUCK5): ");
    if (g_buck5_mv == 0u) cli_printf("unknown");
    else cli_printf("%umV", g_buck5_mv);
    cli_printf("  EN=%u\r\n", (unsigned)g_buck5_en);

    cli_printf("  SPI Mode: %u (CPOL=%u, CPHA=%u)\r\n",
               (unsigned)g_spi_mode,
               (unsigned)((g_spi_mode >> 1u) & 1u),
               (unsigned)(g_spi_mode & 1u));

    cli_printf("  Clock: req %lu MHz, actual %lu Hz (prescaler %u)\r\n",
               (unsigned long)g_spi_req_mhz,
               (unsigned long)g_spi_clk_hz,
               (unsigned)g_spi_prescaler);

    cli_printf("  Frame: %s\r\n", g_spi_frame_motorola ? "Motorola" : "TI");
    cli_printf("  Datasize: %u bit\r\n", (unsigned)g_spi_datasize_bits);
    cli_printf("  First Bit: %s\r\n", g_spi_firstbit_msb ? "MSB" : "LSB");
}

static void spi_setup_show_main(void)
{
    g_setup_state = SPI_SETUP_MAIN;

    cli_printf("\r\n[SPI Setup]\r\n");
    spi_print_setting_summary();
    cli_printf("\r\n");
    cli_printf("  1 - Voltage\r\n");
    cli_printf("  2 - SPI Mode\r\n");
    cli_printf("  3 - Clock\r\n");
    cli_printf("  4 - Frame Format\r\n");
    cli_printf("  5 - Datasize\r\n");
    cli_printf("  6 - First Bit\r\n");
    cli_printf("  q - back to SPI\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void spi_setup_show_voltage(void)
{
    g_setup_state = SPI_SETUP_VOLTAGE;
    spi_refresh_rail();

    cli_printf("\r\n[SPI Setup] Voltage (BUCK5)\r\n");
    cli_printf("Aktuell: ");
    if (g_buck5_mv == 0u) cli_printf("unknown");
    else cli_printf("%umV", g_buck5_mv);
    cli_printf("  EN=%u\r\n\r\n", (unsigned)g_buck5_en);

    cli_printf("  0 - Disable\r\n");
    cli_printf("  1 - 800 mV\r\n");
    cli_printf("  2 - 1800 mV\r\n");
    cli_printf("  3 - 3300 mV\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void spi_setup_show_mode(void)
{
    g_setup_state = SPI_SETUP_MODE;

    cli_printf("\r\n[SPI Setup] SPI Mode\r\n");
    cli_printf("Aktuell: %u\r\n\r\n", (unsigned)g_spi_mode);

    cli_printf("  0 - Mode 0 (CPOL=0, CPHA=0)\r\n");
    cli_printf("  1 - Mode 1 (CPOL=0, CPHA=1)\r\n");
    cli_printf("  2 - Mode 2 (CPOL=1, CPHA=0)\r\n");
    cli_printf("  3 - Mode 3 (CPOL=1, CPHA=1)\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void spi_setup_show_clock_input(void)
{
    g_setup_state = SPI_SETUP_CLOCK_INPUT;

    uint32_t max_mhz = spi_get_max_mhz();

    cli_printf("\r\n[SPI Setup] Clock\r\n");
    cli_printf("Aktuell: req %lu MHz, actual %lu Hz\r\n", (unsigned long)g_spi_req_mhz, (unsigned long)g_spi_clk_hz);
    if (max_mhz > 0u) {
        cli_printf("Max: %lu MHz\r\n", (unsigned long)max_mhz);
    } else {
        cli_printf("Max: unknown\r\n");
    }
    cli_printf("\r\nGib MHz ein (1..max) oder 'q' fuer Zurueck:\r\n> ");
}

static void spi_setup_show_frame(void)
{
    g_setup_state = SPI_SETUP_FRAME;

    cli_printf("\r\n[SPI Setup] Frame Format\r\n");
    cli_printf("Aktuell: %s\r\n\r\n", g_spi_frame_motorola ? "Motorola" : "TI");

    cli_printf("  1 - Motorola\r\n");
    cli_printf("  2 - TI\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void spi_setup_show_datasize_input(void)
{
    g_setup_state = SPI_SETUP_DATASIZE_INPUT;

    cli_printf("\r\n[SPI Setup] Datasize\r\n");
    cli_printf("Aktuell: %u bit\r\n", (unsigned)g_spi_datasize_bits);
    cli_printf("\r\nGib Bits ein (4..32) oder 'q' fuer Zurueck:\r\n> ");
}

static void spi_setup_show_firstbit(void)
{
    g_setup_state = SPI_SETUP_FIRSTBIT;

    cli_printf("\r\n[SPI Setup] First Bit\r\n");
    cli_printf("Aktuell: %s\r\n\r\n", g_spi_firstbit_msb ? "MSB" : "LSB");

    cli_printf("  1 - MSB first\r\n");
    cli_printf("  2 - LSB first\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

// ---------------- Help ----------------
static void spi_print_help(void)
{
    if (!CLI_IsDebugEnabled()) {
        CLI_PrintDebugRequired();
        return;
    }
    cli_printf("SPI Mode Befehle:\r\n");
    cli_printf("  s           - Setup Menu\r\n");
    cli_printf("  w..p        - Write Stream: w(HEX.. )p (TXRX, RX=TX length)\r\n");
    cli_printf("  ?           - Hilfe\r\n");
}

// ---------------- Public API ----------------
void SPI_Mode_Enter(void)
{
    HEXS_Init(&ws_hex);
    spi_ws_reset();
    spi_set_clock_mhz(g_spi_req_mhz);
    if (CLI_IsDebugEnabled()) {
        spi_print_help();
    }
}

uint8_t SPI_Mode_HandleLine(char *line)
{
    if (!line) return 0;

    // trim leading spaces
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return 1;

    if (g_setup_state == SPI_SETUP_CLOCK_INPUT) {
        if (strcmp(line, "q") == 0 || strcmp(line, "Q") == 0) {
            spi_setup_show_main();
            return 1;
        }

        uint32_t mhz = strtoul(line, NULL, 0);
        if (mhz == 0u) {
            cli_printf("\r\nUngueltige Eingabe.\r\n");
            spi_setup_show_clock_input();
            return 1;
        }

        spi_set_clock_mhz(mhz);
        spi_setup_show_main();
        return 1;
    }

    if (g_setup_state == SPI_SETUP_DATASIZE_INPUT) {
        if (strcmp(line, "q") == 0 || strcmp(line, "Q") == 0) {
            spi_setup_show_main();
            return 1;
        }

        uint32_t bits = strtoul(line, NULL, 0);
        if (bits < 4u || bits > 32u) {
            cli_printf("\r\nUngueltige Bits (4..32).\r\n");
            spi_setup_show_datasize_input();
            return 1;
        }

        spi_set_datasize((uint8_t)bits);
        spi_setup_show_main();
        return 1;
    }

    if (strcmp(line, "s") == 0 || strcmp(line, "S") == 0) {
        spi_setup_show_main();
        return 1;
    }

    if (strcmp(line, "?") == 0 || strcmp(line, "help") == 0) {
        spi_print_help();
        return 1;
    }

    return 0;
}

uint8_t SPI_Mode_HandleChar(char ch)
{
    if (g_setup_state == SPI_SETUP_CLOCK_INPUT || g_setup_state == SPI_SETUP_DATASIZE_INPUT) {
        return 0;
    }

    if (g_setup_state != SPI_SETUP_NONE) {
        if (g_setup_state == SPI_SETUP_MAIN) {
            if (ch == '1') { spi_setup_show_voltage(); return 1; }
            if (ch == '2') { spi_setup_show_mode(); return 1; }
            if (ch == '3') { spi_setup_show_clock_input(); return 1; }
            if (ch == '4') { spi_setup_show_frame(); return 1; }
            if (ch == '5') { spi_setup_show_datasize_input(); return 1; }
            if (ch == '6') { spi_setup_show_firstbit(); return 1; }
            if (ch == 'q' || ch == 'Q') {
                g_setup_state = SPI_SETUP_NONE;
                cli_printf("\r\n(SPI setup closed)\r\n");
                CLI_PrintPrompt();
                return 1;
            }
            return 1;
        }

        if (g_setup_state == SPI_SETUP_VOLTAGE) {
            if (ch == '0') { setup_disable_rail(voltage_spi); spi_setup_show_voltage(); return 1; }
            if (ch == '1') { setup_set_voltage(voltage_spi, 800u);  spi_setup_show_voltage(); return 1; }
            if (ch == '2') { setup_set_voltage(voltage_spi, 1800u); spi_setup_show_voltage(); return 1; }
            if (ch == '3') { setup_set_voltage(voltage_spi, 3300u); spi_setup_show_voltage(); return 1; }
            if (ch == 'q' || ch == 'Q') { spi_setup_show_main(); return 1; }
            return 1;
        }

        if (g_setup_state == SPI_SETUP_MODE) {
            if (ch >= '0' && ch <= '3') { spi_set_mode((uint8_t)(ch - '0')); spi_setup_show_mode(); return 1; }
            if (ch == 'q' || ch == 'Q') { spi_setup_show_main(); return 1; }
            return 1;
        }

        if (g_setup_state == SPI_SETUP_FRAME) {
            if (ch == '1') { spi_set_frame_motorola(1u); spi_setup_show_frame(); return 1; }
            if (ch == '2') { spi_set_frame_motorola(0u); spi_setup_show_frame(); return 1; }
            if (ch == 'q' || ch == 'Q') { spi_setup_show_main(); return 1; }
            return 1;
        }

        if (g_setup_state == SPI_SETUP_FIRSTBIT) {
            if (ch == '1') { spi_set_firstbit_msb(1u); spi_setup_show_firstbit(); return 1; }
            if (ch == '2') { spi_set_firstbit_msb(0u); spi_setup_show_firstbit(); return 1; }
            if (ch == 'q' || ch == 'Q') { spi_setup_show_main(); return 1; }
            return 1;
        }

        return 1;
    }

    if (!ws_active) {
            if (ch == 's' || ch == 'S') { spi_setup_show_main(); return 1; }
            if (ch == '?') { spi_print_help(); return 1; }

            if (ch == 'w' || ch == 'W') {
                ws_active = 1;
                HEXS_Begin(&ws_hex);
                cli_printf("\r\nwrite: ");
                return 1;
            }

            return 0;
        }

        if (ch == 'x' || ch == 'X') {
            spi_ws_reset();
            cli_printf("\r\n(write aborted)\r\n");
            return 0;
        }

        if (ch == 'p' || ch == 'P') {
            int res = HEXS_FinalizeSegment(&ws_hex);
            if (res != 0) {
                cli_printf("\r\nwrite: FEHLER (hex/len)\r\n");
                spi_ws_reset();
                return 1;
            }

            uint16_t len = HEXS_BytesLen(&ws_hex);
            uint8_t *tx = HEXS_Bytes(&ws_hex);

            if (len == 0u) {
                cli_printf("\r\nwrite: FEHLER (no data)\r\n");
                spi_ws_reset();
                return 1;
            }

            cli_printf("\r\nSPI TX: ");
            spi_print_bytes(tx, len);
            cli_printf("\r\n");

    #ifdef HAL_SPI_MODULE_ENABLED
            HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&hspi2, tx, ws_rx, len, SPI_TX_TIMEOUT_MS);
            uint32_t err = HAL_SPI_GetError(&hspi2);

            if (st == HAL_OK) {
                cli_printf("SPI RX: ");
                spi_print_bytes(ws_rx, len);
                cli_printf("\r\n");
            } else {
                cli_printf("SPI TXRX FEHLER: st=%lu, err=0x%08lX\r\n",
                           (unsigned long)st,
                           (unsigned long)err);
            }
    #else
            cli_printf("SPI HAL nicht aktiviert (HAL_SPI_MODULE_ENABLED).\r\n");
    #endif

            spi_ws_reset();
            return 1;
        }

        if (HEXS_PushNibbleChar(&ws_hex, ch)) {
            (void)CDC_Transmit_HS((uint8_t *)&ch, 1);
            return 1;
        }

        return 1;

    return 0;
}
