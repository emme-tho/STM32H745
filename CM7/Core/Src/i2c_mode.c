#include "i2c_mode.h"
#include "cli.h"
#include "pmic.h"
#include "usbd_cdc_if.h"   // CDC_Transmit_HS
#include "setup_utils.h"

#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdlib.h>

extern I2C_HandleTypeDef hi2c1;   // I2C1 am Controller

// ============================================================
// I2C MODE (Tool-Port)
//
// Stream-Write (Option A, aktuell):
//   w(ADDR7)(DATA...)(zDATA...)*p
//
// - ADDR7: erstes Byte nach 'w' (2 Hex-Zeichen -> 1 Byte)
//          NUR 7-bit erlaubt: 0x00..0x7F
//          Beispiel: w50AABBp  -> addr=0x50, data=AA BB
//
// - z: Segmentmarker ("no-stop" für später). Aktuell: nur Echo/Trennung.
// - p: finalisieren + alles in EINEM I2C-Transmit senden (Stop am Ende)
//
// - Odd nibble: falls ein Nibble fehlt, wird automatisch '0' vorne eingefuegt
// - 's' (nur im I2C Mode): i2cdetect-style Scan
// - 'x' soll global wirken: Write-Capture wird abgebrochen, 'x' wird NICHT konsumiert
// ============================================================

#define I2C_TX_TIMEOUT_MS     (100u)

/*
 * Runtime-configurable I2C settings for the Tool-Port.
 * NOTE: The I2C speed is set via the TIMING register (hi2c1.Init.Timing).
 * To make 10/100/400kHz work reliably, copy the TIMING values from CubeMX
 * (I2C1 -> Parameter Settings) into these macros (or pass them as compiler defines).
 * If a timing macro is left as 0, the code falls back to the current TIMING value
 * (menu works, but speed does not change).
 */
#ifndef I2C1_TIMING_10KHZ
#define I2C1_TIMING_10KHZ  		0x200009FE
#endif
#ifndef I2C1_TIMING_100KHZ
#define I2C1_TIMING_100KHZ  	0x20000215
#endif
#ifndef I2C1_TIMING_400KHZ
#define I2C1_TIMING_400KHZ  	0x0000020B
#endif


// ---------------- Write stream state ----------------
static uint8_t  ws_active     = 0;
static uint8_t  ws_have_addr  = 0;
static uint8_t  ws_addr7      = 0;

static char     ws_nibbles[2u * 256u + 4u];
static uint16_t ws_nib_len    = 0;

static uint8_t  ws_tx[256];
static uint16_t ws_tx_len     = 0;

static uint32_t ws_seg_idx    = 0;

// ---------------- Read extension ----------------
// After 'r' inside a write-stream, we switch into read-mode.
// Supported forms:
//   w<addr7><reg8>r<LEN_HEX>p   (e.g. w3c57r01p)
//   w<addr7><reg8>rbp          (1 byte)
//   w<addr7><reg8>rwp          (2 bytes)
//   w<addr7><reg8>rhp          (4 bytes)
static uint8_t  rs_active     = 0;
static char     rs_nibbles[4];
static uint8_t  rs_nib_len    = 0;
static uint8_t  rs_len_set    = 0;
static uint16_t rs_len        = 0;
static uint8_t  ws_rx[256];

static const char* voltage_i2c = "ldo1";

// ---------------- Setup menu state ----------------
typedef enum {
    I2C_SETUP_NONE = 0,
    I2C_SETUP_MAIN,
    I2C_SETUP_VOLTAGE,
    I2C_SETUP_CLOCK,
} i2c_setup_state_t;

static i2c_setup_state_t g_setup_state = I2C_SETUP_NONE;

// Current settings (cached for display)
static uint16_t g_ldo1_mv   = 0;   // 0 = unknown
static uint8_t  g_ldo1_en   = 0;
static uint32_t g_i2c_khz   = 0;   // 0 = unknown

// Timing presets (loaded lazily)
static uint8_t  g_timings_inited = 0;
static uint32_t g_timing_10k  = 0;
static uint32_t g_timing_100k = 0;
static uint32_t g_timing_400k = 0;


// ---------------- Helpers ----------------
static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return (c - '0');
    if (c >= 'a' && c <= 'f') return (c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (c - 'A' + 10);
    return -1;
}

static void ws_reset(void)
{
    ws_active = 0;
    ws_have_addr = 0;
    ws_addr7 = 0;
    ws_nib_len = 0;
    ws_tx_len = 0;
    ws_seg_idx = 0;

    memset(ws_nibbles, 0, sizeof(ws_nibbles));
}

static void print_bytes(const uint8_t *b, uint16_t n)
{
    for (uint16_t i = 0; i < n; i++) {
        cli_printf("%02X", b[i]);
        if (i + 1u < n) cli_printf(" ");
    }
}

// Finalisiert aktuelle Segment-Nibbles -> Bytes
// - odd nibble: fuehrende 0
// - erstes Byte (wenn noch keine Adresse): wird Adresse (7-bit enforced)
// - restliche Bytes werden appended in ws_tx[]
static HAL_StatusTypeDef ws_finalize_segment_and_append(uint16_t *out_new_start, uint16_t *out_new_len)
{
    if (out_new_start) *out_new_start = ws_tx_len;
    if (out_new_len)   *out_new_len   = 0;

    if (ws_nib_len == 0) return HAL_OK;

    char tmp[sizeof(ws_nibbles)];
    uint16_t tlen = ws_nib_len;

    if (tlen & 1u) {
        if ((tlen + 1u) > sizeof(tmp)) return HAL_ERROR;
        tmp[0] = '0';
        memcpy(&tmp[1], ws_nibbles, tlen);
        tlen++;
    } else {
        memcpy(tmp, ws_nibbles, tlen);
    }

    uint16_t new_start = ws_tx_len;
    uint16_t new_bytes = 0;

    for (uint16_t i = 0; i < tlen; i += 2u) {
        int a = hex_nibble(tmp[i]);
        int b = hex_nibble(tmp[i + 1u]);
        if (a < 0 || b < 0) return HAL_ERROR;

        uint8_t byte = (uint8_t)((a << 4) | b);

        if (!ws_have_addr) {
            // NUR 7-bit Adresse erlauben
            if (byte > 0x7Fu) {
                return HAL_ERROR;
            }
            ws_addr7 = byte;
            ws_have_addr = 1;
        } else {
            if (ws_tx_len >= sizeof(ws_tx)) return HAL_ERROR;
            ws_tx[ws_tx_len++] = byte;
            new_bytes++;
        }
    }

    ws_nib_len = 0;

    if (out_new_start) *out_new_start = new_start;
    if (out_new_len)   *out_new_len   = new_bytes;
    return HAL_OK;
}

// ---------------- Setup helpers ----------------
static void i2c_init_timings_if_needed(void)
{
    if (g_timings_inited) return;

    // If macros are 0, fall back to current timing (no speed change).
    uint32_t cur = hi2c1.Init.Timing;

    g_timing_10k  = (I2C1_TIMING_10KHZ  != 0u) ? I2C1_TIMING_10KHZ  : cur;
    g_timing_100k = (I2C1_TIMING_100KHZ != 0u) ? I2C1_TIMING_100KHZ : cur;
    g_timing_400k = (I2C1_TIMING_400KHZ != 0u) ? I2C1_TIMING_400KHZ : cur;

    if (g_i2c_khz == 0u) {
        g_i2c_khz = 100u; // best-effort default
    }

    g_timings_inited = 1;
}

// ---------------- Read helpers ----------------
static HAL_StatusTypeDef rs_parse_len(uint16_t *out_len)
{
    if (!out_len) return HAL_ERROR;

    if (rs_len_set) {
        *out_len = rs_len;
        return HAL_OK;
    }

    if (rs_nib_len == 0) {
        return HAL_ERROR;
    }

    // parse up to 4 hex nibbles as integer (e.g. '1' -> 0x1, '01' -> 0x1, '0010' -> 0x10)
    uint16_t val = 0;
    for (uint8_t i = 0; i < rs_nib_len; i++) {
        int n = hex_nibble(rs_nibbles[i]);
        if (n < 0) return HAL_ERROR;
        val = (uint16_t)((val << 4) | (uint16_t)n);
    }

    *out_len = val;
    return HAL_OK;
}


static void i2c_print_setting_summary(void)
{
    cli_printf("  Voltage (LDO1): ");
    if (g_ldo1_mv == 0u) cli_printf("unknown");
    else cli_printf("%umV", g_ldo1_mv);
    cli_printf("  EN=%u\r\n", (unsigned)g_ldo1_en);

    cli_printf("  I2C clk: ");
    if (g_i2c_khz == 0u) cli_printf("unknown");
    else cli_printf("%lu kHz", (unsigned long)g_i2c_khz);
    cli_printf("\r\n");
}

static void i2c_setup_show_main(void)
{
    g_setup_state = I2C_SETUP_MAIN;
    i2c_init_timings_if_needed();

    cli_printf("\r\n[I2C Setup]\r\n");
    i2c_print_setting_summary();
    cli_printf("\r\n");
    cli_printf("  1 - Voltage\r\n");
    cli_printf("  2 - Clock\r\n");
    cli_printf("  q - back to I2C\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void i2c_setup_show_voltage(void)
{
    g_setup_state = I2C_SETUP_VOLTAGE;

    cli_printf("\r\n[I2C Setup] Voltage (LDO1)\r\n");
    cli_printf("Aktuell: ");
    if (g_ldo1_mv == 0u) cli_printf("unknown");
    else cli_printf("%umV", g_ldo1_mv);
    cli_printf("  EN=%u\r\n\r\n", (unsigned)g_ldo1_en);

    cli_printf("  1 - 800 mV\r\n");
    cli_printf("  2 - 1800 mV\r\n");
    cli_printf("  3 - 3300 mV\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void i2c_setup_show_clock(void)
{
    g_setup_state = I2C_SETUP_CLOCK;
    i2c_init_timings_if_needed();

    cli_printf("\r\n[I2C Setup] Clock\r\n");
    cli_printf("Aktuell: ");
    if (g_i2c_khz == 0u) cli_printf("unknown");
    else cli_printf("%lu kHz", (unsigned long)g_i2c_khz);
    cli_printf("\r\n\r\n");

    cli_printf("  1 - 10 kHz\r\n");
    cli_printf("  2 - 100 kHz\r\n");
    cli_printf("  3 - 400 kHz\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}



static void i2c_reinit_with_timing(uint32_t timing, uint32_t khz)
{
    (void)HAL_I2C_DeInit(&hi2c1);
    hi2c1.Init.Timing = timing;
    HAL_StatusTypeDef st = HAL_I2C_Init(&hi2c1);

    if (st == HAL_OK) {
        g_i2c_khz = khz;
        cli_printf("\r\nI2C clk set to %lu kHz\r\n", (unsigned long)khz);

        if ((khz == 10u  && I2C1_TIMING_10KHZ  == 0u) ||
            (khz == 100u && I2C1_TIMING_100KHZ == 0u) ||
            (khz == 400u && I2C1_TIMING_400KHZ == 0u)) {
            cli_printf("WARN: TIMING macro for %lu kHz is 0 -> using current timing (no real change).\r\n",
                       (unsigned long)khz);
        }
    } else {
        cli_printf("\r\nI2C re-init FEHLER (timing=0x%08lX)\r\n", (unsigned long)timing);
    }
}

static void i2c_setup_set_clock_choice(uint8_t choice)
{
    i2c_init_timings_if_needed();
    switch (choice) {
        case 1: i2c_reinit_with_timing(g_timing_10k,  10u);  break;
        case 2: i2c_reinit_with_timing(g_timing_100k, 100u); break;
        case 3: i2c_reinit_with_timing(g_timing_400k, 400u); break;
        default: break;
    }
}

// ---------------- i2cdetect style scan ----------------
static void I2C_PrintDetectTable(void)
{
    // Scan-Parameter (konservativ, analyzer-friendly)
    const uint32_t trials   = 1;
    const uint32_t timeout  = 10;  // ms
    const uint32_t delay_ms = 1;   // pause zwischen probes

    cli_printf("\r\n     ");
    for (uint8_t x = 0; x < 16u; x++) {
        cli_printf("%02x ", x);
    }
    cli_printf("\r\n");

    for (uint8_t row = 0; row < 8u; row++) {
        uint8_t base = (uint8_t)(row << 4);
        cli_printf("%02x: ", base);

        for (uint8_t col = 0; col < 16u; col++) {
            uint8_t addr = (uint8_t)(base + col);

            // wie i2cdetect: nur 0x03..0x77
            if (addr < 0x03u || addr > 0x77u) {
                cli_printf("   ");
                continue;
            }

            // Falls I2C in einem komischen Zustand hängt, einmal recovern
            if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
                (void)HAL_I2C_DeInit(&hi2c1);
                (void)HAL_I2C_Init(&hi2c1);
            }

            HAL_StatusTypeDef st = HAL_I2C_IsDeviceReady(
                    &hi2c1,
                    (uint16_t)(addr << 1),
                    trials,
                    timeout);

            // ErrorCode löschen, damit der nächste Probe sauber läuft
            hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;

            if (st == HAL_OK) cli_printf("%02x ", addr);
            else              cli_printf("-- ");

            if (delay_ms) HAL_Delay(delay_ms);
        }

        cli_printf("\r\n");
    }

    cli_printf("\r\n");
}

// ---------------- Help ----------------
static void i2c_print_help(void)
{
    cli_printf("I2C Mode Befehle:\r\n");
    cli_printf("  v <mv>      - LDO1 Spannung setzen (500..3300mV) und enable\r\n");
    cli_printf("  s           - I2C scan (i2cdetect-style)\r\n");
    cli_printf("  w..z..p     - Write Stream: w(ADDR7)(DATA..)(zDATA..)*p\r\n");
    cli_printf("  w..r..p     - Read Stream : w(ADDR7)(REG..)(rLEN|rb|rw|rh)p\r\n");
    cli_printf("               Beispiel: w3c57r01p (read 1 byte ab reg 0x57)\r\n");
    cli_printf("               ADDR7 muss 0x00..0x7F sein (z.B. w50AABBp)\r\n");
    cli_printf("  ?  	      - diese Hilfe\r\n");
}


// ---------------- Public API ----------------
void I2C_Mode_Enter(void)
{
    ws_reset();
    i2c_print_help();
}

uint8_t I2C_Mode_HandleLine(char *line)
{
    // trim leading spaces
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return 1;

    char *cmd = strtok(line, " \t");
    if (!cmd) return 1;

    // Setup menu auch als Line-Command: 's'
    if (strcmp(cmd, "s") == 0) {
        i2c_setup_show_main();
        return 1;
    }


    // Scan als Line-Command: 'scan'
    if (strcmp(cmd, "scan") == 0) {
        cli_printf("\r\nI2C scan (0x03..0x77):\r\n");
        I2C_PrintDetectTable();
        return 1;
    }

    if (strcmp(cmd, "v") == 0 || strcmp(cmd, "voltage") == 0)
    {
        cli_printf("Hinweis: v <mv> ist deprecated, nutze Setup (s).\r\n");
        char *mv_s = strtok(NULL, " \t");
        if (!mv_s) {
            cli_printf("Usage: v <mv>\r\n");
            return 1;
        }

        uint32_t mv = strtoul(mv_s, NULL, 0);
        setup_set_voltage(voltage_i2c,mv);
    }

    cli_printf("Unbekannt: %s (help)\r\n", cmd);
    return 1;
}

// ============================================================
// CHAR Handler fuer sofortige Eingabe (s, w..z..p)
// Rueckgabe: 1 = konsumiert, 0 = nicht meins
// ============================================================
uint8_t I2C_Mode_HandleChar(char ch)
{
    // Wenn gerade kein write läuft:
    if (!ws_active) {
        // Setup menu per Taste 's' (konsistent für alle Interfaces)
        if (ch == 's' || ch == 'S') {
            i2c_setup_show_main();
            return 1;
        }

        if (ch == '?') { i2c_print_help(); return 1; }

        // Wenn Setup aktiv ist: Auswahl verarbeiten
        if (g_setup_state != I2C_SETUP_NONE) {
            if (g_setup_state == I2C_SETUP_MAIN) {
                if (ch == '1') { i2c_setup_show_voltage(); return 1; }
                if (ch == '2') { i2c_setup_show_clock();   return 1; }
                if (ch == 'q' || ch == 'Q') {
                    g_setup_state = I2C_SETUP_NONE;
                    cli_printf("\r\n(I2C setup closed)\r\n");
                    CLI_PrintPrompt();
                    return 1;
                }
                return 1;
            }

            if (g_setup_state == I2C_SETUP_VOLTAGE) {
                if (ch == '1') { setup_set_voltage(voltage_i2c,800u);  i2c_setup_show_voltage(); return 1; }
                if (ch == '2') { setup_set_voltage(voltage_i2c,1800u); i2c_setup_show_voltage(); return 1; }
                if (ch == '3') { setup_set_voltage(voltage_i2c,3300); i2c_setup_show_voltage(); return 1; }
                if (ch == 'q' || ch == 'Q') { i2c_setup_show_main(); return 1; }
                return 1;
            }

            if (g_setup_state == I2C_SETUP_CLOCK) {
                if (ch == '1') { i2c_setup_set_clock_choice(1); i2c_setup_show_clock(); return 1; }
                if (ch == '2') { i2c_setup_set_clock_choice(2); i2c_setup_show_clock(); return 1; }
                if (ch == '3') { i2c_setup_set_clock_choice(3); i2c_setup_show_clock(); return 1; }
                if (ch == 'q' || ch == 'Q') { i2c_setup_show_main(); return 1; }
                return 1;
            }

            return 1;
        }


        // Scan per Taste 'c'
        if (ch == 'c' || ch == 'C') {
            cli_printf("\r\nI2C scan (0x03..0x77):\r\n");
            I2C_PrintDetectTable();
            return 1;
        }

        // Start write stream
         if (ch == 'w' || ch == 'W') {
        	 ws_active  = 1;
        	 ws_nib_len = 0;
        	 ws_tx_len  = 0;
        	 ws_seg_idx = 0;
        	 ws_have_addr = 0;
        	 ws_addr7 = 0;

        	 rs_active = 0;
             rs_nib_len = 0;
             rs_len_set = 0;
             rs_len = 0;
             memset(rs_nibbles, 0, sizeof(rs_nibbles));

             cli_printf("\r\nwrite: ");
             return 1;
         }
         return 0;
    }

    // write active: x soll global wirken -> abbrechen, NICHT konsumieren
    if (ch == 'x' || ch == 'X') {
        ws_reset();
        cli_printf("\r\n(write aborted)\r\n");
        return 0;
    }


    // ------------------------------------------------------------
    // READ-MODE (nach 'r'): hier sammeln wir nur noch LEN / Format
    // ------------------------------------------------------------
    if (rs_active) {
        // Shortcuts: rb/rw/rh (nur wenn noch keine Hex-Ziffern gesammelt wurden)
        if (!rs_len_set && rs_nib_len == 0) {
            if (ch == 'b' || ch == 'B') {
                rs_len = 1;
                rs_len_set = 1;
                (void)CDC_Transmit_HS((uint8_t*)&ch, 1);
                return 1;
            }
            if (ch == 'w' || ch == 'W') {
                rs_len = 2;
                rs_len_set = 1;
                (void)CDC_Transmit_HS((uint8_t*)&ch, 1);
                return 1;
            }
            if (ch == 'h' || ch == 'H') {
                rs_len = 4;
                rs_len_set = 1;
                (void)CDC_Transmit_HS((uint8_t*)&ch, 1);
                return 1;
            }
        }

        // LEN als Hex (1..4 Nibbles)
        if (hex_nibble(ch) >= 0) {
            if (rs_nib_len < (uint8_t)sizeof(rs_nibbles)) {
                rs_nibbles[rs_nib_len++] = ch;
                (void)CDC_Transmit_HS((uint8_t*)&ch, 1);
            }
            return 1;
        }

        // 'p' wird unten finalisiert
        // alles andere ignorieren
    }

    // ------------------------------------------------------------
    // 'r' startet Read: wir finalisieren bis dahin die aktuellen Nibbles
    // ------------------------------------------------------------
    if (!rs_active && (ch == 'r' || ch == 'R')) {
        uint16_t new_start = 0, new_len = 0;
        HAL_StatusTypeDef fst = ws_finalize_segment_and_append(&new_start, &new_len);
        (void)new_start; (void)new_len;

        if (fst != HAL_OK) {
            cli_printf("\r\nwrite: FEHLER (addr>0x7F oder hex/len)\r\n");
            ws_reset();
            return 1;
        }

        if (!ws_have_addr) {
            cli_printf("\r\nread: FEHLER (addr missing)\r\n");
            ws_reset();
            return 1;
        }

        rs_active = 1;
        rs_nib_len = 0;
        rs_len_set = 0;
        rs_len = 0;
        memset(rs_nibbles, 0, sizeof(rs_nibbles));

        (void)CDC_Transmit_HS((uint8_t*)&ch, 1); // echo 'r'
        return 1;
    }

    // hex nibble
    if (hex_nibble(ch) >= 0) {
        if (ws_nib_len < (uint16_t)(sizeof(ws_nibbles) - 1u)) {
            ws_nibbles[ws_nib_len++] = ch;
            (void)CDC_Transmit_HS((uint8_t*)&ch, 1); // echo raw
        }
        return 1;
    }

    // ------------------------------------------------------------
    // Normaler Hex-Daten-Stream (nur wenn NICHT im read-mode)
    // ------------------------------------------------------------
    if (!rs_active && (hex_nibble(ch) >= 0)) {
        if (ws_nib_len < (uint16_t)(sizeof(ws_nibbles) - 1u)) {
            ws_nibbles[ws_nib_len++] = ch;
            (void)CDC_Transmit_HS((uint8_t*)&ch, 1); // echo raw
        }
        return 1;
    }

    // z = Segment Ende (no-stop marker)  (nur wenn NICHT im read-mode)
    if (!rs_active && (ch == 'z' || ch == 'Z')) {
        uint16_t new_start = 0, new_len = 0;
        HAL_StatusTypeDef fst = ws_finalize_segment_and_append(&new_start, &new_len);
        if (fst != HAL_OK) {
            if (!ws_have_addr) {
                cli_printf("\r\nwrite: FEHLER (addr>0x7F oder hex)\r\n");
            } else {
                cli_printf("\r\nwrite: FEHLER (hex/len)\r\n");
            }
            ws_reset();
            return 1;
        }

        if (!ws_have_addr) {
            cli_printf("\r\nwrite(seg%lu, no-stop): (addr missing)\r\nwrite: ",
                       (unsigned long)ws_seg_idx);
        } else {
            cli_printf("\r\nwrite(seg%lu, no-stop): addr7=0x%02X (bus=0x%02X) data=",
                       (unsigned long)ws_seg_idx, ws_addr7, (uint8_t)(ws_addr7 << 1));
            print_bytes(&ws_tx[new_start], new_len);
            cli_printf("\r\nwrite: ");
        }

        ws_seg_idx++;
        return 1;
    }


    // ------------------------------------------------------------
    // p = final (stop + send)  oder final read
    // ------------------------------------------------------------
    if (ch == 'p' || ch == 'P') {
        if (rs_active) {
            // READ finalisieren
            uint16_t len = 0;
            if (rs_parse_len(&len) != HAL_OK) {
                cli_printf("\r\nread: FEHLER (len missing/hex)\r\n");
                ws_reset();
                return 1;
            }
            if (len == 0 || len > sizeof(ws_rx)) {
                cli_printf("\r\nread: FEHLER (len 1..%u)\r\n", (unsigned)sizeof(ws_rx));
                ws_reset();
                return 1;
            }

            cli_printf("\r\nread(final): addr7=0x%02X (bus=0x%02X) prewrite=",
                       ws_addr7, (uint8_t)(ws_addr7 << 1));
            print_bytes(ws_tx, ws_tx_len);
            cli_printf("  len=%u\r\n", (unsigned)len);

            HAL_StatusTypeDef st = HAL_ERROR;

            if (ws_tx_len == 0) {
                // Direktes Lesen ohne Register-Pointer
                st = HAL_I2C_Master_Receive(
                        &hi2c1,
                        (uint16_t)(ws_addr7 << 1),
                        ws_rx,
                        (uint16_t)len,
                        I2C_TX_TIMEOUT_MS
                );
            } else if (ws_tx_len == 1) {
                uint16_t mem = ws_tx[0];
                st = HAL_I2C_Mem_Read(
                        &hi2c1,
                        (uint16_t)(ws_addr7 << 1),
                        mem,
                        I2C_MEMADD_SIZE_8BIT,
                        ws_rx,
                        (uint16_t)len,
                        I2C_TX_TIMEOUT_MS
                );
            } else if (ws_tx_len == 2) {
                uint16_t mem = (uint16_t)((ws_tx[0] << 8) | ws_tx[1]);
                st = HAL_I2C_Mem_Read(
                        &hi2c1,
                        (uint16_t)(ws_addr7 << 1),
                        mem,
                        I2C_MEMADD_SIZE_16BIT,
                        ws_rx,
                        (uint16_t)len,
                        I2C_TX_TIMEOUT_MS
                );
            } else {
                cli_printf("read: FEHLER (prewrite len=%u nicht unterstuetzt, nur 0/1/2)\r\n",
                           (unsigned)ws_tx_len);
                ws_reset();
                return 1;
            }

            uint32_t err   = HAL_I2C_GetError(&hi2c1);
            uint32_t state = HAL_I2C_GetState(&hi2c1);

            if (st == HAL_OK) {
                cli_printf("I2C RX OK: ");
                print_bytes(ws_rx, (uint16_t)len);
                cli_printf("\r\n");
            } else {
                cli_printf("I2C RX FEHLER: st=%lu, err=0x%08lX, state=%lu\r\n",
                           (unsigned long)st,
                           (unsigned long)err,
                           (unsigned long)state);
            }

            // ErrorCode löschen, damit der nächste Versuch sauber ist
            hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;

            ws_reset();
            return 1;
        }

        // WRITE finalisieren
        uint16_t new_start = 0, new_len = 0;
        (void)new_start;
        (void)new_len;

        HAL_StatusTypeDef fst = ws_finalize_segment_and_append(&new_start, &new_len);
        if (fst != HAL_OK) {
            cli_printf("\r\nwrite: FEHLER (addr>0x7F oder hex/len)\r\n");
            ws_reset();
            return 1;
        }

        if (!ws_have_addr) {
            cli_printf("\r\nwrite: FEHLER (addr missing)\r\n");
            ws_reset();
            return 1;
        }

        cli_printf("\r\nwrite(final, stop): addr7=0x%02X (bus=0x%02X) data=",
                   ws_addr7, (uint8_t)(ws_addr7 << 1));
        print_bytes(ws_tx, ws_tx_len);
        cli_printf("\r\n");

        HAL_StatusTypeDef st = HAL_I2C_Master_Transmit(
                &hi2c1,
                (uint16_t)(ws_addr7 << 1),
                ws_tx,
                ws_tx_len,
                I2C_TX_TIMEOUT_MS
        );

        uint32_t err   = HAL_I2C_GetError(&hi2c1);
        uint32_t state = HAL_I2C_GetState(&hi2c1);

        if (st == HAL_OK) {
            cli_printf("I2C TX OK\r\n");
        } else {
            cli_printf("I2C TX FEHLER: st=%lu, err=0x%08lX, state=%lu\r\n",
                       (unsigned long)st,
                       (unsigned long)err,
                       (unsigned long)state);
        }

        // ErrorCode löschen, damit der nächste Versuch sauber ist
        hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;

        ws_reset();
        return 1;
    }

    // alles andere im write-mode ignorieren
    return 1;
}
