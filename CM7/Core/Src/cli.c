/*
 * cli.c
 *
 *  Created on: Dec 30, 2025
 *      Author: emmethsg
 */

#include "cli.h"
#include "ringbuf.h"
#include "usbd_cdc_if.h"
#include "pmic.h"
#include "modes.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern ringbuf_t g_rx_ringbuf;

// ----------------------------- Config -----------------------------
#define CLI_LINE_MAX        128u
#define CLI_TX_MAX          256u
#define CLI_HISTORY_SIZE    10u   // wie viele Kommandos merken

// ----------------------------- Prompt -----------------------------
static const char *g_prompt = "> ";

// ----------------------------- Buffers/State -----------------------------
static char cli_tx_buf[CLI_TX_MAX];
static char cli_line[CLI_LINE_MAX];
static uint16_t cli_line_pos = 0;

static uint8_t cli_banner_printed = 0;
static volatile uint8_t cli_connect_event = 0;

// ESC sequence parsing (for arrow keys)
static uint8_t esc_state = 0; // 0 none, 1 got ESC, 2 got ESC[

// ----------------------------- History -----------------------------
static char history[CLI_HISTORY_SIZE][CLI_LINE_MAX];
static uint8_t hist_count = 0;        // number of stored entries (<= size)
static uint8_t hist_head = 0;         // next write index
static int16_t hist_view = -1;        // browsing index: -1 = not browsing
static char hist_edit_buf[CLI_LINE_MAX]; // stores current typed line when entering history mode

// -------------------------------------------------------------
// Public Prompt API
// -------------------------------------------------------------
void CLI_SetPrompt(const char *prompt)
{
    g_prompt = (prompt && prompt[0]) ? prompt : "> ";
}

void CLI_PrintPrompt(void)
{
    cli_printf("\r\n%s", g_prompt);
}

// ----------------------------- USB printf -----------------------------
void cli_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(cli_tx_buf, sizeof(cli_tx_buf), fmt, args);
    va_end(args);

    if (len <= 0) return;
    if (len > (int)sizeof(cli_tx_buf)) len = sizeof(cli_tx_buf);

    uint8_t result;
    do {
        result = CDC_Transmit_HS((uint8_t*)cli_tx_buf, (uint16_t)len);
        if (result == USBD_BUSY) HAL_Delay(1);
    } while (result == USBD_BUSY);
}

// ----------------------------- Banner -----------------------------
static void CLI_PrintBanner(void)
{
    cli_printf(
        "\r\n"
        "========================================\r\n"
        "  %s\r\n"
        "  Version : %s\r\n"
        "  Author  : %s\r\n"
        "  Build   : %s %s\r\n"
        "========================================\r\n",
        FW_NAME, FW_VERSION, FW_AUTHOR, __DATE__, __TIME__
    );
}

// ----------------------------- History helpers -----------------------------
static void history_reset_view(void)
{
    hist_view = -1;
}

static void history_push(const char *line)
{
    if (!line || line[0] == '\0') return;

    // prevent duplicates (same as last)
    if (hist_count > 0) {
        uint8_t last_idx = (uint8_t)((hist_head + CLI_HISTORY_SIZE - 1u) % CLI_HISTORY_SIZE);
        if (strncmp(history[last_idx], line, CLI_LINE_MAX) == 0) {
            return;
        }
    }

    strncpy(history[hist_head], line, CLI_LINE_MAX - 1u);
    history[hist_head][CLI_LINE_MAX - 1u] = '\0';

    hist_head = (uint8_t)((hist_head + 1u) % CLI_HISTORY_SIZE);
    if (hist_count < CLI_HISTORY_SIZE) hist_count++;

    history_reset_view();
}

static const char* history_get_by_view(int16_t view)
{
    if (hist_count == 0) return NULL;
    if (view < 0) return NULL;
    if (view >= (int16_t)hist_count) return NULL;

    // newest is at (head-1)
    int16_t newest_idx = (int16_t)hist_head - 1;
    if (newest_idx < 0) newest_idx += CLI_HISTORY_SIZE;

    int16_t idx = newest_idx - view;
    while (idx < 0) idx += CLI_HISTORY_SIZE;
    idx %= CLI_HISTORY_SIZE;

    return history[idx];
}

static const char* history_prev(void)
{
    if (hist_count == 0) return NULL;

    if (hist_view == -1) {
        // entering history mode: save current edit line
        strncpy(hist_edit_buf, cli_line, CLI_LINE_MAX - 1u);
        hist_edit_buf[CLI_LINE_MAX - 1u] = '\0';
        hist_view = 0;
    } else {
        if (hist_view < (int16_t)(hist_count - 1u)) hist_view++;
    }
    return history_get_by_view(hist_view);
}

static const char* history_next(void)
{
    if (hist_count == 0) return NULL;
    if (hist_view == -1) return NULL;

    if (hist_view == 0) {
        hist_view = -1;
        return hist_edit_buf;
    } else {
        hist_view--;
        return history_get_by_view(hist_view);
    }
}

// Redraw current input line with current prompt
static void CLI_RedrawLine(const char *new_line)
{
    if (!new_line) new_line = "";

    // Clear line + redraw prompt + text
    cli_printf("\r\033[2K%s%s", g_prompt, new_line);

    strncpy(cli_line, new_line, CLI_LINE_MAX - 1u);
    cli_line[CLI_LINE_MAX - 1u] = '\0';
    cli_line_pos = (uint16_t)strlen(cli_line);
}

// ----------------------------- Help -----------------------------
static void CLI_PrintHelp(void)
{
    cli_printf("Verfuegbare Befehle:\r\n");
    cli_printf("  help | ?\r\n");
    cli_printf("  info\r\n");
    cli_printf("  clear | cls\r\n");
    cli_printf("  start                - Mode-Menue starten\r\n");
    cli_printf("\r\nPMIC:\r\n");
    cli_printf("  pmic ping\r\n");
    cli_printf("  pmic scan\r\n");
    cli_printf("  pmic read  <reg>\r\n");
    cli_printf("  pmic write <reg> <val>\r\n");
    cli_printf("  pmic dump <from> <to>\r\n");
    cli_printf("  pmic rails\r\n");
    cli_printf("  pmic en  <rail> <0|1>\r\n");
    cli_printf("  pmic set <rail> <mv>\r\n");
    cli_printf("  pmic get <rail>\r\n");
    cli_printf("\r\nCLI:\r\n");
    cli_printf("  History: Pfeil Hoch/Runter (↑/↓)\r\n");
}

// ----------------------------- Top-Level command handler
// return 1 if handled, else 0 (so we can forward to Mode handler)
static uint8_t CLI_HandleLine_TopLevel(char *line)
{
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return 1;

    char *cmd = strtok(line, " \t");
    if (!cmd) return 1;

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        CLI_PrintHelp();
        return 1;
    }
    if (strcmp(cmd, "info") == 0) {
        CLI_PrintBanner();
        return 1;
    }
    if (strcmp(cmd, "clear") == 0 || strcmp(cmd, "cls") == 0) {
        cli_printf("\033[2J\033[H");
        CLI_PrintPrompt();
        return 1;
    }
    if (strcmp(cmd, "start") == 0) {
        MODES_StartMenu();
        return 1;
    }

    // ---- PMIC commands ----
    if (strcmp(cmd, "pmic") == 0) {
        char *sub = strtok(NULL, " \t");
        if (!sub) { CLI_PrintHelp(); return 1; }

        if (strcmp(sub, "ping") == 0) {
            if (PMIC_Ping() == HAL_OK) cli_printf("PMIC: OK (addr 0x%02X)\r\n", PMIC_I2C_ADDR_7BIT);
            else                       cli_printf("PMIC: NICHT erreichbar (addr 0x%02X)\r\n", PMIC_I2C_ADDR_7BIT);
            return 1;
        }

        if (strcmp(sub, "scan") == 0) {
            uint8_t found[32];
            uint32_t cnt = 0;

            if (PMIC_I2C_Scan(found, 32, &cnt) != HAL_OK) {
                cli_printf("I2C Scan Fehler\r\n");
                return 1;
            }

            cli_printf("I2C scan (0x08..0x77):\r\n");
            uint32_t show = (cnt > 32u) ? 32u : cnt;
            for (uint32_t i = 0; i < show; i++) cli_printf(" - found device at 0x%02X\r\n", found[i]);
            if (cnt > 32u) cli_printf(" ... (%lu weitere, Liste gekuerzt)\r\n", (unsigned long)(cnt - 32u));
            cli_printf("Scan done.\r\n");
            return 1;
        }

        if (strcmp(sub, "rails") == 0) {
            cli_printf("Rails:\r\n");
            cli_printf("  buck1 buck3 buck4 buck5 ldo1 ldo2 ldo3 ldo4\r\n");
            cli_printf("BUCK2 ist gesperrt.\r\n");
            return 1;
        }

        if (strcmp(sub, "read") == 0) {
            char *reg_str = strtok(NULL, " \t");
            if (!reg_str) { cli_printf("Usage: pmic read <reg>\r\n"); return 1; }
            uint32_t reg = strtoul(reg_str, NULL, 0);
            uint8_t val = 0;
            if (PMIC_ReadReg((uint8_t)reg, &val) == HAL_OK)
                cli_printf("PMIC[0x%02lX] = 0x%02X\r\n", reg, val);
            else
                cli_printf("Fehler beim Lesen von Reg 0x%02lX\r\n", reg);
            return 1;
        }

        if (strcmp(sub, "write") == 0) {
            char *reg_str = strtok(NULL, " \t");
            char *val_str = strtok(NULL, " \t");
            if (!reg_str || !val_str) { cli_printf("Usage: pmic write <reg> <val>\r\n"); return 1; }
            uint32_t reg = strtoul(reg_str, NULL, 0);
            uint32_t val = strtoul(val_str, NULL, 0);

            HAL_StatusTypeDef st = PMIC_WriteReg((uint8_t)reg, (uint8_t)val);
            if (st == HAL_OK) cli_printf("PMIC[0x%02lX] <- 0x%02lX (OK)\r\n", reg, val);
            else              cli_printf("PMIC write BLOCKED/FAILED at 0x%02lX\r\n", reg);
            return 1;
        }

        if (strcmp(sub, "dump") == 0) {
            char *from_s = strtok(NULL, " \t");
            char *to_s   = strtok(NULL, " \t");
            if (!from_s || !to_s) {
                cli_printf("Usage: pmic dump <from> <to>   (z.B. pmic dump 0x00 0x40)\r\n");
                return 1;
            }

            uint32_t from = strtoul(from_s, NULL, 0);
            uint32_t to   = strtoul(to_s, NULL, 0);

            if (from > 0xFFu || to > 0xFFu || from > to) {
                cli_printf("Ungueltiger Bereich. Erlaubt: 0x00..0xFF und from<=to\r\n");
                return 1;
            }

            cli_printf("PMIC dump 0x%02lX..0x%02lX\r\n", from, to);

            uint8_t val = 0;
            for (uint32_t reg = from; reg <= to; reg++) {
                if ((reg - from) % 16u == 0u) cli_printf("\r\n0x%02lX: ", reg);

                if (PMIC_ReadReg((uint8_t)reg, &val) == HAL_OK) cli_printf("%02X ", val);
                else                                             cli_printf("?? ");
            }
            cli_printf("\r\n");
            return 1;
        }

        if (strcmp(sub, "en") == 0) {
            char *rail = strtok(NULL, " \t");
            char *val  = strtok(NULL, " \t");
            if (!rail || !val) { cli_printf("Usage: pmic en <rail> <0|1>\r\n"); return 1; }
            uint32_t en = strtoul(val, NULL, 0);

            if (PMIC_SetRailEnable(rail, (uint8_t)(en ? 1u : 0u)) == HAL_OK)
                cli_printf("%s enable -> %lu OK\r\n", rail, (unsigned long)en);
            else
                cli_printf("%s enable -> FEHLER/BLOCKED\r\n", rail);
            return 1;
        }

        if (strcmp(sub, "set") == 0) {
            char *rail = strtok(NULL, " \t");
            char *mv_s = strtok(NULL, " \t");
            if (!rail || !mv_s) { cli_printf("Usage: pmic set <rail> <mv>\r\n"); return 1; }

            uint32_t mv = strtoul(mv_s, NULL, 0);
            uint16_t applied = 0;

            HAL_StatusTypeDef st = PMIC_SetRail_mV(rail, (uint16_t)mv, &applied);
            if (st == HAL_OK) cli_printf("%s request %lumV -> applied %umV OK\r\n", rail, (unsigned long)mv, applied);
            else              cli_printf("%s set %lumV FEHLER\r\n", rail, (unsigned long)mv);
            return 1;
        }

        if (strcmp(sub, "get") == 0) {
            char *rail = strtok(NULL, " \t");
            if (!rail) { cli_printf("Usage: pmic get <rail>\r\n"); return 1; }

            uint8_t en=0, vsel=0, c1=0, c2=0;
            uint16_t mv = 0;

            if (PMIC_GetRailStatus(rail, &en, &vsel, &c1, &c2, &mv) == HAL_OK) {
                if (strncmp(rail, "buck", 4) == 0)
                    cli_printf("%s: EN=%u, VSEL=%u, VOUT1=0x%02X, VOUT2=0x%02X, ACTIVE=%umV\r\n",
                               rail, en, vsel, c1, c2, mv);
                else
                    cli_printf("%s: EN=%u, VSET=0x%02X, MV=%umV\r\n", rail, en, c1, mv);
            } else {
                cli_printf("%s: FEHLER\r\n", rail);
            }
            return 1;
        }

        cli_printf("Unbekannter PMIC-Befehl: %s\r\n", sub);
        cli_printf("Tippe 'pmic' oder 'help' fuer Hilfe.\r\n");
        return 1;
    }

    // not handled at top-level
    return 0;
}

// ----------------------------- Public API -----------------------------
void CLI_Init(void)
{
    cli_banner_printed = 0;
    cli_line_pos = 0;
    esc_state = 0;

    memset(cli_line, 0, sizeof(cli_line));

    // history init
    memset(history, 0, sizeof(history));
    hist_count = 0;
    hist_head = 0;
    hist_view = -1;
    memset(hist_edit_buf, 0, sizeof(hist_edit_buf));

    // default prompt
    CLI_SetPrompt("> ");
}

void CLI_OnUsbConnect(uint8_t connected)
{
    if (connected) cli_connect_event = 1;
}

void CLI_Process(void)
{
    if (cli_connect_event && !cli_banner_printed) {
        cli_connect_event = 0;
        CLI_PrintBanner();
        CLI_PrintPrompt();
        cli_banner_printed = 1;
    }

    int c;
    while ((c = ringbuf_get(&g_rx_ringbuf)) != -1) {
        uint8_t ch = (uint8_t)c;

        // ---- MENU Mode: Single-Key sofort verarbeiten ----
        if (MODES_GetMode() == MODE_MENU) {
            (void)MODES_HandleMenuChar((char)ch);
            // Linebuffer sicher leer halten
            cli_line_pos = 0;
            cli_line[0] = '\0';
            history_reset_view();
            esc_state = 0;
            continue;
        }

        // ---- ESC sequence parsing for arrows ----
        if (esc_state == 0) {
            if (ch == 0x1B) { esc_state = 1; continue; }
        } else if (esc_state == 1) {
            if (ch == '[') { esc_state = 2; continue; }
            esc_state = 0;
            continue;
        } else if (esc_state == 2) {
            esc_state = 0;
            if (ch == 'A') { // Up
                const char *h = history_prev();
                if (h) CLI_RedrawLine(h);
                continue;
            } else if (ch == 'B') { // Down
                const char *h = history_next();
                if (h) CLI_RedrawLine(h);
                continue;
            } else {
                continue; // ignore others
            }
        }

        // ---- Backspace / DEL ----
        if (ch == 0x08 || ch == 0x7F) {
            history_reset_view();
            if (cli_line_pos > 0u) {
                cli_line_pos--;
                cli_line[cli_line_pos] = '\0';
                const char bs_seq[] = "\b \b";
                (void)CDC_Transmit_HS((uint8_t*)bs_seq, (uint16_t)strlen(bs_seq));
            }
            continue;
        }

        // ---- Enter ----
        if (ch == '\r' || ch == '\n') {
            (void)CDC_Transmit_HS(&ch, 1);

            cli_line[cli_line_pos] = '\0';

            if (cli_line_pos > 0u) {
                history_push(cli_line);

                char tmp[CLI_LINE_MAX];
                strncpy(tmp, cli_line, CLI_LINE_MAX - 1u);
                tmp[CLI_LINE_MAX - 1u] = '\0';

                // 1) Top-level versuchen
                uint8_t handled = CLI_HandleLine_TopLevel(tmp);

                // 2) Wenn nicht handled -> an Mode weiterreichen
                if (!handled) {
                    // tmp wurde evtl. von strtok verändert -> neu kopieren
                    strncpy(tmp, cli_line, CLI_LINE_MAX - 1u);
                    tmp[CLI_LINE_MAX - 1u] = '\0';

                    if (!MODES_HandleLine(tmp)) {
                        cli_printf("Unbekanntes Kommando: %s\r\n", cli_line);
                        cli_printf("Tippe 'help' fuer Hilfe.\r\n");
                    }
                }
            }

            cli_line_pos = 0;
            cli_line[0] = '\0';
            history_reset_view();

            CLI_PrintPrompt();
            continue;
        }

        // ---- Mode kann Zeichen "schlucken" (z.B. w...z...p) ----
        if (MODES_HandleChar((char)ch)) {
            // nicht in cli_line übernehmen
            continue;
        }


        // ---- Hotkey: 'x' -> immer ins Interface-Menü (nur wenn Zeile leer ist) ----
        if (cli_line_pos == 0u) {
            if (ch == 'x' || ch == 'X') {
                MODES_GotoMenu();      // <--- statt ExitToRoot
                // Input-Zeile sauber lassen
                cli_line_pos = 0;
                cli_line[0] = '\0';
                history_reset_view();
                esc_state = 0;
                continue;              // kein Echo von 'x'
            }
        }
        // ---- Normal character ----
        history_reset_view();

        // Echo
        (void)CDC_Transmit_HS(&ch, 1);

        if (cli_line_pos < (CLI_LINE_MAX - 1u)) {
            cli_line[cli_line_pos++] = (char)ch;
            cli_line[cli_line_pos] = '\0';
        }
    }
}
