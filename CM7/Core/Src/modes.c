#include "modes.h"
#include "cli.h"
#include <string.h>

#include "i2c_mode.h"   // jetzt nur I2C, Rest später
#include "dio_mode.h"   // DIOs
#include "spi_mode.h"   // SPI Master
#include "uart_mode.h"   // UART
static ubt_mode_t g_mode = MODE_NONE;

ubt_mode_t MODES_GetMode(void) { return g_mode; }

void MODES_Init(void)
{
    g_mode = MODE_NONE;
}

static void print_main_menu(void)
{
    cli_printf("\r\nMain Menu:\r\n");
    cli_printf("  I - I2C Mode\r\n");
    cli_printf("  C - CAN Mode\r\n");
    cli_printf("  S - SPI Mode\r\n");
    cli_printf("  U - UART Mode\r\n");
    cli_printf("  E - SSI Mode\r\n");
    cli_printf("  D - Digital IO Mode\r\n");
    cli_printf("\r\nDruecke Taste (I/C/S/U/E/D) um sofort zu wechseln...\r\n");
    cli_printf("  ESC  - Menu stoppen (zur Root-CLI)\r\n");
    cli_printf("  x    - bleibt im Menu (Hotkey wird in CLI abgefangen)\r\n");
}

void MODES_StartMenu(void)
{
    g_mode = MODE_MENU;
    CLI_SetPrompt("MODE> ");
    print_main_menu();
    CLI_PrintPrompt();
}

void MODES_GotoMenu(void)
{
    // Egal woher: ins Menü
    g_mode = MODE_MENU;
    CLI_SetPrompt("MODE> ");
    print_main_menu();
    CLI_PrintPrompt();
}

// in MENU: ein einzelner Buchstabe wechselt sofort
uint8_t MODES_HandleMenuChar(char ch)
{
    if (g_mode != MODE_MENU) return 0;

    // ignore CR/LF
    if (ch == '\r' || ch == '\n') return 1;

    // ESC -> Menu stoppen (zur Root-CLI)
    if ((uint8_t)ch == 0x1B) {
        MODES_ExitToRoot();
        return 1;
    }

    // 'x' im Menü: nichts wechseln, nur Prompt neu
    if (ch == 'x' || ch == 'X') {
        CLI_PrintPrompt();
        return 1;
    }

    // normalize
    if (ch >= 'a' && ch <= 'z') ch = (char)(ch - 'a' + 'A');

    switch (ch)
    {
        case 'I':
            g_mode = MODE_I2C;
            CLI_SetPrompt("I2C> ");
            I2C_Mode_Enter();
            CLI_PrintPrompt();
            return 1;

        case 'C':
            g_mode = MODE_CAN;
            CLI_SetPrompt("CAN> ");
            cli_printf("\r\nCAN Mode folgt.\r\n");
            CLI_PrintPrompt();
            return 1;

        case 'S':
            g_mode = MODE_SPI;
            CLI_SetPrompt("SPI> ");
            SPI_Mode_Enter();
            CLI_PrintPrompt();
            return 1;

        case 'U':
            g_mode = MODE_UART;
            CLI_SetPrompt("UART> ");
            UART_Mode_Enter();
            CLI_PrintPrompt();
            return 1;

        case 'E':
            g_mode = MODE_SSI;
            CLI_SetPrompt("SSI> ");
            cli_printf("\r\nSSI Mode folgt.\r\n");
            CLI_PrintPrompt();
            return 1;

        case 'D':
            g_mode = MODE_DIO;
            CLI_SetPrompt("DIO> ");
            DIO_Mode_Enter();
            CLI_PrintPrompt();
            return 1;

        default:
            print_main_menu();
            CLI_PrintPrompt();
            return 1;
    }
}

uint8_t MODES_HandleLine(char *line)
{
    if (!line) return 0;

    // x als Textkommando (mit Enter) -> Menü
    if (strcmp(line, "x") == 0 || strcmp(line, "X") == 0) {
        MODES_GotoMenu();
        return 1;
    }


    switch (g_mode)
    {
        case MODE_I2C:   return I2C_Mode_HandleLine(line);
        case MODE_DIO:   return DIO_Mode_HandleLine(line);
        case MODE_SPI:   return SPI_Mode_HandleLine(line);
        case MODE_UART:  return UART_Mode_HandleLine(line);

        default:        return 0;
    }
}

uint8_t MODES_HandleChar(char ch)
{
    switch (g_mode)
    {
        case MODE_I2C:
            return I2C_Mode_HandleChar(ch);
        case MODE_DIO:
        	return DIO_Mode_HandleChar(ch);
        case MODE_SPI:
            return SPI_Mode_HandleChar(ch);
        case MODE_UART:
            return UART_Mode_HandleChar(ch);
        default:
            return 0;
    }
}


uint8_t MODES_IsRawActive(void)
{
    if (g_mode == MODE_UART) {
        return UART_Mode_IsRawActive();
    }
    return 0;
}


void MODES_ExitToRoot(void)
{
    g_mode = MODE_NONE;
    CLI_SetPrompt("> ");
    cli_printf("\r\nZurueck.\r\n");
    CLI_PrintPrompt();
}
