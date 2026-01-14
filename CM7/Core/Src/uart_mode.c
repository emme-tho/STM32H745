#include "uart_mode.h"
#include "cli.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include <string.h>
#include "usbd_cdc_if.h"

// ============================================================
// UART MODE (RS485/UART via THVD1424R)
//
// Setup (s):
//   - RS485 120R Termination (GPIO)
//   - RS485 SLR Control (GPIO)
//   - Baudrate (9600 / 115200)
//
// Tunnel (w):
//   - alle Zeichen werden zur UART weitergereicht
//   - beendet mit ESC
// ============================================================

typedef enum {
    UART_SETUP_NONE = 0,
    UART_SETUP_MAIN,
    UART_SETUP_120R,
    UART_SETUP_SLR,
    UART_SETUP_BAUD,
} uart_setup_state_t;

static uart_setup_state_t g_setup_state = UART_SETUP_NONE;
static uint8_t g_uart_tunnel = 0;
static uint8_t g_rs485_120r = 0;
static uint8_t g_rs485_slr = 0;
static uint32_t g_uart_baud = 115200u;

#define UART_TX_TIMEOUT_MS (100u)

#ifdef HAL_UART_MODULE_ENABLED
__attribute__((weak)) UART_HandleTypeDef huart4;
__attribute__((weak)) UART_HandleTypeDef huart8;
#endif

static uint8_t uart_has_120r(void)
{
#if defined(RS485_EN_120R_GPIO_Port) && defined(RS485_EN_120R_Pin)
    return 1u;
#else
    return 0u;
#endif
}

static uint8_t uart_has_slr(void)
{
#if defined(RS485_SLR_Control_GPIO_Port) && defined(RS485_SLR_Control_Pin)
    return 1u;
#else
    return 0u;
#endif
}

static void uart_set_tx_en(uint8_t en)
{
#if defined(UART_TX_EN_GPIO_Port) && defined(UART_TX_EN_Pin)
    HAL_GPIO_WritePin(UART_TX_EN_GPIO_Port, UART_TX_EN_Pin, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)en;
#endif
}

static void uart_set_120r(uint8_t en)
{
#if defined(RS485_EN_120R_GPIO_Port) && defined(RS485_EN_120R_Pin)
    HAL_GPIO_WritePin(RS485_EN_120R_GPIO_Port, RS485_EN_120R_Pin, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)en;
#endif
}

static void uart_set_slr(uint8_t en)
{
#if defined(RS485_SLR_Control_GPIO_Port) && defined(RS485_SLR_Control_Pin)
    HAL_GPIO_WritePin(RS485_SLR_Control_GPIO_Port, RS485_SLR_Control_Pin, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    (void)en;
#endif
}

static uint8_t uart_read_120r(void)
{
#if defined(RS485_EN_120R_GPIO_Port) && defined(RS485_EN_120R_Pin)
    return (HAL_GPIO_ReadPin(RS485_EN_120R_GPIO_Port, RS485_EN_120R_Pin) == GPIO_PIN_SET) ? 1u : 0u;
#else
    return 0u;
#endif
}

static uint8_t uart_read_slr(void)
{
#if defined(RS485_SLR_Control_GPIO_Port) && defined(RS485_SLR_Control_Pin)
    return (HAL_GPIO_ReadPin(RS485_SLR_Control_GPIO_Port, RS485_SLR_Control_Pin) == GPIO_PIN_SET) ? 1u : 0u;
#else
    return 0u;
#endif
}

#ifdef HAL_UART_MODULE_ENABLED
static UART_HandleTypeDef *uart_get_handle(void)
{
    if (huart4.Instance != NULL) return &huart4;
    if (huart8.Instance != NULL) return &huart8;
    return NULL;
}
#endif

static void uart_refresh_gpio_state(void)
{
    g_rs485_120r = uart_read_120r();
    g_rs485_slr = uart_read_slr();
}

static void uart_sync_from_handle(void)
{
#ifdef HAL_UART_MODULE_ENABLED
    UART_HandleTypeDef *huart = uart_get_handle();
    if (huart != NULL) {
        g_uart_baud = huart->Init.BaudRate;
    }
#endif
}

static void uart_apply_baud(void)
{
#ifdef HAL_UART_MODULE_ENABLED
    UART_HandleTypeDef *huart = uart_get_handle();
    if (huart == NULL) {
        cli_printf("\r\nUART handle fehlt (huart4/huart8 nicht definiert).\r\n");
        return;
    }

    huart->Init.BaudRate = g_uart_baud;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    (void)HAL_UART_DeInit(huart);
    HAL_StatusTypeDef st = HAL_UART_Init(huart);
    if (st != HAL_OK) {
        cli_printf("\r\nUART re-init FEHLER\r\n");
        return;
    }

    (void)HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_1_8);
    (void)HAL_UARTEx_DisableFifoMode(huart);

    cli_printf("\r\nUART re-init OK (%lu Baud)\r\n", (unsigned long)g_uart_baud);
#else
    cli_printf("\r\nUART HAL nicht aktiviert (HAL_UART_MODULE_ENABLED).\r\n");
#endif
}

static void uart_print_labels(void)
{
    cli_printf("UART Labels:\r\n");
    cli_printf("  UART_TX (uart4)\r\n");
    cli_printf("  UART_RX (uart4)\r\n");
    cli_printf("  UART_TX_EN (gpio)\r\n");
    cli_printf("  RS485_EN_120R (gpio)\r\n");
    cli_printf("  RS485_SLR_Control (gpio)\r\n");
}

static void uart_setup_show_main(void)
{
    g_setup_state = UART_SETUP_MAIN;
    uart_refresh_gpio_state();

    cli_printf("\r\n[UART Setup]\r\n");
    cli_printf("  120R Termination: %s\r\n", uart_has_120r() ? (g_rs485_120r ? "ON" : "OFF") : "n/a");
    cli_printf("  SLR Control:      %s\r\n", uart_has_slr() ? (g_rs485_slr ? "ON" : "OFF") : "n/a");
    cli_printf("  Baudrate:         %lu\r\n\r\n", (unsigned long)g_uart_baud);

    cli_printf("  1 - RS485 120R\r\n");
    cli_printf("  2 - RS485 SLR\r\n");
    cli_printf("  3 - Baudrate\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void uart_setup_show_120r(void)
{
    g_setup_state = UART_SETUP_120R;
    uart_refresh_gpio_state();

    cli_printf("\r\n[UART Setup] RS485 120R\r\n");
    cli_printf("Aktuell: %s\r\n\r\n", uart_has_120r() ? (g_rs485_120r ? "ON" : "OFF") : "n/a");
    cli_printf("  0 - OFF\r\n");
    cli_printf("  1 - ON\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void uart_setup_show_slr(void)
{
    g_setup_state = UART_SETUP_SLR;
    uart_refresh_gpio_state();

    cli_printf("\r\n[UART Setup] RS485 SLR\r\n");
    cli_printf("Aktuell: %s\r\n\r\n", uart_has_slr() ? (g_rs485_slr ? "ON" : "OFF") : "n/a");
    cli_printf("  0 - OFF\r\n");
    cli_printf("  1 - ON\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void uart_setup_show_baud(void)
{
    g_setup_state = UART_SETUP_BAUD;

    cli_printf("\r\n[UART Setup] Baudrate\r\n");
    cli_printf("Aktuell: %lu\r\n\r\n", (unsigned long)g_uart_baud);
    cli_printf("  1 - 9600\r\n");
    cli_printf("  2 - 115200\r\n");
    cli_printf("  q - back\r\n");
    cli_printf("\r\nAuswahl: ");
}

static void uart_print_help(void)
{
    cli_printf("UART Mode Befehle:\r\n");
    cli_printf("  s        - Setup\r\n");
    cli_printf("  w        - UART Tunnel (ESC beendet)\r\n");
    cli_printf("  ?        - diese Hilfe\r\n");
}

void UART_Mode_Enter(void)
{
    g_uart_tunnel = 0;
    g_setup_state = UART_SETUP_NONE;
    uart_set_tx_en(0u);
    uart_sync_from_handle();
    uart_refresh_gpio_state();
    uart_print_help();
    uart_print_labels();
}

uint8_t UART_Mode_HandleLine(char *line)
{
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return 1;

    if (strcmp(line, "s") == 0 || strcmp(line, "S") == 0) {
        uart_setup_show_main();
        return 1;
    }

    if (strcmp(line, "w") == 0 || strcmp(line, "W") == 0) {
        g_uart_tunnel = 1;
        uart_set_tx_en(0u);
        cli_printf("\r\nUART tunnel aktiv (ESC beendet)\r\n");
        return 1;
    }

    if (strcmp(line, "?") == 0) {
        uart_print_help();
        return 1;
    }

    return 1;
}

uint8_t UART_Mode_HandleChar(char ch)
{
    if (g_uart_tunnel) {
        if ((uint8_t)ch == 0x1B) {
            g_uart_tunnel = 0;
            uart_set_tx_en(0u);
            cli_printf("\r\n(UART tunnel beendet)\r\n");
            CLI_PrintPrompt();
            return 1;
        }

#ifdef HAL_UART_MODULE_ENABLED
        UART_HandleTypeDef *huart = uart_get_handle();
        if (huart != NULL) {
        	uart_set_tx_en(1u);
            (void)HAL_UART_Transmit(huart, (uint8_t *)&ch, 1u, UART_TX_TIMEOUT_MS);
            uart_set_tx_en(0u);
        }
#endif
        return 1;
    }

    if (g_setup_state != UART_SETUP_NONE) {
        if (g_setup_state == UART_SETUP_MAIN) {
            if (ch == '1') { uart_setup_show_120r(); return 1; }
            if (ch == '2') { uart_setup_show_slr(); return 1; }
            if (ch == '3') { uart_setup_show_baud(); return 1; }
            if (ch == 'q' || ch == 'Q') {
                g_setup_state = UART_SETUP_NONE;
                cli_printf("\r\n(UART setup closed)\r\n");
                CLI_PrintPrompt();
                return 1;
            }
            return 1;
        }

        if (g_setup_state == UART_SETUP_120R) {
            if (ch == '0') { uart_set_120r(0u); uart_setup_show_120r(); return 1; }
            if (ch == '1') { uart_set_120r(1u); uart_setup_show_120r(); return 1; }
            if (ch == 'q' || ch == 'Q') { uart_setup_show_main(); return 1; }
            return 1;
        }

        if (g_setup_state == UART_SETUP_SLR) {
            if (ch == '0') { uart_set_slr(0u); uart_setup_show_slr(); return 1; }
            if (ch == '1') { uart_set_slr(1u); uart_setup_show_slr(); return 1; }
            if (ch == 'q' || ch == 'Q') { uart_setup_show_main(); return 1; }
            return 1;
        }

        if (g_setup_state == UART_SETUP_BAUD) {
            if (ch == '1') { g_uart_baud = 9600u; uart_apply_baud(); uart_setup_show_baud(); return 1; }
            if (ch == '2') { g_uart_baud = 115200u; uart_apply_baud(); uart_setup_show_baud(); return 1; }
            if (ch == 'q' || ch == 'Q') { uart_setup_show_main(); return 1; }
            return 1;
        }

        return 1;
    }

    if (ch == 's' || ch == 'S') { uart_setup_show_main(); return 1; }
    if (ch == '?') { uart_print_help(); return 1; }
    if (ch == 'w' || ch == 'W') {
        g_uart_tunnel = 1;
        uart_set_tx_en(0u);
        cli_printf("\r\nUART tunnel aktiv (ESC beendet)\r\n");
        return 1;
    }

    return 0;
}

uint8_t UART_Mode_IsRawActive(void)
{
    return g_uart_tunnel ? 1u : 0u;
}

void UART_Mode_Poll(void)
{
    if (!g_uart_tunnel) {
        return;
    }

#ifdef HAL_UART_MODULE_ENABLED
    UART_HandleTypeDef *huart = uart_get_handle();
    if (huart == NULL) {
        return;
    }

    for (uint32_t i = 0; i < 64u; i++) {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) == RESET) {
            break;
        }

        uint8_t ch = (uint8_t)(huart->Instance->RDR & 0xFFu);
        if (CDC_Transmit_HS(&ch, 1u) == USBD_BUSY) {
            break;
        }
    }
#endif
}

