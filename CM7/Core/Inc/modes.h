#ifndef INC_MODES_H_
#define INC_MODES_H_

#include <stdint.h>
#include "stm32h7xx_hal.h"

typedef enum {
    MODE_NONE = 0,
    MODE_MENU,   // wartet auf I/C/S/U/E/D (sofort)
    MODE_I2C,
    MODE_CAN,
    MODE_SPI,
    MODE_UART,
    MODE_SSI,
    MODE_DIO
} ubt_mode_t;

void MODES_Init(void);

// Menü starten (nach "start")
void MODES_StartMenu(void);

// GoTo Menü
void MODES_GotoMenu(void);   // Ins Interface-Menü springen

// Wird aus CLI_Process pro Zeichen aufgerufen, wenn MODE_MENU aktiv ist
// return 1 wenn Zeichen verarbeitet (z.B. Mode gewechselt), sonst 0
uint8_t MODES_HandleMenuChar(char ch);
uint8_t MODES_IsRawActive(void);


// Line-Handler: wenn ein Mode aktiv ist, wird die Zeile an den Mode weitergereicht
// return 1 wenn verarbeitet, sonst 0
uint8_t MODES_HandleLine(char *line);

// Aktueller Mode
ubt_mode_t MODES_GetMode(void);

// Zurück ins Hauptprompt (z.B. "exit")
void MODES_ExitToRoot(void);

uint8_t MODES_HandleChar(char ch);

#endif /* INC_MODES_H_ */
