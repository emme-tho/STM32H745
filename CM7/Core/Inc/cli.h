/*
 * cli.h
 *
 *  Created on: Dec 30, 2025
 *      Author: emmethsg
 */

#ifndef INC_CLI_H_
#define INC_CLI_H_

#include <stdint.h>

// Firmware-Infos
#define FW_NAME    "UBT-TOOL-V2"
#define FW_VERSION "v0.1.0"
#define FW_AUTHOR  "Emmert Thomas"

void CLI_Init(void);
void CLI_SetPrompt(const char *prompt);
void CLI_PrintPrompt(void);

// Wird aus dem USB-CDC Control Event getriggert (Port geöffnet)
void CLI_OnUsbConnect(uint8_t connected);

// Ausgabe über USB CDC
void cli_printf(const char *fmt, ...);




#endif /* INC_CLI_H_ */
