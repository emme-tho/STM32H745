#ifndef INC_UART_MODE_H_
#define INC_UART_MODE_H_

#include <stdint.h>

void UART_Mode_Enter(void);
uint8_t UART_Mode_HandleLine(char *line);
uint8_t UART_Mode_HandleChar(char ch);
uint8_t UART_Mode_IsRawActive(void);
void UART_Mode_Poll(void);

#endif /* INC_UART_MODE_H_ */
