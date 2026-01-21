/*
 * can_mode.h
 *
 *  Created on: Jan 21, 2026
 *      Author: emmethsg
 */
#ifndef INC_CAN_MODE_H_
#define INC_CAN_MODE_H_

#include <stdint.h>

void CAN_Mode_Enter(void);
uint8_t CAN_Mode_HandleLine(char *line);
uint8_t CAN_Mode_HandleChar(char ch);
void CAN_Mode_Poll(void);

#endif /* INC_CAN_MODE_H_ */
