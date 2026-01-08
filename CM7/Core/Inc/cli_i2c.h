/*
 * cli_i2c.h
 *
 *  Created on: Dec 30, 2025
 *      Author: emmethsg
 */

#ifndef INC_CLI_I2C_H_
#define INC_CLI_I2C_H_

#include <stdint.h>
#include <stdbool.h>

void CLI_I2C_Enter(void);
bool CLI_I2C_HandleChar(uint8_t ch);

#endif /* INC_CLI_I2C_H_ */
