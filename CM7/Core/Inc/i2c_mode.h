#ifndef INC_I2C_MODE_H_
#define INC_I2C_MODE_H_

#include <stdint.h>

void I2C_Mode_Enter(void);
uint8_t I2C_Mode_HandleLine(char *line);
uint8_t I2C_Mode_HandleChar(char ch);   // <--- NEU

#endif /* INC_I2C_MODE_H_ */
