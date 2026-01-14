#ifndef INC_DIO_MODE_H_
#define INC_DIO_MODE_H_

#include <stdint.h>

void    DIO_Mode_Enter(void);
uint8_t DIO_Mode_HandleLine(char *line);
uint8_t DIO_Mode_HandleChar(char ch);

#endif /* INC_DIO_MODE_H_ */
