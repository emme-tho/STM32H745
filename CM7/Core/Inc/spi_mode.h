#ifndef INC_SPI_MODE_H_
#define INC_SPI_MODE_H_

#include <stdint.h>

void SPI_Mode_Enter(void);
uint8_t SPI_Mode_HandleLine(char *line);
uint8_t SPI_Mode_HandleChar(char ch);

#endif /* INC_SPI_MODE_H_ */
