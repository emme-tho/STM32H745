#ifndef INC_HEXSTREAM_H_
#define INC_HEXSTREAM_H_

#include <stdint.h>

typedef enum {
    HEXS_IDLE = 0,
    HEXS_ACTIVE
} hexs_state_t;

typedef struct {
    hexs_state_t st;

    // Nibbles als ASCII
    char     nib[512];
    uint16_t nib_len;

    // Ergebnisbytes je Segment
    uint8_t  bytes[256];
    uint16_t bytes_len;

} hexstream_t;

void HEXS_Init(hexstream_t *h);
void HEXS_Reset(hexstream_t *h);
void HEXS_Begin(hexstream_t *h);

// returns 1 if consumed (hex nibble), 0 otherwise
uint8_t HEXS_PushNibbleChar(hexstream_t *h, char ch);

// Finalisiert aktuelles Segment: odd nibble -> 0 prefix, konvertiert zu bytes[]
// return 0 ok, <0 error
int HEXS_FinalizeSegment(hexstream_t *h);

uint16_t HEXS_BytesLen(const hexstream_t *h);
uint8_t*  HEXS_Bytes(hexstream_t *h);

#endif /* INC_HEXSTREAM_H_ */
