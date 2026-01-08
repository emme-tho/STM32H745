/*
 * ringbuf.h
 *
 *  Created on: Dec 30, 2025
 *      Author: emmethsg
 */

#ifndef INC_RINGBUF_H_
#define INC_RINGBUF_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t *buf;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
} ringbuf_t;

void ringbuf_init(ringbuf_t *rb, uint8_t *buffer, uint16_t size);
bool ringbuf_put(ringbuf_t *rb, uint8_t data);
int  ringbuf_get(ringbuf_t *rb); // -1 wenn leer
bool ringbuf_is_empty(ringbuf_t *rb);



#endif /* INC_RINGBUF_H_ */
