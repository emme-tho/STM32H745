/*
 * ringbuf.c
 *
 *  Created on: Dec 30, 2025
 *      Author: emmethsg
 */

#include "ringbuf.h"

void ringbuf_init(ringbuf_t *rb, uint8_t *buffer, uint16_t size)
{
    rb->buf  = buffer;
    rb->size = size;
    rb->head = 0;
    rb->tail = 0;
}

bool ringbuf_put(ringbuf_t *rb, uint8_t data)
{
    uint16_t next = (rb->head + 1) % rb->size;
    if (next == rb->tail) {
        // Buffer voll
        return false;
    }
    rb->buf[rb->head] = data;
    rb->head = next;
    return true;
}

int ringbuf_get(ringbuf_t *rb)
{
    if (rb->head == rb->tail) {
        return -1; // leer
    }
    uint8_t data = rb->buf[rb->tail];
    rb->tail = (rb->tail + 1) % rb->size;
    return data;
}

bool ringbuf_is_empty(ringbuf_t *rb)
{
    return (rb->head == rb->tail);
}



