#include "hexstream.h"
#include <string.h>

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return (c - '0');
    if (c >= 'a' && c <= 'f') return (c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (c - 'A' + 10);
    return -1;
}

void HEXS_Init(hexstream_t *h)
{
    if (!h) return;
    memset(h, 0, sizeof(*h));
    h->st = HEXS_IDLE;
}

void HEXS_Reset(hexstream_t *h)
{
    if (!h) return;
    h->st = HEXS_IDLE;
    h->nib_len = 0;
    h->bytes_len = 0;
}

void HEXS_Begin(hexstream_t *h)
{
    if (!h) return;
    h->st = HEXS_ACTIVE;
    h->nib_len = 0;
    h->bytes_len = 0;
}

uint8_t HEXS_PushNibbleChar(hexstream_t *h, char ch)
{
    if (!h) return 0;
    if (h->st != HEXS_ACTIVE) return 0;

    if (hex_nibble(ch) < 0) return 0;

    if (h->nib_len < sizeof(h->nib)) {
        h->nib[h->nib_len++] = ch;
    }
    return 1;
}

int HEXS_FinalizeSegment(hexstream_t *h)
{
    if (!h) return -1;

    h->bytes_len = 0;

    if (h->nib_len == 0) {
        // leeres Segment => 0 Bytes
        return 0;
    }

    if (h->nib_len >= sizeof(h->nib)) return -2;

    // tmp: ggf. prefix '0'
    char tmp[512];
    uint16_t tlen = h->nib_len;

    if (tlen & 1u) {
        if (tlen + 1u > sizeof(tmp)) return -3;
        tmp[0] = '0';
        memcpy(&tmp[1], h->nib, tlen);
        tlen++;
    } else {
        memcpy(tmp, h->nib, tlen);
    }

    // convert pairs
    for (uint16_t i = 0; i < tlen; i += 2) {
        int a = hex_nibble(tmp[i]);
        int b = hex_nibble(tmp[i+1]);
        if (a < 0 || b < 0) return -4;
        if (h->bytes_len >= sizeof(h->bytes)) return -5;
        h->bytes[h->bytes_len++] = (uint8_t)((a << 4) | b);
    }

    // reset nib buffer for next segment
    h->nib_len = 0;
    return 0;
}

uint16_t HEXS_BytesLen(const hexstream_t *h)
{
    return h ? h->bytes_len : 0;
}

uint8_t* HEXS_Bytes(hexstream_t *h)
{
    return h ? h->bytes : 0;
}
