#include "rle.h"

static void rle_init(rle_state *s, uint8_t *base, uint32_t stride,
                     uint32_t rowbytes, uint32_t rows) {
    s->base = base; s->stride = stride; s->rowbytes = rowbytes; s->rows = rows;
    s->r = 0; s->bpos = 0; s->hi = 1;
    s->left = rowbytes * rows * 2;
    s->st = 0; s->cnt = 0; s->color = 0; s->err = 0;
}

/* Write `n` pixels of color `v`. Aligned whole-byte spans are filled a byte at a time
 * (both nibbles at once, v*0x11); only a leading/trailing odd nibble is read-modify-
 * written. */
static void rle_emit(rle_state *s, uint8_t v, uint32_t n) {
    if (n > s->left) { s->err = 1; return; }        /* overruns the frame -> malformed */
    s->left -= n;
    uint8_t pair = (uint8_t)(v * 0x11u);
    while (n) {
        if (s->r >= s->rows) { s->err = 1; return; }
        uint8_t *row = s->base + s->r * s->stride;
        if (!s->hi) {                                /* finish the byte we're inside */
            row[s->bpos] = (uint8_t)((row[s->bpos] & 0xf0u) | v);
            s->bpos++; s->hi = 1; n--;
        } else {
            uint32_t avail = s->rowbytes - s->bpos;  /* whole bytes left in this row */
            uint32_t bytes = n >> 1;
            if (bytes > avail) bytes = avail;
            for (uint32_t i = 0; i < bytes; i++) row[s->bpos + i] = pair;
            s->bpos += bytes; n -= bytes * 2;
            if (n && s->bpos < s->rowbytes) {        /* odd tail: open the next byte */
                row[s->bpos] = (uint8_t)((row[s->bpos] & 0x0fu) | (uint8_t)(v << 4));
                s->hi = 0; n--;
            }
        }
        if (s->hi && s->bpos >= s->rowbytes) { s->bpos = 0; s->r++; }   /* next row */
    }
}

/* Feed `n` bytes of RLE stream. Stops early (leaving err set) on a malformed stream. */
static void rle_feed(rle_state *s, const uint8_t *p, uint32_t n) {
    for (uint32_t i = 0; i < n && !s->err; i++) {
        uint8_t b = p[i];
        if (s->st == 0) {
            s->color = (uint8_t)(b & 0x0fu);
            uint32_t c = b >> 4;
            if (c) rle_emit(s, s->color, c);         /* short form: count in the high nibble */
            else s->st = 1;                          /* escape: count follows */
        } else if (s->st == 1) {
            if (b) { rle_emit(s, s->color, b); s->st = 0; }
            else s->st = 2;                          /* second escape: 16-bit count follows */
        } else if (s->st == 2) {
            s->cnt = b; s->st = 3;
        } else {
            s->cnt |= (uint32_t)b << 8;
            if (s->cnt == 0) s->err = 1;             /* a 0-length run can't be encoded */
            else rle_emit(s, s->color, s->cnt);
            s->st = 0;
        }
    }
}
