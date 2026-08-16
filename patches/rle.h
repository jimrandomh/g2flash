#pragma once
#include <stdint.h>

/* ---- RLE over 4bpp pixels (the inner layer of modes 3 and 6) ----------------
 *
 * See the RLE paragraph at the top of the file for the token format. The decoder is a
 * byte-at-a-time state machine so it can be driven straight from inflate's output in
 * small chunks (a token may straddle a chunk boundary), writing pixels into a
 * rectangular 4bpp destination: `rows` rows of `rowbytes` bytes, row r at
 * base + r*stride. For mode 6 that's the whole shadow; for mode 3 it's the box within
 * it (rowbytes < stride). Runs cross rows freely — the nibble stream is the wire
 * order, not per-row.
 *
 * `left` counts the nibbles still expected; the decode is complete only when it hits
 * exactly 0 with no token half-parsed. Overrunning it (or running past the last row)
 * sets `err` and the caller drops the frame. */
typedef struct {
    uint8_t *base;
    uint32_t stride;      /* bytes between rows in the destination */
    uint32_t rowbytes;    /* bytes per row actually written */
    uint32_t rows;
    uint32_t r;           /* current row */
    uint32_t bpos;        /* byte offset within the current row */
    uint32_t hi;          /* 1 = next nibble is the high (left) one */
    uint32_t left;        /* nibbles still expected */
    uint32_t st;          /* token parser: 0 = opcode, 1 = cnt8, 2 = cntLo, 3 = cntHi */
    uint32_t cnt;
    uint8_t  color;
    uint8_t  err;
} rle_state;

static void rle_init(rle_state *s, uint8_t *base, uint32_t stride, uint32_t rowbytes, uint32_t rows);
static void rle_emit(rle_state *s, uint8_t v, uint32_t n);
static void rle_feed(rle_state *s, const uint8_t *p, uint32_t n);