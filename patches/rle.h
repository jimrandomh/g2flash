#pragma once
#include <stdint.h>

/* ---- RLE over 4bpp pixels ----------------------------------------------------
 *
 * A stream is a sequence of runs, each one color (0..15) and a length:
 *
 *   [count4|color4]             1..15 pixels, the count in the high nibble
 *   [0|color4][count8]          1..255 pixels
 *   [0|color4][0][count16 LE]   1..65535 pixels; a zero count16 is malformed
 *
 * A stream covers a width x height rectangle in reading order with no row
 * padding, so a run may continue onto later rows. Bounding-box draw calls,
 * cached images and font glyphs all carry this format.
 *
 * Streams are checked with cfw_rle_end before anything is drawn, so that a bad
 * one cannot leave a half-drawn frame. cfw_rle_draw checks again as it goes
 * (it never reads past `end` or writes outside the destination), which keeps
 * it safe to call on a stream nobody has checked. */

/* A packed 4bpp buffer (the high nibble is the left pixel of a byte) and the
 * part of it a draw may write: [x0,x1) x [y0,y1), which lies inside the buffer. */
typedef struct {
    uint8_t *pixels;
    uint32_t stride;      /* bytes between rows */
    int32_t x0, y0, x1, y1;
} cfw_rle_dest;

/* The end of the stream at `p` that covers exactly `pixels` pixels, or null if
 * it is cut short by `end`, has a zero-length run, or its last run overshoots. */
static const uint8_t *cfw_rle_end(const uint8_t *p, const uint8_t *end, uint32_t pixels);

/* Draw the width x height stream at `p` with its top-left pixel at (x, y),
 * which may be anywhere, writing only inside the destination's region. Each
 * run's color goes through `lut` (16 entries, 0..15); with `transparent`,
 * runs of source color 0 are skipped. Stops after the last visible row.
 * Returns 0, or -1 if the stream is cut short by `end` or has a zero-length
 * run before that row is complete. */
static int cfw_rle_draw(const cfw_rle_dest *dest, int32_t x, int32_t y,
                        uint32_t width, uint32_t height,
                        const uint8_t *p, const uint8_t *end,
                        const uint8_t *lut, int transparent);

/* A lut that draws the stream's colors as they are. */
static const uint8_t cfw_rle_identity[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
