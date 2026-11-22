#include "rle.h"
#include "memory.h"

/* Fills of at least this many whole bytes go through memset, whose word stores
 * beat a byte loop once its call and alignment overhead is paid for. */
#define CFW_RLE_MEMSET_BYTES 12
/* In cfw_rle_draw's color map: a source color that draws nothing. */
#define CFW_RLE_SKIP 0xffu

/* Read one run: its length (never zero) and color. Returns 0, leaving the
 * cursor where it was, if the run is cut short by `end` or has zero length. */
static inline int cfw_rle_token(const uint8_t **cursor, const uint8_t *end,
                                uint32_t *count, uint32_t *color) {
    const uint8_t *p = *cursor;
    if (p == end) return 0;
    uint32_t op = *p++;
    *color = op & 15u;
    if (op >> 4) {
        *count = op >> 4;
    } else {
        if (p == end) return 0;
        uint32_t n = *p++;
        if (!n) {
            if (end - p < 2) return 0;
            n = (uint32_t)p[0] | ((uint32_t)p[1] << 8);
            if (!n) return 0;
            p += 2;
        }
        *count = n;
    }
    *cursor = p;
    return 1;
}

static const uint8_t *cfw_rle_end(const uint8_t *p, const uint8_t *end, uint32_t pixels) {
    while (pixels) {
        uint32_t count, color;
        if (!cfw_rle_token(&p, end, &count, &color) || count > pixels) return 0;
        pixels -= count;
    }
    return p;
}

/* Set pixels [from, to) of a packed row to `value`; from < to. A span that
 * starts or ends in the middle of a byte keeps that byte's other pixel. */
static inline void cfw_rle_fill(uint8_t *row, uint32_t from, uint32_t to, uint32_t value) {
    uint8_t *p = row + (from >> 1), *last = row + (to >> 1);
    if (from & 1u) {
        *p = (uint8_t)((*p & 0xf0u) | value);
        p++;
    }
    if (p < last) {
        uint32_t pair = value * 0x11u;
        if (last - p >= CFW_RLE_MEMSET_BYTES) {
            memset(p, (int)pair, (uint32_t)(last - p));
        } else {
            #pragma clang loop unroll(disable)
            do *p++ = (uint8_t)pair; while (p != last);
        }
    }
    if (to & 1u) *last = (uint8_t)((*last & 0x0fu) | (value << 4));
}

/* Where cfw_rle_draw is in the destination. Columns are the destination's: a
 * row of the rectangle is [x, row_end), of which [clip_left, clip_right) is
 * visible. */
typedef struct {
    uint8_t *row;         /* the row being drawn */
    uint32_t rows;        /* visible rows not yet completed, this one included */
    int32_t at;           /* the next pixel's column */
    int32_t x, row_end;
    int32_t clip_left, clip_right;
    uint32_t stride;
} cfw_rle_cursor;

/* Move the cursor over `count` pixels without drawing them, onto later rows if
 * need be. Returns 1 once the last visible row is behind it. */
__attribute__((noinline))
static int cfw_rle_pass(cfw_rle_cursor *c, uint32_t count) {
    int32_t at = c->at + (int32_t)count;
    if (at >= c->row_end) {
        uint32_t width = (uint32_t)(c->row_end - c->x);
        uint32_t past = (uint32_t)(at - c->x), down = past / width;
        if (down >= c->rows) return 1;
        at = c->x + (int32_t)(past - down * width);
        c->rows -= down;
        c->row += down * c->stride;
    }
    c->at = at;
    return 0;
}

/* Draw a run of `count` pixels (at least one) from the cursor, onto later rows
 * if need be. Returns 1 once the last visible row is complete. */
__attribute__((noinline))
static int cfw_rle_run(cfw_rle_cursor *c, uint32_t count, uint32_t value) {
    int32_t at = c->at;
    uint8_t *row = c->row;
    do {
        /* The part of the run on this row, then the visible part of that. */
        uint32_t n = (uint32_t)(c->row_end - at);
        if (n > count) n = count;
        count -= n;
        int32_t from = at, to = at + (int32_t)n;
        at = to;
        if (from < c->clip_left) from = c->clip_left;
        if (to > c->clip_right) to = c->clip_right;
        if (from < to) cfw_rle_fill(row, (uint32_t)from, (uint32_t)to, value);
        if (at == c->row_end) {
            if (!--c->rows) return 1;
            at = c->x;
            row += c->stride;
        }
    } while (count);
    c->at = at;
    c->row = row;
    return 0;
}

/* Not inlined: bounding-box calls reach this from the display-list walker,
 * whose stack frame repeats at every level of list nesting.
 *
 * Most runs in antialiased text and dithered images are a single pixel, so
 * those (and transparent runs that stay on their row) are handled in the loop
 * here. Everything else goes through cfw_rle_run and cfw_rle_pass, which work
 * on the cursor in memory: that keeps this loop's own state in registers. */
__attribute__((noinline))
static int cfw_rle_draw(const cfw_rle_dest *dest, int32_t x, int32_t y,
                        uint32_t width, uint32_t height,
                        const uint8_t *p, const uint8_t *end,
                        const uint8_t *lut, int transparent) {
    /* The visible part of the rectangle, in its own coordinates. */
    int32_t left = dest->x0 - x, right = dest->x1 - x;
    int32_t top = dest->y0 - y, bottom = dest->y1 - y;
    if (left < 0) left = 0;
    if (top < 0) top = 0;
    if (right > (int32_t)width) right = (int32_t)width;
    if (bottom > (int32_t)height) bottom = (int32_t)height;
    if (left >= right || top >= bottom) return 0;

    /* What each source color draws. */
    uint8_t map[16];
    for (uint32_t i = 0; i < 16; i++) map[i] = lut[i];
    if (transparent) map[0] = CFW_RLE_SKIP;

    /* Rows above the visible part are read but not drawn. The run that
     * crosses into the first visible row keeps what is left of its length. */
    uint32_t count = 0, color = 0;
    for (uint32_t skip = (uint32_t)top * width; skip;) {
        if (!cfw_rle_token(&p, end, &count, &color)) return -1;
        if (count > skip) {
            count -= skip;
            break;
        }
        skip -= count;
        count = 0;
    }

    cfw_rle_cursor c;
    c.stride = dest->stride;
    c.row = dest->pixels + (uint32_t)(y + top) * c.stride;
    c.rows = (uint32_t)(bottom - top);
    c.at = c.x = x;
    c.row_end = x + (int32_t)width;
    c.clip_left = x + left;
    c.clip_right = x + right;

    const int32_t row_end = c.row_end, clip_left = c.clip_left;
    const uint32_t clip_width = (uint32_t)(right - left);
    int32_t at = c.at;
    uint8_t *row = c.row;
    for (;;) {
        if (!count && !cfw_rle_token(&p, end, &count, &color)) return -1;
        uint32_t value = map[color];
        int done;
        if (value == CFW_RLE_SKIP) {
            at += (int32_t)count;
            count = 0;
            if (at < row_end) continue;
            c.at = at;
            done = cfw_rle_pass(&c, 0);
        } else if (count == 1) {
            if ((uint32_t)(at - clip_left) < clip_width) {
                uint8_t *b = row + ((uint32_t)at >> 1);
                *b = (uint8_t)((at & 1) ? (*b & 0xf0u) | value : (*b & 0x0fu) | (value << 4));
            }
            count = 0;
            if (++at != row_end) continue;
            c.at = at;
            done = cfw_rle_pass(&c, 0);
        } else {
            c.at = at;
            done = cfw_rle_run(&c, count, value);
            count = 0;
        }
        if (done) return 0;
        at = c.at;
        row = c.row;
    }
}
