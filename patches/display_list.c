/* Revision 29. All calls inherit a target; a call-local override is scoped. */
#include "resource_cache.h"

static int cfw_draw_right_lens(void);
static int decode_image_rle(const uint8_t *, uint32_t, uint8_t *, uint32_t, uint32_t, uint32_t);

#define CFW_DRAW_MAX_OPS 4096u
#define CFW_DRAW_MAX_DEPTH 8u
#define CFW_DRAW_SCREEN 65535u
#define CFW_DRAW_CURRENT 65534u

/* Keep names and wire IDs in sync with Faceclaw DrawCallType.kt. */
typedef enum {
    DRAW_OP_BOUNDING_BOX = 1,
    DRAW_OP_RECT_COPY = 2,
    DRAW_OP_STOCK_FONT_STRING = 3,
    DRAW_OP_IMAGE = 4,
    DRAW_OP_TEXT = 5,
    DRAW_OP_REMAP_COLORS = 6,
    DRAW_OP_DISPLAY_LIST = 7,
    DRAW_OP_ROUNDED_RECT = 8,
    DRAW_OP_CLEAR = 9,
} cfw_draw_op;

/* Keep flag names and values in sync with Faceclaw DrawFlags.kt. */
typedef enum {
    DRAW_FLAG_RESOURCE_TARGET = 1,
    DRAW_FLAG_DEPTH = 2,
    DRAW_FLAGS_MASK = DRAW_FLAG_RESOURCE_TARGET | DRAW_FLAG_DEPTH,
} cfw_draw_call_flags;

typedef enum {
    DRAW_BBOX_FLAG_U16 = 1,
} cfw_draw_bbox_flags;

#define DRAW_ROUNDED_RECT_NO_BORDER 16u

typedef struct {
    uint8_t *pixels;
    uint32_t width, height, stride;
    int32_t shift_x;
} cfw_draw_target;

typedef struct {
    uint32_t remaining, depth;
    uint16_t ancestors[CFW_DRAW_MAX_DEPTH];
    uint8_t *references;
    uint32_t elapsed_ms;
    int animation_pending;
} cfw_draw_walk;

/* Everything a draw-call handler needs besides its own bytes and target. */
typedef struct {
    customCfwContext *ctx;
    cfw_draw_walk *walk;
    int apply;
} cfw_draw_env;

/* ---- Message reader ------------------------------------------------------
 *
 * Draw calls are decoded like a byte stream: every READ_* macro returns one
 * field and advances the cursor past it. Handlers read all of their fields
 * first and check bounds once at the end, with READ_DONE when the call must be
 * consumed exactly or READ_IN_BOUNDS when variable-length data follows.
 *
 * Truncated reads latch failure and never advance outside the byte slice. */
typedef struct {
    const uint8_t *p;
    const uint8_t *end;
    int failed;
} cfw_reader;

static cfw_reader cfw_reader_init(const uint8_t *p, uint32_t n) {
    cfw_reader r={p,p+n,0};
    return r;
}

static uint32_t cfw_read_u8(cfw_reader *r) {
    if (r->failed || r->p==r->end) { r->failed=1; return 0; }
    return *r->p++;
}

static uint32_t cfw_read_u16(cfw_reader *r) {
    uint32_t low=cfw_read_u8(r);
    return low | (cfw_read_u8(r)<<8);
}

static const uint8_t *cfw_read_bytes(cfw_reader *r, uint32_t n) {
    if (r->failed || n>(uint32_t)(r->end-r->p)) { r->failed=1; return 0; }
    const uint8_t *start=r->p;
    r->p+=n;
    return start;
}

#define READ_U8(r) cfw_read_u8(&(r))
#define READ_S8(r) ((int32_t)(int8_t)cfw_read_u8(&(r)))
#define READ_U16(r) cfw_read_u16(&(r))
#define READ_S16(r) ((int32_t)(int16_t)cfw_read_u16(&(r)))
#define READ_BYTES(r, n) cfw_read_bytes(&(r), (n))
#define READ_SKIP(r, n) ((void)cfw_read_bytes(&(r), (n)))
#define READ_IN_BOUNDS(r) (!(r).failed)
#define READ_DONE(r) (!(r).failed && (r).p==(r).end)
#define READ_REMAINING(r) ((uint32_t)((r).end-(r).p))

#include "draw_expression.c"

/* ---- Targets and pixels -------------------------------------------------- */

static int cfw_draw_resource_target(customCfwContext *ctx, uint32_t id, cfw_draw_target *out) {
    uint32_t size;
    const uint8_t *data = cfw_resource_get(ctx, id, &size);
    cfw_cached_image image;
    if (!data || (data[0] & (CFW_RESOURCE_TYPE_MASK | CFW_RESOURCE_FLAG_RLE))) {
        return -1;
    }
    if (!cfw_texture_image_at(data, size, 0, &image)) {
        return -1;
    }
    out->pixels = (uint8_t *)image.rle;
    out->width = image.width;
    out->height = image.height;
    out->stride = (image.width + 1u) >> 1;
    out->shift_x = 0;
    return 0;
}

static void cfw_draw_ref(cfw_draw_walk *walk, uint32_t id) {
    if (walk->references && id < CFW_RESOURCE_COUNT) {
        walk->references[id >> 3] |= 1u << (id & 7u);
    }
}

static uint8_t cfw_draw_pixel(cfw_draw_target *t, uint32_t x, uint32_t y) {
    return (t->pixels[y * t->stride + (x >> 1)] >> ((x & 1u) ? 0 : 4)) & 15u;
}

static void cfw_draw_put(cfw_draw_target *t, int32_t x, int32_t y, uint8_t value) {
    x += t->shift_x;
    if (x < 0 || y < 0 || (uint32_t)x >= t->width || (uint32_t)y >= t->height) {
        return;
    }
    uint8_t *p = t->pixels + (uint32_t)y * t->stride + ((uint32_t)x >> 1);
    if (x & 1) {
        *p = (*p & 0xf0u) | value;
    } else {
        *p = (*p & 15u) | (value << 4);
    }
}

static int cfw_draw_rect_in_target(const cfw_draw_target *t, uint32_t x, uint32_t y, uint32_t w, uint32_t h) {
    if (!w || !h) {
        return 0;
    }
    if (x > t->width || w > t->width - x || y > t->height || h > t->height - y) {
        return 0;
    }
    return 1;
}

/* Left inset for a scanline, using the same pixel-center circle as the
 * renderer's original containment test. Reuse the previous row's inset:
 * each corner boundary moves at most radius pixels over the whole corner. */
static int32_t cfw_draw_rounded_inset(int32_t row, int32_t height, int32_t radius, int32_t inset) {
    int32_t edge = row < height - 1 - row ? row : height - 1 - row;
    int32_t dy = 2 * radius - (2 * edge + 1);
    if (dy <= 0) return 0;
    int32_t limit = 4 * radius * radius - dy * dy;
    int32_t dx = 2 * (radius - inset) - 1;
    while (dx > 0 && dx * dx > limit) {
        inset++;
        dx -= 2;
    }
    while (inset > 0 && (dx + 2) * (dx + 2) <= limit) {
        inset--;
        dx += 2;
    }
    return inset;
}

/* Clip once, preserve partial-byte neighbors, then handle two pixels per byte.
 * A zero max-blended fill is a no-op; white fill is an unconditional overwrite. */
static void cfw_draw_rounded_span(uint8_t *row, int32_t width, int32_t left, int32_t right,
                                  uint32_t value, int blend) {
    if (left < 0) left = 0;
    if (right > width) right = width;
    if (left >= right || (blend && value == 0)) return;
    if (value == 15) blend = 0;
    if (left & 1) {
        uint8_t *p = row + (left >> 1);
        uint32_t old = *p & 15u;
        if (!blend || old < value) *p = (*p & 0xf0u) | value;
        left++;
    }
    uint8_t *p = row + (left >> 1);
    uint8_t *end = row + (right >> 1);
    if (blend) {
        while (p < end) {
            uint32_t old = *p;
            uint32_t hi = old >> 4, lo = old & 15u;
            if (hi < value) hi = value;
            if (lo < value) lo = value;
            *p++ = (hi << 4) | lo;
        }
    } else {
        uint8_t packed = (value << 4) | value;
        while (p < end) *p++ = packed;
    }
    if (left < right && (right & 1)) {
        uint32_t old = *p >> 4;
        if (!blend || old < value) *p = (*p & 15u) | (value << 4);
    }
}

/* Control bytes 1..31 pass through; everything else must be well-formed UTF-8. */
static int cfw_draw_validate_utf8(const uint8_t *text, uint32_t length) {
    uint32_t pos = 0;
    while (pos < length) {
        if (text[pos] >= 1 && text[pos] <= 31) {
            pos++;
            continue;
        }
        uint32_t used, codepoint;
        if (!cfw_texture_utf8(text + pos, length - pos, &used, &codepoint)) {
            return -1;
        }
        pos += used;
    }
    return 0;
}

/* Control bytes 1..31 pass through; every other byte must be ASCII 32..127
 * with a complete glyph image in the font. */
static int cfw_draw_validate_font_text(const uint8_t *font, uint32_t font_size, const uint8_t *text, uint32_t length) {
    if (font_size < 193 || font[0] != CFW_RESOURCE_TYPE_FONT) {
        return -1;
    }
    for (uint32_t i = 0; i < length; i++) {
        uint32_t ch = text[i];
        if (ch >= 1 && ch <= 31) {
            continue;
        }
        if (ch < 32 || ch > 127) {
            return -1;
        }
        uint32_t offset = rd16(font + 1 + (ch - 32) * 2);
        cfw_cached_image image;
        if (offset < 193 || !cfw_texture_image_at(font, font_size, offset, &image)) {
            return -1;
        }
    }
    return 0;
}

/* ---- Draw-call handlers --------------------------------------------------
 *
 * Each handler receives its own copy of the reader positioned just after the
 * call header (opcode, flags, and any target/depth overrides). */

static int cfw_draw_sequence(customCfwContext *, const uint8_t *, uint32_t, cfw_draw_target, int, cfw_draw_walk *);

static int cfw_draw_list(customCfwContext *ctx, uint32_t id, cfw_draw_target target, int apply, cfw_draw_walk *walk) {
    if (walk->depth >= CFW_DRAW_MAX_DEPTH) {
        return -1;
    }
    for (uint32_t i = 0; i < walk->depth; i++) {
        if (walk->ancestors[i] == id) {
            return -1;
        }
    }
    uint32_t size;
    const uint8_t *data = cfw_resource_get(ctx, id, &size);
    if (!data || size < 3 || data[0] != CFW_RESOURCE_TYPE_DISPLAY_LIST) {
        return -1;
    }
    cfw_draw_ref(walk, id);
    walk->ancestors[walk->depth++] = id;
    int result = cfw_draw_sequence(ctx, data + 1, size - 1, target, apply, walk);
    walk->depth--;
    return result;
}

/* [bbox-flags8] then x, y, w, h (compact: u8 in units of 4, 2, 4, 2 pixels;
 * DRAW_BBOX_FLAG_U16: raw u16), then a pixel RLE stream with no row padding. */
static int cfw_draw_op_bounding_box(const cfw_draw_env *env, cfw_reader r, cfw_draw_target target) {
    uint32_t bbox_flags = READ_U8(r);
    if (bbox_flags & ~DRAW_BBOX_FLAG_U16) {
        return -1;
    }
    int wide = (bbox_flags & DRAW_BBOX_FLAG_U16) != 0;
    uint32_t x, y, w, h;
    if (wide) {
        x = READ_U16(r);
        y = READ_U16(r);
        w = READ_U16(r);
        h = READ_U16(r);
    } else {
        x = READ_U8(r) * 4u;
        y = READ_U8(r) * 2u;
        w = READ_U8(r) * 4u;
        h = READ_U8(r) * 2u;
    }
    if (!READ_IN_BOUNDS(r)) {
        return -1;
    }
    const uint8_t *rle = r.p;
    uint32_t rle_length = READ_REMAINING(r);
    if (!cfw_draw_rect_in_target(&target, x, y, w, h)) {
        return -1;
    }

    /* Compact boxes that stay on even columns and inside the target after the
     * depth shift can be decoded straight into the buffer. */
    int fast = !wide && !(target.shift_x & 1)
            && (int32_t)x + target.shift_x >= 0
            && (int32_t)(x + w) + target.shift_x <= (int32_t)target.width;

    uint32_t pixel = 0;
    uint32_t total = w * h;
    while (!READ_DONE(r) && pixel < total) {
        uint32_t used, count;
        uint8_t color;
        if (!cfw_texture_rle_token(r.p, READ_REMAINING(r), &used, &count, &color)) {
            return -1;
        }
        if (count > total - pixel) {
            return -1;
        }
        if (env->apply && !fast) {
            for (uint32_t i = 0; i < count; i++) {
                cfw_draw_put(&target, x + (pixel + i) % w, y + (pixel + i) / w, color);
            }
        }
        pixel += count;
        READ_SKIP(r, used);
    }
    if (!READ_DONE(r) || pixel != total) {
        return -1;
    }
    if (env->apply && fast) {
        uint8_t *dest = target.pixels + y * target.stride + ((x + target.shift_x) >> 1);
        if (!decode_image_rle(rle, rle_length, dest, target.stride, w >> 1, h)) {
            return -1;
        }
    }
    return 0;
}

/* Save edge pixels before the byte move: either edge's source byte may be
 * overwritten by an overlapping body. Destination neighbors stay untouched. */
static void cfw_draw_copy_row(uint8_t *dst, const uint8_t *src,
                              uint32_t dx, uint32_t sx, uint32_t width, int reverse) {
    uint32_t first = dx & 1u, last = (dx + width) & 1u;
    uint32_t first_color = 0, last_color = 0;
    if (first) first_color = (src[sx >> 1] >> ((sx & 1u) ? 0 : 4)) & 15u;
    uint32_t end = sx + width - 1;
    if (last) last_color = (src[end >> 1] >> ((end & 1u) ? 0 : 4)) & 15u;
    uint32_t count = (width - first - last) >> 1;
    uint8_t *d = dst + ((dx + first) >> 1);
    const uint8_t *s = src + ((sx + first) >> 1);
    if (!((sx ^ dx) & 1u)) {
        memmove(d, s, count);
    } else if (reverse) {
        for (uint32_t i = count; i > 0; i--) d[i - 1] = (s[i - 1] << 4) | (s[i] >> 4);
    } else {
        for (uint32_t i = 0; i < count; i++) d[i] = (s[i] << 4) | (s[i + 1] >> 4);
    }
    if (first) dst[dx >> 1] = (dst[dx >> 1] & 0xf0u) | first_color;
    if (last) dst[(dx + width) >> 1] = (dst[(dx + width) >> 1] & 15u) | (last_color << 4);
}

/* [source-id16][x extended][y extended][w16][h16][dx extended][dy extended]
 * Source CFW_DRAW_SCREEN is the screen buffer; CFW_DRAW_CURRENT is the target itself.
 * Animated source coordinates must stay inside the source at every time. */
static int cfw_draw_op_rect_copy(const cfw_draw_env *env, cfw_reader r, cfw_draw_target target) {
    cfw_expression_frame frame={env->walk->elapsed_ms,0};
    uint32_t id = READ_U16(r);
    int32_t sx = cfw_read_extended(&r,&frame);
    int32_t sy = cfw_read_extended(&r,&frame);
    uint32_t w = READ_U16(r);
    uint32_t h = READ_U16(r);
    int32_t dx = cfw_read_extended(&r,&frame);
    int32_t dy = cfw_read_extended(&r,&frame);
    if (!READ_DONE(r) || sx < 0 || sy < 0) {
        return -1;
    }
    uint32_t x = (uint32_t)sx, y = (uint32_t)sy;
    env->walk->animation_pending |= frame.animation_pending;

    cfw_draw_target source = target;
    if (id == CFW_DRAW_SCREEN) {
        source.pixels = env->ctx->screen_buffer;
        source.width = 640;
        source.height = 480;
        source.stride = 320;
    } else if (id != CFW_DRAW_CURRENT) {
        if (cfw_draw_resource_target(env->ctx, id, &source)) {
            return -1;
        }
        cfw_draw_ref(env->walk, id);
    }
    if (!source.pixels || !cfw_draw_rect_in_target(&source, x, y, w, h)) {
        return -1;
    }
    if (!env->apply) {
        return 0;
    }

    /* Clip the destination once and advance the source by the same amount.
     * Far-off destinations return before the depth shift can overflow. */
    if (dx < -65536 || dx > 65536) return 0;
    dx += target.shift_x;
    if (dx >= (int32_t)target.width || dy >= (int32_t)target.height ||
        dx <= -(int32_t)w || dy <= -(int32_t)h) return 0;
    if (dx < 0) { x -= dx; w += dx; dx = 0; }
    if (dy < 0) { y -= dy; h += dy; dy = 0; }
    if (w > target.width - (uint32_t)dx) w = target.width - (uint32_t)dx;
    if (h > target.height - (uint32_t)dy) h = target.height - (uint32_t)dy;
    /* Bottom-up rows preserve vertical overlap; the row helper handles
     * horizontal overlap and opposite nibble alignment without a scratch row. */
    int reverse = source.pixels == target.pixels
               && (dy > (int32_t)y || (dy == (int32_t)y && dx > (int32_t)x));
    for (uint32_t i = 0; i < h; i++) {
        uint32_t row = reverse ? h - 1 - i : i;
        cfw_draw_copy_row(target.pixels + ((uint32_t)dy + row) * target.stride,
                          source.pixels + (y + row) * source.stride, dx, x, w, reverse);
    }
    return 0;
}

/* [x16][y16][options8][length8][UTF-8 text] drawn with the stock 20 px font. */
static int cfw_draw_op_stock_font_string(const cfw_draw_env *env, cfw_reader r, cfw_draw_target target) {
    const uint8_t *call = r.p;
    uint32_t call_length = READ_REMAINING(r);
    READ_SKIP(r, 5); /* x, y, options: decoded by the string renderer. */
    uint32_t length = READ_U8(r);
    const uint8_t *text = READ_BYTES(r, length);
    if (!text || !READ_DONE(r)) {
        return -1;
    }
    if (cfw_draw_validate_utf8(text, length)) {
        return -1;
    }
    if (!env->apply) {
        return 0;
    }
    cfw_rectlist rl;
    rl.n = 0;
    return cfw_builtin_draw_string_shifted(target.pixels, target.stride, target.width, target.height,
                                           call, call_length, &rl, target.shift_x);
}

/* [image-id16][x s16][y s16][options8] */
static int cfw_draw_op_image(const cfw_draw_env *env, cfw_reader r, cfw_draw_target target) {
    const uint8_t *call = r.p;
    uint32_t call_length = READ_REMAINING(r);
    uint32_t id = READ_U16(r);
    READ_SKIP(r, 5); /* x, y, options: decoded by the image renderer. */
    if (!READ_DONE(r)) {
        return -1;
    }

    uint32_t size;
    const uint8_t *data = cfw_resource_get(env->ctx, id, &size);
    if (!data) {
        return -1;
    }
    cfw_draw_ref(env->walk, id);
    cfw_cached_image image;
    if (!cfw_texture_image_at(data, size, 0, &image)) {
        return -1;
    }
    if (!env->apply) {
        return 0;
    }
    cfw_rectlist rl;
    rl.n = 0;
    return cfw_texture_draw_image_shifted(target.pixels, target.stride, target.width, target.height,
                                          call, call_length, &rl, target.shift_x);
}

/* [font-id16][x s16][y s16][options8][length8][text] drawn with a font resource. */
static int cfw_draw_op_text(const cfw_draw_env *env, cfw_reader r, cfw_draw_target target) {
    const uint8_t *call = r.p;
    uint32_t call_length = READ_REMAINING(r);
    uint32_t id = READ_U16(r);
    READ_SKIP(r, 5); /* x, y, options: decoded by the string renderer. */
    uint32_t length = READ_U8(r);
    const uint8_t *text = READ_BYTES(r, length);
    if (!text || !READ_DONE(r)) {
        return -1;
    }

    uint32_t size;
    const uint8_t *font = cfw_resource_get(env->ctx, id, &size);
    if (!font) {
        return -1;
    }
    cfw_draw_ref(env->walk, id);
    if (cfw_draw_validate_font_text(font, size, text, length)) {
        return -1;
    }
    if (!env->apply) {
        return 0;
    }
    cfw_rectlist rl;
    rl.n = 0;
    return cfw_texture_draw_string_shifted(target.pixels, target.stride, target.width, target.height,
                                           call, call_length, &rl, target.shift_x);
}

/* Expand the nibble LUT once; whole destination bytes then need one lookup.
 * Keep this leaf out of the recursive walker's stack frame. */
__attribute__((noinline)) static void cfw_draw_remap_rows(cfw_draw_target target,
        uint32_t x, uint32_t y, uint32_t w, uint32_t h, const uint8_t *lut) {
    int32_t left = (int32_t)x + target.shift_x;
    int32_t right = left + (int32_t)w;
    if (left < 0) left = 0;
    if (right > (int32_t)target.width) right = target.width;
    if (left >= right) return;
    uint8_t packed_lut[256];
    for (uint32_t hi = 0; hi < 16; hi++) {
        uint32_t upper = (lut[hi >> 1] >> ((hi & 1) ? 0 : 4)) & 15u;
        for (uint32_t lo = 0; lo < 16; lo++) {
            uint32_t lower = (lut[lo >> 1] >> ((lo & 1) ? 0 : 4)) & 15u;
            packed_lut[(hi << 4) | lo] = (upper << 4) | lower;
        }
    }
    for (uint32_t yy = y; yy < y + h; yy++) {
        uint8_t *row = target.pixels + yy * target.stride;
        int32_t start = left;
        if (start & 1) {
            uint8_t *p = row + (start >> 1);
            *p = (*p & 0xf0u) | (packed_lut[*p] & 15u);
            start++;
        }
        uint8_t *p = row + (start >> 1), *end = row + (right >> 1);
        while (p < end) { *p = packed_lut[*p]; p++; }
        if (start < right && (right & 1)) *p = (*p & 15u) | (packed_lut[*p] & 0xf0u);
    }
}

/* [x16][y16][w16][h16][lut: sixteen packed nibbles] */
static int cfw_draw_op_remap_colors(const cfw_draw_env *env, cfw_reader r, cfw_draw_target target) {
    uint32_t x = READ_U16(r);
    uint32_t y = READ_U16(r);
    uint32_t w = READ_U16(r);
    uint32_t h = READ_U16(r);
    const uint8_t *lut = READ_BYTES(r, 8);
    if (!lut || !READ_DONE(r)) {
        return -1;
    }
    if (!cfw_draw_rect_in_target(&target, x, y, w, h)) {
        return -1;
    }
    if (!env->apply) {
        return 0;
    }
    cfw_draw_remap_rows(target, x, y, w, h, lut);
    return 0;
}

/* [x extended][y extended][w16][h16][radius16][fill8][border8]
 * The fill is max-blended over existing pixels; border DRAW_ROUNDED_RECT_NO_BORDER draws no outline. */
static int cfw_draw_op_rounded_rect(const cfw_draw_env *env, cfw_reader r, cfw_draw_target target) {
    cfw_expression_frame frame={env->walk->elapsed_ms,0};
    int32_t x=cfw_read_extended(&r,&frame);
    int32_t y=cfw_read_extended(&r,&frame);
    uint32_t w = READ_U16(r);
    uint32_t h = READ_U16(r);
    uint32_t radius = READ_U16(r);
    uint32_t fill = READ_U8(r);
    uint32_t border = READ_U8(r);
    if (!READ_DONE(r)) {
        return -1;
    }
    if (!w || !h || w > 640 || h > 480 || fill > 15 || border > DRAW_ROUNDED_RECT_NO_BORDER) {
        return -1;
    }
    env->walk->animation_pending |= frame.animation_pending;
    if (!env->apply) {
        return 0;
    }
    if (x < -(int32_t)w-target.shift_x || x >= (int32_t)target.width-target.shift_x ||
        y < -(int32_t)h || y >= (int32_t)target.height) return 0;
    x += target.shift_x;
    if (radius > w / 2) radius = w / 2;
    if (radius > h / 2) radius = h / 2;
    int32_t inner_radius = radius ? (int32_t)radius - 1 : 0;
    int32_t top = y < 0 ? -y : 0;
    int32_t bottom = (int32_t)target.height - y;
    if (bottom > (int32_t)h) bottom = h;
    int32_t outer_inset = 0, inner_inset = 0;
    for (int32_t yy = top; yy < bottom; yy++) {
        outer_inset = cfw_draw_rounded_inset(yy, h, radius, outer_inset);
        int32_t left = x + outer_inset, right = x + (int32_t)w - outer_inset;
        uint8_t *row = target.pixels + (y + yy) * target.stride;
        if (border == DRAW_ROUNDED_RECT_NO_BORDER) {
            cfw_draw_rounded_span(row, target.width, left, right, fill, 1);
        } else if (w <= 2 || h <= 2 || yy == 0 || yy == (int32_t)h - 1) {
            cfw_draw_rounded_span(row, target.width, left, right, border, 0);
        } else {
            inner_inset = cfw_draw_rounded_inset(yy - 1, h - 2, inner_radius, inner_inset);
            int32_t inner_left = x + 1 + inner_inset;
            int32_t inner_right = x + (int32_t)w - 1 - inner_inset;
            cfw_draw_rounded_span(row, target.width, left, inner_left, border, 0);
            cfw_draw_rounded_span(row, target.width, inner_left, inner_right, fill, 1);
            cfw_draw_rounded_span(row, target.width, inner_right, right, border, 0);
        }
    }
    return 0;
}

/* [color8] fills the whole target, padding included. A clear has no position,
 * so depth does not apply. */
static int cfw_draw_op_clear(const cfw_draw_env *env, cfw_reader r, cfw_draw_target target) {
    uint32_t color = READ_U8(r);
    if (!READ_DONE(r) || color > 15) {
        return -1;
    }
    if (!env->apply) {
        return 0;
    }
    memset(target.pixels, (int)(color * 17u), target.stride * target.height);
    return 0;
}

/* [display-list-id16] */
static int cfw_draw_op_display_list(const cfw_draw_env *env, cfw_reader r, cfw_draw_target target) {
    uint32_t id = READ_U16(r);
    if (!READ_DONE(r)) {
        return -1;
    }
    return cfw_draw_list(env->ctx, id, target, env->apply, env->walk);
}

/* ---- Call and sequence decoding ------------------------------------------ */

/* [op8][flags8] [resource-target-id16 if DRAW_FLAG_RESOURCE_TARGET] [depth s8 if DRAW_FLAG_DEPTH] [op payload] */
static int cfw_draw_call(customCfwContext *ctx, const uint8_t *p, uint32_t n,
                         cfw_draw_target target, int apply, cfw_draw_walk *walk) {
    if (!walk->remaining--) {
        return -1;
    }
    cfw_reader r = cfw_reader_init(p, n);
    cfw_draw_op op = (cfw_draw_op)READ_U8(r);
    uint32_t flags = READ_U8(r);
    if (flags & ~DRAW_FLAGS_MASK) {
        return -1;
    }

    if (flags & DRAW_FLAG_RESOURCE_TARGET) {
        uint32_t id = READ_U16(r);
        if (!READ_IN_BOUNDS(r)) {
            return -1;
        }
        int32_t inherited_shift = target.shift_x;
        if (cfw_draw_resource_target(ctx, id, &target)) {
            return -1;
        }
        target.shift_x = inherited_shift;
        cfw_draw_ref(walk, id);
    }

    if (flags & DRAW_FLAG_DEPTH) {
        /* Split the horizontal separation between the two lenses; the right
         * lens takes the odd pixel. RoundedDepthTest pins the exact rounding. */
        int32_t depth = READ_S8(r);
        int32_t value = cfw_draw_right_lens() ? depth + 1 : depth;
        int32_t half = value < 0 ? (value - 1) / 2 : value / 2;
        target.shift_x += cfw_draw_right_lens() ? -half : half;
    }

    if (!READ_IN_BOUNDS(r)) {
        return -1;
    }

    cfw_draw_env env;
    env.ctx = ctx;
    env.walk = walk;
    env.apply = apply;
    switch (op) {
    case DRAW_OP_BOUNDING_BOX:
        return cfw_draw_op_bounding_box(&env, r, target);
    case DRAW_OP_RECT_COPY:
        return cfw_draw_op_rect_copy(&env, r, target);
    case DRAW_OP_STOCK_FONT_STRING:
        return cfw_draw_op_stock_font_string(&env, r, target);
    case DRAW_OP_IMAGE:
        return cfw_draw_op_image(&env, r, target);
    case DRAW_OP_TEXT:
        return cfw_draw_op_text(&env, r, target);
    case DRAW_OP_REMAP_COLORS:
        return cfw_draw_op_remap_colors(&env, r, target);
    case DRAW_OP_DISPLAY_LIST:
        return cfw_draw_op_display_list(&env, r, target);
    case DRAW_OP_ROUNDED_RECT:
        return cfw_draw_op_rounded_rect(&env, r, target);
    case DRAW_OP_CLEAR:
        return cfw_draw_op_clear(&env, r, target);
    default:
        return -1;
    }
}

/* [count16] { [length16][draw call] } */
static int cfw_draw_sequence(customCfwContext *ctx, const uint8_t *p, uint32_t n,
                             cfw_draw_target target, int apply, cfw_draw_walk *walk) {
    cfw_reader r = cfw_reader_init(p, n);
    uint32_t count = READ_U16(r);
    if (!READ_IN_BOUNDS(r)) {
        return -1;
    }
    for (uint32_t i = 0; i < count; i++) {
        uint32_t length = READ_U16(r);
        const uint8_t *call = READ_BYTES(r, length);
        if (!call) {
            return -1;
        }
        if (cfw_draw_call(ctx, call, length, target, apply, walk)) {
            return -1;
        }
    }
    return READ_DONE(r) ? 0 : -1;
}

static void cfw_draw_walk_init(cfw_draw_walk *walk, uint8_t *references) {
    walk->remaining = CFW_DRAW_MAX_OPS;
    walk->depth = 0;
    walk->references = references;
    walk->elapsed_ms=0;
    walk->animation_pending=0;
}

static int cfw_draw_run(customCfwContext *ctx, const uint8_t *p, uint32_t n,
                        cfw_draw_target target, int apply, uint8_t *references) {
    cfw_draw_walk walk;
    cfw_draw_walk_init(&walk, references);
    walk.elapsed_ms=ctx->draw_elapsed_ms;
    return cfw_draw_sequence(ctx, p, n, target, apply, &walk);
}

static int cfw_draw_root(customCfwContext *ctx, uint32_t id, cfw_draw_target target, int apply, uint8_t *references) {
    if (id == CFW_DRAW_SCREEN) {
        return 0;
    }
    cfw_draw_walk walk;
    cfw_draw_walk_init(&walk, references);
    walk.elapsed_ms=ctx->draw_elapsed_ms;
    int result=cfw_draw_list(ctx,id,target,apply,&walk);
    if (!result && apply) ctx->animation_pending |= walk.animation_pending;
    return result;
}
