#include <stdint.h>
#include "cfw_context.h"
#include "debug.h"
#include "texture_cache.h"

/* Cached image format:
 *
 *   [width:u8][height:u8][4bpp RLE tokens...]
 *
 * RLE uses the same tokens as modes 3/6, but covers exactly width*height
 * pixels (there is no packed-row pad nibble). Since an image has no encoded
 * byte length, a valid stream ends at the first token that completes that
 * pixel count. The scanner never reads past the 64 KiB cache. */
typedef struct {
    const uint8_t *rle;
    uint32_t rle_len;
    uint32_t width;
    uint32_t height;
} cfw_cached_image;

#define CFW_TEXTURE_OPT_TRANSPARENT 0x10u
#define CFW_TEXTURE_OPT_INVERSE     0x20u

/* Stock LVGL font ABI (firmware 2.2.9.22, Thumb entry points). After building
 * the background chain, the font manager dereferences its private 12-byte
 * wrapper and publishes the live 20 px lv_font_t root in this SRAM slot.
 * Using lv_font_get_glyph_dsc with letter_next preserves each font's stock
 * pair-kerning behavior, including fallback selection. */
typedef int (*cfw_font_dsc_fn)(const void *font, void *dsc,
                               uint32_t letter, uint32_t letter_next);
typedef const uint8_t *(*cfw_font_bitmap_fn)(void *dsc, void *draw_buf);
typedef void (*cfw_font_release_fn)(void *dsc);

#define CFW_FONT_GET_DSC ((cfw_font_dsc_fn)0x004e64fdU)
#define CFW_FONT_GET_BITMAP ((cfw_font_bitmap_fn)0x004e644fU)
#define CFW_FONT_RELEASE ((cfw_font_release_fn)0x004e64a1U)
#define CFW_FONT20_ROOT (*(const uint8_t * volatile *)0x200769acU)

/* The stock build uses the embedded ARM short-enum ABI. Keep this opaque so
 * our compiler's enum-size flags cannot silently change the layout.
 * lv_font_glyph_dsc_t offsets: resolved font 0, metrics 4..12, format 14,
 * flags 15/20, glyph id 24, cache entry 28; total size 32. */
#define CFW_GLYPH_DSC_SIZE 32u
#define CFW_GLYPH_ADV_W     4u
#define CFW_GLYPH_BOX_W     6u
#define CFW_GLYPH_BOX_H     8u
#define CFW_GLYPH_OFS_X    10u
#define CFW_GLYPH_OFS_Y    12u
#define CFW_GLYPH_FORMAT   14u
#define CFW_GLYPH_RAW_FLAG 20u
#define CFW_GLYPH_A4_ALIGNED 0x14u
#define CFW_DRAW_BUF_SIZE 28u
#define CFW_FONT_LINE_HEIGHT 12u
#define CFW_FONT_BASE_LINE   16u
#define CFW_TEXT_CONTROL_FLAG 0x80000000u

/* Decode one RLE token without reading beyond `len`. Return its byte length and
 * pixel count, rejecting zero counts and truncated escape forms. */
static int cfw_texture_rle_token(const uint8_t *p, uint32_t len,
                                 uint32_t *used, uint32_t *count,
                                 uint8_t *color) {
    if (len < 1) return 0;
    uint8_t op = p[0];
    *color = (uint8_t)(op & 0x0fu);
    uint32_t n = op >> 4;
    if (n) {
        *used = 1;
        *count = n;
        return 1;
    }
    if (len < 2) return 0;
    n = p[1];
    if (n) {
        *used = 2;
        *count = n;
        return 1;
    }
    if (len < 4) return 0;
    n = (uint32_t)p[2] | ((uint32_t)p[3] << 8);
    if (n == 0) return 0;
    *used = 4;
    *count = n;
    return 1;
}

/* Validate one complete cached image and determine exactly how many RLE bytes
 * it occupies. This pass happens before drawing so malformed cache contents
 * cannot leave a partially modified shadow. */
static int cfw_texture_image_at(customCfwContext *ctx, uint32_t offset,
                                cfw_cached_image *out) {
    if (ctx == 0 || ctx->texture_cache == 0 ||
        offset > CFW_TEXTURE_CACHE_SIZE - 2u)
        return 0;

    const uint8_t *image = ctx->texture_cache + offset;
    uint32_t width = image[0];
    uint32_t height = image[1];
    if (width == 0 || height == 0) return 0;

    const uint8_t *p = image + 2;
    uint32_t available = CFW_TEXTURE_CACHE_SIZE - offset - 2u;
    uint32_t pos = 0;
    uint32_t left = width * height;
    while (left) {
        uint32_t used, count;
        uint8_t color;
        if (!cfw_texture_rle_token(p + pos, available - pos,
                                   &used, &count, &color) || count > left)
            return 0;
        (void)color;
        pos += used;
        left -= count;
    }

    out->rle = p;
    out->rle_len = pos;
    out->width = width;
    out->height = height;
    return 1;
}

/* Build the 4-bit color LUT. `top` 15 is the identity ramp; smaller values
 * scale the 0..15 source range proportionally into 0..top. Inverse reverses
 * the completed ramp. */
static void cfw_texture_make_lut(uint8_t options, uint8_t *lut) {
    uint32_t top = options & 0x0fu;
    for (uint32_t i = 0; i < 16u; i++) {
        uint32_t source = (options & CFW_TEXTURE_OPT_INVERSE) ? 15u - i : i;
        lut[i] = (uint8_t)((source * top) / 15u);
    }
}

/* Render a previously validated image, clipping signed coordinates to the
 * physical packed-4bpp shadow. Transparency tests the original source value,
 * before the LUT, so source color 0 is skipped even for an inverse ramp. */
static void cfw_texture_render(uint8_t *shadow, uint32_t stride,
                               uint32_t panel_w, uint32_t panel_h,
                               int32_t x0, int32_t y0,
                               const cfw_cached_image *image,
                               const uint8_t *lut, int transparent) {
    uint32_t pos = 0;
    uint32_t pixel = 0;
    while (pos < image->rle_len) {
        uint32_t used, count;
        uint8_t color;
        /* The validation pass guarantees success here. */
        if (!cfw_texture_rle_token(image->rle + pos,
                                   image->rle_len - pos,
                                   &used, &count, &color))
            return;
        pos += used;
        int skip = transparent && color == 0;
        uint8_t mapped = lut[color];
        for (uint32_t i = 0; i < count; i++, pixel++) {
            int32_t x = x0 + (int32_t)(pixel % image->width);
            int32_t y = y0 + (int32_t)(pixel / image->width);
            if (!skip && x >= 0 && y >= 0 &&
                (uint32_t)x < panel_w && (uint32_t)y < panel_h) {
                uint8_t *b = shadow + (uint32_t)y * stride + ((uint32_t)x >> 1);
                if (x & 1) *b = (uint8_t)((*b & 0xf0u) | mapped);
                else       *b = (uint8_t)((*b & 0x0fu) | (uint8_t)(mapped << 4));
            }
        }
    }
}

/* Render stock LVGL's raw A4-aligned glyph data (high nibble first, each row
 * byte-aligned). Unlike cached RLE images, glyph boxes can have signed offsets. */
static void cfw_texture_render_a4(uint8_t *shadow, uint32_t stride,
                                  uint32_t panel_w, uint32_t panel_h,
                                  int32_t x0, int32_t y0,
                                  const uint8_t *bitmap,
                                  uint32_t width, uint32_t height,
                                  const uint8_t *lut, int transparent) {
    /* Even's LV_FONT_FMT_PLAIN_ALIGNED encoder advances to a fresh byte after
     * every row even when an even-width row already ended on a byte boundary.
     * Thus every row occupies floor(width/2)+1 bytes, including a whole pad
     * byte for even widths. This is verified by every embedded 20/22 px font
     * and the external Source Han Sans font partition. */
    uint32_t row_stride = (width >> 1) + 1u;
    for (uint32_t sy = 0; sy < height; sy++) {
        int32_t y = y0 + (int32_t)sy;
        if (y < 0 || (uint32_t)y >= panel_h) continue;
        const uint8_t *row = bitmap + sy * row_stride;
        for (uint32_t sx = 0; sx < width; sx++) {
            int32_t x = x0 + (int32_t)sx;
            uint8_t packed = row[sx >> 1];
            uint8_t color = (sx & 1u) ? (packed & 0x0fu) : (packed >> 4);
            if ((transparent && color == 0) || x < 0 || (uint32_t)x >= panel_w)
                continue;
            uint8_t mapped = lut[color];
            uint8_t *dst = shadow + (uint32_t)y * stride + ((uint32_t)x >> 1);
            if (x & 1) *dst = (uint8_t)((*dst & 0xf0u) | mapped);
            else       *dst = (uint8_t)((*dst & 0x0fu) | (uint8_t)(mapped << 4));
        }
    }
}

static void cfw_texture_add_rect(cfw_rectlist *rl, int32_t x, int32_t y,
                                 uint32_t width, uint32_t height,
                                 uint32_t panel_w, uint32_t panel_h) {
    int32_t right = x + (int32_t)width;
    int32_t bottom = y + (int32_t)height;
    int32_t left = x < 0 ? 0 : x;
    int32_t top = y < 0 ? 0 : y;
    if (right > (int32_t)panel_w) right = (int32_t)panel_w;
    if (bottom > (int32_t)panel_h) bottom = (int32_t)panel_h;
    if (left < right && top < bottom)
        rl_add(rl, (uint32_t)left, (uint32_t)top,
               (uint32_t)(right - left), (uint32_t)(bottom - top));
}

/* Clear the published pointer before freeing so repeated release/cleanup is
 * harmless. The heap coordinator serializes against other heap-13 users. */
static void cfw_texture_cache_release(customCfwContext *ctx) {
    if (ctx && ctx->texture_cache) {
        uint8_t *cache = ctx->texture_cache;
        ctx->texture_cache = 0;
        cfw_heap13_free(cache);
    }
}

/* Mode 12 payload: a list of [offset:u16][length:u16][data...] entries. Validate
 * the complete list before allocating or writing, then lazily allocate and zero
 * the 64 KiB phone-owned region on the first nonempty write. */
static int cfw_texture_cache_update(const uint8_t *src, uint32_t len) {
    if (src == 0) return -1;
    uint32_t pos = 0;
    int has_data = 0;
    while (pos < len) {
        if (len - pos < 4u) return -1;
        uint32_t offset = rd16(src + pos);
        uint32_t entry_len = rd16(src + pos + 2u);
        pos += 4u;
        if (entry_len > len - pos || offset + entry_len > CFW_TEXTURE_CACHE_SIZE)
            return -1;
        if (entry_len) has_data = 1;
        pos += entry_len;
    }

    if (!has_data) return 0;
    if (!cfw_fb_lease_active()) return -1;
    customCfwContext *ctx = getCustomCfwContext();
    if (ctx == 0) return -1;
    if (ctx->texture_cache == 0) {
        uint8_t *cache = (uint8_t *)cfw_heap13_malloc(CFW_TEXTURE_CACHE_SIZE);
        if (cache == 0) return -1;
        bzero(cache, CFW_TEXTURE_CACHE_SIZE);
        ctx->texture_cache = cache;
    }

    pos = 0;
    while (pos < len) {
        uint32_t offset = rd16(src + pos);
        uint32_t entry_len = rd16(src + pos + 2u);
        pos += 4u;
        for (uint32_t i = 0; i < entry_len; i++)
            ctx->texture_cache[offset + i] = src[pos + i];
        pos += entry_len;
    }
    return 0;
}

/* Mode 13 payload: [offset:u16][x:u16][y:u16][options:u8]. */
static int cfw_texture_draw_image(uint8_t *shadow, uint32_t stride,
                                  uint32_t panel_w, uint32_t panel_h,
                                  const uint8_t *src, uint32_t len,
                                  cfw_rectlist *rl) {
    if (shadow == 0 || src == 0 || len != 7u || !cfw_fb_lease_active()) return -1;
    customCfwContext *ctx = getCustomCfwContext();
    cfw_cached_image image;
    if (!cfw_texture_image_at(ctx, rd16(src), &image)) return -1;
    int32_t x = (int32_t)rd16(src + 2);
    int32_t y = (int32_t)rd16(src + 4);
    uint8_t options = src[6];
    uint8_t lut[16];
    cfw_texture_make_lut(options, lut);
    cfw_texture_render(shadow, stride, panel_w, panel_h, x, y, &image,
                       lut, (options & CFW_TEXTURE_OPT_TRANSPARENT) != 0);
    cfw_texture_add_rect(rl, x, y, image.width, image.height, panel_w, panel_h);
    return 0;
}

/* Mode 14 payload: [font-offset:u16][x:u16][y:u16][options:u8][strlen:u8][string].
 * The font starts with 96 little-endian uint16 image offsets for characters
 * 32..127. Bytes 1..31 adjust x by -10..20; byte 0 and bytes >127 are invalid. */
static int cfw_texture_draw_string(uint8_t *shadow, uint32_t stride,
                                   uint32_t panel_w, uint32_t panel_h,
                                   const uint8_t *src, uint32_t len,
                                   cfw_rectlist *rl) {
    if (shadow == 0 || src == 0 || len < 8u || !cfw_fb_lease_active()) return -1;
    uint32_t font_offset = rd16(src);
    uint8_t options = src[6];
    uint32_t string_len = src[7];
    if (len != 8u + string_len ||
        font_offset > CFW_TEXTURE_CACHE_SIZE - 96u * 2u)
        return -1;

    customCfwContext *ctx = getCustomCfwContext();
    if (ctx == 0 || ctx->texture_cache == 0) return -1;
    const uint8_t *table = ctx->texture_cache + font_offset;
    const uint8_t *string = src + 8;
    int32_t x = (int32_t)rd16(src + 2);
    int32_t y = (int32_t)rd16(src + 4);

    /* Validate every character/table entry/RLE stream before drawing any glyph. */
    int32_t scan_x = x;
    uint8_t lut[16];
    cfw_texture_make_lut(options, lut);
    int transparent = (options & CFW_TEXTURE_OPT_TRANSPARENT) != 0;
    for (uint32_t i = 0; i < string_len; i++) {
        uint32_t ch = string[i];
        if (ch >= 1u && ch <= 31u) {
            scan_x += (int32_t)ch - 11;
            continue;
        }
        if (ch < 32u || ch > 127u) return -1;
        uint32_t image_offset = rd16(table + (ch - 32u) * 2u);
        cfw_cached_image image;
        if (!cfw_texture_image_at(ctx, image_offset, &image)) return -1;
        scan_x += (int32_t)image.width;
    }
    (void)scan_x;

    for (uint32_t i = 0; i < string_len; i++) {
        uint32_t ch = string[i];
        if (ch <= 31u) {
            x += (int32_t)ch - 11;
            continue;
        }
        uint32_t image_offset = rd16(table + (ch - 32u) * 2u);
        cfw_cached_image image;
        /* Already validated above; cache contents cannot change in this handler. */
        if (!cfw_texture_image_at(ctx, image_offset, &image)) return -1;
        cfw_texture_render(shadow, stride, panel_w, panel_h, x, y, &image,
                           lut, transparent);
        cfw_texture_add_rect(rl, x, y, image.width, image.height, panel_w, panel_h);
        x += (int32_t)image.width;
    }
    return 0;
}

/* Decode one strict UTF-8 scalar. Control bytes 1..31 are deliberately handled
 * by the caller before this function; NUL, malformed/overlong encodings,
 * surrogates, and values above U+10FFFF are rejected. */
static int cfw_texture_utf8(const uint8_t *src, uint32_t len,
                            uint32_t *used, uint32_t *codepoint) {
    if (len == 0) return 0;
    uint32_t a = src[0];
    if (a >= 0x20u && a <= 0x7fu) {
        *used = 1;
        *codepoint = a;
        return 1;
    }
    if (a >= 0xc2u && a <= 0xdfu) {
        if (len < 2u || (src[1] & 0xc0u) != 0x80u) return 0;
        *used = 2;
        *codepoint = ((a & 0x1fu) << 6) | (src[1] & 0x3fu);
        return 1;
    }
    if (a >= 0xe0u && a <= 0xefu) {
        if (len < 3u || (src[1] & 0xc0u) != 0x80u ||
            (src[2] & 0xc0u) != 0x80u ||
            (a == 0xe0u && src[1] < 0xa0u) ||
            (a == 0xedu && src[1] >= 0xa0u)) return 0;
        *used = 3;
        *codepoint = ((a & 0x0fu) << 12) |
                     ((uint32_t)(src[1] & 0x3fu) << 6) |
                     (src[2] & 0x3fu);
        return 1;
    }
    if (a >= 0xf0u && a <= 0xf4u) {
        if (len < 4u || (src[1] & 0xc0u) != 0x80u ||
            (src[2] & 0xc0u) != 0x80u || (src[3] & 0xc0u) != 0x80u ||
            (a == 0xf0u && src[1] < 0x90u) ||
            (a == 0xf4u && src[1] >= 0x90u)) return 0;
        *used = 4;
        *codepoint = ((a & 7u) << 18) |
                     ((uint32_t)(src[1] & 0x3fu) << 12) |
                     ((uint32_t)(src[2] & 0x3fu) << 6) |
                     (src[3] & 0x3fu);
        return 1;
    }
    return 0;
}

static uint32_t cfw_texture_next_glyph(const uint32_t *tokens,
                                       uint32_t count, uint32_t pos) {
    for (uint32_t i = pos + 1u; i < count; i++)
        if ((tokens[i] & CFW_TEXT_CONTROL_FLAG) == 0) return tokens[i];
    return 0;
}

static int32_t cfw_texture_s16(const uint8_t *p) {
    return (int32_t)(int16_t)rd16(p);
}

static int32_t cfw_texture_s32(const uint8_t *p) {
    uint32_t value = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                     ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    return (int32_t)value;
}

/* Load a descriptor and, when it has a visible bitmap, request and validate
 * stock's raw A4-aligned data. Missing glyphs retain LVGL's default advance but
 * have no bitmap. Returns 1 for a valid token and 0 for an unsupported format
 * or unavailable bitmap. `bitmap` may be null for spaces/missing glyphs. */
static int cfw_builtin_glyph(const uint8_t *font, uint32_t letter,
                             uint32_t letter_next, uint8_t *dsc,
                             const uint8_t **bitmap) {
    bzero(dsc, CFW_GLYPH_DSC_SIZE);
    int found = CFW_FONT_GET_DSC(font, dsc, letter, letter_next);
    *bitmap = 0;
    uint32_t box_w = rd16(dsc + CFW_GLYPH_BOX_W);
    uint32_t box_h = rd16(dsc + CFW_GLYPH_BOX_H);
    if (!found || cfw_texture_s32(dsc) == 0 || box_w == 0 || box_h == 0)
        return 1;
    if (dsc[CFW_GLYPH_FORMAT] != CFW_GLYPH_A4_ALIGNED) return 0;
    dsc[CFW_GLYPH_RAW_FLAG] |= 1u;
    /* lv_font_get_bitmap_fmt_txt reads draw_buf->data before consulting the
     * req_raw_bitmap flag. It does not use that value on the raw fast path, but
     * the draw-buffer object itself must still be non-null. In the stock ABI
     * its data pointer is at +16 and the complete struct is 28 bytes; an
     * all-zero dummy is sufficient and avoids allocating an A8 conversion
     * buffer. */
    uint8_t draw_buf[CFW_DRAW_BUF_SIZE];
    bzero(draw_buf, CFW_DRAW_BUF_SIZE);
    *bitmap = CFW_FONT_GET_BITMAP(dsc, draw_buf);
    if (*bitmap == 0) CFW_FONT_RELEASE(dsc);
    return *bitmap != 0;
}

/* Mode 15 payload: [x:u16][y:u16][options:u8][strlen:u8][UTF-8 string].
 * It draws through the stock background 20 px font chain. Bytes 1..31 retain
 * mode 14's inline x adjustments (-10..20); all other text is strict UTF-8.
 * Supplying the next real glyph to LVGL applies the built-in default kerning. */
static int cfw_builtin_draw_string(uint8_t *shadow, uint32_t stride,
                                   uint32_t panel_w, uint32_t panel_h,
                                   const uint8_t *src, uint32_t len,
                                   cfw_rectlist *rl) {
    if (shadow == 0 || src == 0 || len < 6u || !cfw_fb_lease_active()) return -1;
    uint32_t string_len = src[5];
    if (len != 6u + string_len) return -1;
    const uint8_t *font = CFW_FONT20_ROOT;
    if (font == 0) return -1;
    int32_t line_height = cfw_texture_s32(font + CFW_FONT_LINE_HEIGHT);
    int32_t base_line = cfw_texture_s32(font + CFW_FONT_BASE_LINE);
    if (line_height <= 0 || line_height > 128 ||
        base_line < -128 || base_line > 128) return -1;

    /* At most one token per input byte. Decode the complete payload before any
     * font calls or shadow writes, preserving all-or-nothing syntax handling. */
    uint32_t tokens[255];
    uint32_t token_count = 0;
    uint32_t pos = 0;
    const uint8_t *string = src + 6;
    while (pos < string_len) {
        uint32_t byte = string[pos];
        if (byte >= 1u && byte <= 31u) {
            tokens[token_count++] = CFW_TEXT_CONTROL_FLAG | byte;
            pos++;
            continue;
        }
        uint32_t used, codepoint;
        if (!cfw_texture_utf8(string + pos, string_len - pos,
                              &used, &codepoint)) return -1;
        tokens[token_count++] = codepoint;
        pos += used;
    }

    /* Preflight every raw bitmap before drawing, so bad stock/fallback formats
     * cannot leave a partially modified shadow. */
    for (uint32_t i = 0; i < token_count; i++) {
        if (tokens[i] & CFW_TEXT_CONTROL_FLAG) continue;
        uint8_t dsc[CFW_GLYPH_DSC_SIZE];
        const uint8_t *bitmap;
        if (!cfw_builtin_glyph(font, tokens[i],
                               cfw_texture_next_glyph(tokens, token_count, i),
                               dsc, &bitmap)) return -1;
        if (bitmap) CFW_FONT_RELEASE(dsc);
    }

    int32_t x = (int32_t)rd16(src);
    int32_t y = (int32_t)rd16(src + 2);
    uint8_t options = src[4];
    uint8_t lut[16];
    cfw_texture_make_lut(options, lut);
    int transparent = (options & CFW_TEXTURE_OPT_TRANSPARENT) != 0;
    for (uint32_t i = 0; i < token_count; i++) {
        if (tokens[i] & CFW_TEXT_CONTROL_FLAG) {
            x += (int32_t)(tokens[i] & 0x1fu) - 11;
            continue;
        }
        uint8_t dsc[CFW_GLYPH_DSC_SIZE];
        const uint8_t *bitmap;
        if (!cfw_builtin_glyph(font, tokens[i],
                               cfw_texture_next_glyph(tokens, token_count, i),
                               dsc, &bitmap)) return -1;
        uint32_t box_w = rd16(dsc + CFW_GLYPH_BOX_W);
        uint32_t box_h = rd16(dsc + CFW_GLYPH_BOX_H);
        int32_t gx = x + cfw_texture_s16(dsc + CFW_GLYPH_OFS_X);
        int32_t gy = y + (line_height - base_line) - (int32_t)box_h -
                     cfw_texture_s16(dsc + CFW_GLYPH_OFS_Y);
        if (bitmap) {
            cfw_texture_render_a4(shadow, stride, panel_w, panel_h,
                                  gx, gy, bitmap, box_w, box_h,
                                  lut, transparent);
            cfw_texture_add_rect(rl, gx, gy, box_w, box_h, panel_w, panel_h);
            CFW_FONT_RELEASE(dsc);
        }
        x += (int32_t)rd16(dsc + CFW_GLYPH_ADV_W);
    }
    return 0;
}
