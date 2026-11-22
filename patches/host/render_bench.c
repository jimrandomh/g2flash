/* Renderer replay driver for render_bench.py.
 *
 * Compiled with build.py's flags into a position-independent Thumb blob that
 * holds the firmware's own resource cache, texture and display-list code plus
 * the three entry points below, and run under Unicorn. There is no firmware
 * around it, so everything it touches sits at fixed addresses that
 * render_bench.py maps; keep the two in sync. The stock font (draw op 3)
 * calls into the stock image and cannot run here. */
#include <stdint.h>
#include "cfw_context.h"
#include "debug.h"
#include "memory.c"
#include "utils.c"
#include "resource_cache.h"

#define BENCH_CTX ((customCfwContext *)0x20100000u)
#define BENCH_RIGHT_LENS (*(volatile uint32_t *)0x20110000u)
#define BENCH_CACHE ((uint8_t *)0x20200000u)
#define BENCH_SCREEN ((uint8_t *)0x20300000u)
#define BENCH_FRAMEBUFFER ((uint8_t *)0x20340000u)
#define BENCH_FRAME_BYTES (640u * 480u / 2u)

static customCfwContext *getCustomCfwContext(void) { return BENCH_CTX; }
int cfw_fb_lease_active(void) { return 1; }
#define CFW_RESOURCE_CACHE_STORAGE BENCH_CACHE
static void rl_add(cfw_rectlist *rl, uint32_t l, uint32_t t, uint32_t w, uint32_t h) {
    (void)rl; (void)l; (void)t; (void)w; (void)h;
}

#include "rle.c"
#include "resource_cache.c"
#include "texture_draw.c"
static int cfw_draw_right_lens(void) { return (int)BENCH_RIGHT_LENS; }
#include "display_list.c"

#ifdef RENDER_BENCH_OLD_RLE
/* Source trees from before rle.c's span decoder kept this in zlib_glue.c. */
static int decode_image_rle(const uint8_t *src, uint32_t size, uint8_t *base,
                            uint32_t stride, uint32_t rowbytes, uint32_t rows) {
    rle_state rs;
    rle_init(&rs, base, stride, rowbytes, rows);
    rle_feed(&rs, src, size);
    return !rs.err && rs.left == 0 && rs.st == 0;
}
#endif

/* Compose one frame into the framebuffer, as cfw_animation_present and
 * cfw_present_draw do between them. Returns 1 while the root list is still
 * animating, 0 when it is not, -1 on failure. */
__attribute__((noinline)) int bench_present(uint32_t elapsed_ms) {
    customCfwContext *ctx = BENCH_CTX;
    ctx->animation_pending = 0;
    ctx->draw_elapsed_ms = elapsed_ms;
    uint32_t root = ctx->root_display_list ? ctx->root_display_list - 1u : CFW_DRAW_SCREEN;
    cfw_draw_target target = {BENCH_FRAMEBUFFER, 640, 480, 320, 0, 0, {0, 0, 0, 0}};
    if (cfw_draw_root(ctx, root, target, 0, 0)) return -1;
    int result = 0;
    if (root != CFW_DRAW_SCREEN && cfw_draw_root(ctx, root, target, 1, 0)) {
        result = -1;
        root = CFW_DRAW_SCREEN;
    }
    if (root == CFW_DRAW_SCREEN) memcpy(BENCH_FRAMEBUFFER, ctx->screen_buffer, BENCH_FRAME_BYTES);
    return result ? result : ctx->animation_pending ? 1 : 0;
}

/* One private-stream message, as image_dispatch handles the renderer's types
 * (see cfw_message_type in zlib_glue.c). Returns -2 for any other type. */
__attribute__((noinline)) int bench_message(const uint8_t *src, uint32_t srclen) {
    enum { UPLOAD = 21, EVICT = 22, DRAW_CALLS = 26, SET_ROOT = 27, PRESENT = 28, CREATE_SURFACE = 29 };
    customCfwContext *ctx = BENCH_CTX;
    ctx->screen_buffer = BENCH_SCREEN;
    uint32_t mode = src[0] & 127u;
    if (mode == UPLOAD) return cfw_resource_upload(src + 1, srclen - 1);
    if (mode == EVICT) return cfw_resource_evict(src + 1, srclen - 1);
    if (mode == CREATE_SURFACE) return cfw_resource_create(src + 1, srclen - 1);
    cfw_draw_target target = {BENCH_SCREEN, 640, 480, 320, 0, 0, {0, 0, 0, 0}};
    if (mode == DRAW_CALLS) {
        ctx->draw_elapsed_ms = 0;
        if (cfw_draw_run(ctx, src + 1, srclen - 1, target, 0, 0)) return -1;
        return cfw_draw_run(ctx, src + 1, srclen - 1, target, 1, 0);
    }
    if (mode == SET_ROOT) {
        if (srclen != 3) return -1;
        uint32_t id = rd16(src + 1);
        ctx->draw_elapsed_ms = 0;
        if (cfw_draw_root(ctx, id, target, 0, 0)) return -1;
        ctx->root_display_list = id == 65535u ? 0 : (uint16_t)(id + 1);
        return 0;
    }
    if (mode == PRESENT) return bench_present(0);
    return -2;
}

/* FNV-1a over every complete resource, in ID order. */
__attribute__((noinline)) uint32_t bench_resource_hash(void) {
    uint32_t hash = 2166136261u;
    for (uint32_t id = 0; id < CFW_RESOURCE_COUNT; id++) {
        uint32_t size = 0;
        const uint8_t *data = cfw_resource_get(BENCH_CTX, id, &size);
        if (!data) continue;
        hash = (hash ^ id) * 16777619u;
        for (uint32_t i = 0; i < size; i++) hash = (hash ^ data[i]) * 16777619u;
    }
    return hash;
}
