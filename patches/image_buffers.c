#include "cfw_context.h"

#define CFW_FRAMEBUFFER_BYTES (640u * 480u / 2u)
#ifndef CFW_IMAGE_ALLOC
#define CFW_IMAGE_ALLOC cfw_heap13_malloc
#define CFW_IMAGE_FREE cfw_heap13_free
#endif

/* The caller owns the display gate until its refresh is consumed. */
static uint8_t *cfw_shadow_buffer(void) {
    customCfwContext *ctx = getCustomCfwContext();
    if (!ctx) return 0;
    if (!ctx->framebuffer_shadow) {
        uint8_t *buffer = CFW_IMAGE_ALLOC(CFW_FRAMEBUFFER_BYTES);
        if (!buffer) return 0;
        for (uint32_t i = 0; i < CFW_FRAMEBUFFER_BYTES; ++i) buffer[i] = 0;
        ctx->framebuffer_shadow = buffer;
    }
    return ctx->framebuffer_shadow;
}

static void cfw_shadow_release(customCfwContext *ctx) {
    if (ctx->framebuffer_shadow) CFW_IMAGE_FREE(ctx->framebuffer_shadow);
    ctx->framebuffer_shadow = 0;
}
