#include "cfw_context.h"

#define CFW_FRAMEBUFFER_BYTES (640u * 480u / 2u)
#ifndef CFW_IMAGE_ALLOC
#define CFW_IMAGE_ALLOC cfw_heap13_malloc
#define CFW_IMAGE_FREE cfw_heap13_free
#endif

/* The caller owns the display gate until its refresh is consumed. */
static uint8_t *cfw_composition_buffer(void) {
    customCfwContext *ctx = getCustomCfwContext();
    if (!ctx) return 0;
    if (!ctx->composition_buffer) {
        uint8_t *buffer = CFW_IMAGE_ALLOC(CFW_FRAMEBUFFER_BYTES);
        if (!buffer) return 0;
        for (uint32_t i = 0; i < CFW_FRAMEBUFFER_BYTES; ++i) buffer[i] = 0;
        ctx->composition_buffer = buffer;
    }
    return ctx->composition_buffer;
}

static void cfw_framebuffers_release(customCfwContext *ctx) {
    if (ctx->composition_buffer) CFW_IMAGE_FREE(ctx->composition_buffer);
    ctx->composition_buffer = 0;
    if (ctx->screen_buffer) FW_FREE(ctx->screen_buffer);
    ctx->screen_buffer = 0;
    ctx->root_display_list = 0;
}

static uint8_t *cfw_screen_buffer(void) {
    customCfwContext *ctx=getCustomCfwContext();
    if(!ctx) return 0;
    if(!ctx->screen_buffer) {
        uint8_t *p=(uint8_t *)cfw_malloc(CFW_FRAMEBUFFER_BYTES);
        if(!p) return 0;
        bzero(p,CFW_FRAMEBUFFER_BYTES);ctx->screen_buffer=p;
    }
    return ctx->screen_buffer;
}
