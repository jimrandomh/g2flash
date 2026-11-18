#include "cfw_context.h"

#define CFW_FRAMEBUFFER_BYTES (640u * 480u / 2u)

static void cfw_framebuffers_release(customCfwContext *ctx) {
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
