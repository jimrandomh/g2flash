#include "cfw_context.h"

#define CFW_FRAMEBUFFER_BYTES (640u * 480u / 2u)

/* The screen buffer has a fixed address (ram_layout.h), so claiming it cannot
 * fail. ctx->screen_buffer is null until the first draw of a session and after
 * mode 11; claiming clears whatever the previous session left. */
#ifndef CFW_SCREEN_BUFFER_STORAGE
#include "ram_layout.h"
_Static_assert(CFW_FRAMEBUFFER_BYTES == CFW_SCREEN_BUFFER_SIZE, "screen buffer size");
#define CFW_SCREEN_BUFFER_STORAGE ((uint8_t *)CFW_SCREEN_BUFFER_ADDR)
#endif

static void cfw_framebuffers_release(customCfwContext *ctx) {
    ctx->screen_buffer = 0;
    ctx->root_display_list = 0;
}

static uint8_t *cfw_screen_buffer(void) {
    customCfwContext *ctx=getCustomCfwContext();
    if(!ctx) return 0;
    if(!ctx->screen_buffer) {
        uint8_t *p=CFW_SCREEN_BUFFER_STORAGE;
        bzero(p,CFW_FRAMEBUFFER_BYTES);ctx->screen_buffer=p;
    }
    return ctx->screen_buffer;
}
