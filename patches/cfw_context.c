#include "cfw_context.h"

/* Return the singleton only if it already exists and passes the slot/magic checks.
 * Ordinary stock refreshes pass through display_copy_hook, so that hook must never
 * allocate CFW state. */
static customCfwContext *peekCustomCfwContext(void) {
    customCfwContext *ctx = *(customCfwContext **)CFW_CTX_SLOT;
    if (((uintptr_t)ctx & 3) == 0 && (uintptr_t)ctx - 0x20000000u < 0x00800000u &&
        ctx->magic == CFW_CTX_MAGIC)
        return ctx;
    return 0;
}

/* Fetch (or lazily create) the CFW singleton context. Its pointer lives in the
 * explicitly reserved primary-TLSF tail (CFW_CTX_SLOT); we only ever touch that
 * word through this helper (image traffic or the private settings lease). The
 * slot ptr is range-checked to SRAM and the struct's magic verified before
 * trusting it, so warm-reset garbage can't be mistaken for a live context.
 * Returns 0 if the one-time struct malloc fails. */
static customCfwContext *getCustomCfwContext(void) {
    customCfwContext *ctx = peekCustomCfwContext();
    if (ctx) return ctx;
    ctx = (customCfwContext *)cfw_malloc(sizeof(customCfwContext));
    if (ctx) {
        bzero((uint8_t *)ctx, sizeof(customCfwContext));
        ctx->magic = CFW_CTX_MAGIC;
        ctx->diag_hide = 1;    /* overlay off by default; mode 7 sub 2 turns it on */
        for (uint32_t i = 0; i < CFW_FID_RING; i++) ctx->recent_fids[i] = 0xffff;  /* sentinel */
    }
    *(customCfwContext **)CFW_CTX_SLOT = ctx;      /* 0 on OOM: retried next message */
    return ctx;
}
