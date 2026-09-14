#include "cfw_context.h"

#define CFW_FRAMEBUFFER_BYTES (640u * 480u / 2u)
#define CFW_SNAP_RESERVED ((uint8_t *)(uintptr_t)1)
#ifndef CFW_IMAGE_ALLOC
#define CFW_IMAGE_ALLOC cfw_heap13_malloc
#define CFW_IMAGE_FREE cfw_heap13_free
#endif

/* The caller owns the display gate until its refresh is consumed. A single
 * full-panel shadow serves both NULL-state messages and legacy container input. */
static uint8_t *cfw_shadow_buffer(uint8_t *state) {
    (void)state;
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

/* Only an owner of a BUSY slot clears it. Publish emptiness last; the producer
 * may immediately reuse the slot, so no fields are touched after that store. */
static void cfw_snap_clear(cfw_snap *snap) {
    uint8_t *buffer = snap->buf;
    snap->buf = 0;
    snap->len = 0;
    __atomic_store_n(&snap->seq, CFW_SNAP_BUSY_SEQ, __ATOMIC_RELAXED);
    __atomic_store_n(&snap->state, (uint8_t *)0, __ATOMIC_RELEASE);
    if (buffer) CFW_IMAGE_FREE(buffer);
}

/* state=NULL selects any pending legacy message (eviction/cleanup). Actual
 * NULL-state private messages bypass this FIFO because they already own input. */
static cfw_snap *cfw_snap_take(customCfwContext *ctx, uint8_t *state) {
    for (;;) {
        cfw_snap *oldest = 0;
        uint32_t seq = CFW_SNAP_BUSY_SEQ;
        for (unsigned i = 0; i < CFW_SNAP_RING; ++i) {
            cfw_snap *snap = &ctx->snaps[i];
            uint8_t *owner = __atomic_load_n(&snap->state, __ATOMIC_ACQUIRE);
            uint32_t candidate = __atomic_load_n(&snap->seq, __ATOMIC_RELAXED);
            if (!owner || owner == CFW_SNAP_RESERVED || (state && owner != state)) continue;
            if (candidate < seq) { seq = candidate; oldest = snap; }
        }
        if (!oldest) return 0;
        if (__atomic_compare_exchange_n(&oldest->seq, &seq, CFW_SNAP_BUSY_SEQ,
                                         0, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
            uint8_t *owner = __atomic_load_n(&oldest->state, __ATOMIC_ACQUIRE);
            if (owner && owner != CFW_SNAP_RESERVED && (!state || state == owner)) return oldest;
            /* The slot was recycled while scanning; don't consume another
             * container's message or a producer's not-yet-published record. */
            __atomic_store_n(&oldest->seq, seq, __ATOMIC_RELEASE);
        }
    }
}

static void cfw_snap_discard_pending(customCfwContext *ctx) {
    cfw_snap *snap;
    while ((snap = cfw_snap_take(ctx, 0))) cfw_snap_clear(snap);
}

static int cfw_snapshot_enqueue(uint8_t *state, const uint8_t *src, uint32_t length) {
    customCfwContext *ctx = getCustomCfwContext();
    if (!ctx || !state || !src || !length) return -1;
    uint8_t *copy = CFW_IMAGE_ALLOC(length);
    if (!copy) { ctx->f_snap_of = 1; return -1; }
    for (uint32_t i = 0; i < length; ++i) copy[i] = src[i];
    for (;;) {
        for (unsigned i = 0; i < CFW_SNAP_RING; ++i) {
            cfw_snap *snap = &ctx->snaps[i];
            uint8_t *empty = 0;
            if (!__atomic_compare_exchange_n(&snap->state, &empty, CFW_SNAP_RESERVED,
                                              0, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) continue;
            snap->buf = copy;
            snap->len = length;
            uint32_t seq = __atomic_fetch_add(&ctx->snap_seq, 1u, __ATOMIC_RELAXED);
            if (seq == CFW_SNAP_BUSY_SEQ) seq = __atomic_fetch_add(&ctx->snap_seq, 1u, __ATOMIC_RELAXED);
            __atomic_store_n(&snap->seq, seq, __ATOMIC_RELEASE);
            __atomic_store_n(&snap->state, state, __ATOMIC_RELEASE);
            return 0;
        }
        cfw_snap *victim = cfw_snap_take(ctx, 0);
        ctx->f_snap_of = 1;
        if (!victim) { CFW_IMAGE_FREE(copy); return -1; } /* All slots busy. */
        cfw_snap_clear(victim);
    }
}
