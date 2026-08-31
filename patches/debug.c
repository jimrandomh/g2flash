#include "debug.h"

/* --- glasses-side timing: Arm DWT cycle counter, clock derived by calibration -----
 * CYCCNT is a free-running core-cycle counter (~4 ns). To use it we (1) UNLOCK the DWT
 * via its CoreSight software-lock register (write 0xC5ACCE55 to LAR at DWT_base+0xFB0)
 * — without this, writes to DWT->CTRL are ignored and CYCCNT stays 0 (observed on hw);
 * then (2) set DEMCR.TRCENA and DWT->CTRL.CYCCNTENA. We re-assert all three cheaply per
 * measurement (only the idle SWO-trace block touches DEMCR).
 *
 * To convert cycles->us we need the core clock. The guessed global 0x2007646c reads 0
 * on hardware (it's only written on a DVFS event, if ever), so instead we CALIBRATE:
 * measure how many CYCCNT cycles elapse across one edge of the firmware's 1 ms OS tick
 * (RAM 0x20076d80, SysTick chain) — that IS cycles-per-ms. Cached in the ctx; a bounded
 * spin falls back to 250 MHz if the tick never advances. All divides are 32-bit
 * (hardware UDIV) — a 64-bit divide would emit an external __aeabi_uldivmod build.py
 * rejects. (Limitation: cached across DVFS; a clock switch makes the figure ~stale.) */
#define DWT_DEMCR   (*(volatile uint32_t *)0xE000EDFCU)  /* CoreDebug->DEMCR (TRCENA bit24) */
#define DWT_LAR     (*(volatile uint32_t *)0xE0001FB0U)  /* DWT CoreSight Lock Access Reg */
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000U)  /* DWT->CTRL (CYCCNTENA bit0) */
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004U)  /* DWT->CYCCNT (core cycles) */
#define FW_CORE_HZ  (*(volatile uint32_t *)0x2007646cU)  /* guessed core-clock global (reads 0 on hw) */
#define DWT_UNLOCK_KEY 0xC5ACCE55U


/* Calibrate DWT cycles-per-millisecond against the firmware's 1 ms OS tick, once,
 * cached in the ctx. DWT must already be unlocked + enabled. Bounded spin across two
 * tick edges (~1-2 ms when the tick runs); returns 0 if the tick never advances (the
 * caller then falls back to an assumed clock). */
static uint32_t cfw_cyc_per_ms(customCfwContext *ctx) {
    if (ctx->cyc_per_ms) return ctx->cyc_per_ms;
    uint32_t g = 500000u;
    uint32_t t0 = FW_MS_TICK;
    while (FW_MS_TICK == t0 && --g) ;               /* wait for a tick edge */
    if (g == 0) return 0;
    uint32_t c0 = DWT_CYCCNT, t1 = FW_MS_TICK;
    g = 500000u;
    while (FW_MS_TICK == t1 && --g) ;               /* wait for the next edge (~1 ms) */
    if (g == 0) return 0;
    ctx->cyc_per_ms = DWT_CYCCNT - c0;              /* cycles elapsed across one 1 ms tick */
    return ctx->cyc_per_ms;
}

/* Time a region of code with the DWT cycle counter (see the timing note above):
 *
 *   uint32_t t; cfw_time_start(&t); ...work...; uint32_t us = cfw_time_end(&t);
 *
 * start re-asserts the DWT unlock/enable (cheap, and the state is not ours to assume)
 * and calibrates cycles-per-ms if that hasn't happened yet — deliberately BEFORE the
 * start stamp is taken, so the one-time ~1-2 ms calibration spin is never billed to the
 * region being measured. Regions may nest: by the time an inner start runs, the outer
 * one has already primed the calibration. end converts to microseconds; the subtraction
 * is unsigned, so it tolerates one CYCCNT wrap (~17 s at 250 MHz). */
static void cfw_time_start(uint32_t *t) {
    DWT_LAR   = DWT_UNLOCK_KEY;                     /* unlock the DWT (CoreSight lock) */
    DWT_DEMCR |= (1u << 24);                        /* TRCENA */
    DWT_CTRL  |= 1u;                                /* CYCCNTENA */
    customCfwContext *ctx = getCustomCfwContext();
    if (ctx) cfw_cyc_per_ms(ctx);                   /* calibrate once, outside the window */
    *t = DWT_CYCCNT;
}

static uint32_t cfw_time_end(const uint32_t *t) {
    uint32_t dc = DWT_CYCCNT - *t;
    customCfwContext *ctx = getCustomCfwContext();
    uint32_t cpm = ctx ? cfw_cyc_per_ms(ctx) : 0;      /* calibrated cycles/ms (cached) */
    uint32_t cyc_per_us = cpm ? (cpm / 1000u) : 250u;  /* fallback: assume 250 MHz */
    if (cyc_per_us == 0) cyc_per_us = 1;
    return dc / cyc_per_us;
}

/* Diagnostic: record whether the frames the worker processes arrive in order /
 * skipped / DUPLICATED (mode-3 frame ids). Sticky flags shown by cfw_draw_flags;
 * with the snapshot-FIFO fix these should stay clear. `has_fid`=0 for a mode-6
 * keyframe (no id; it rebaselines the next delta so keyframe gaps aren't "skips").
 * Returns 1 if this fid is a DUPLICATE of a recently-seen one (caller skips). */
static int cfw_diag(int has_fid, uint16_t fid) {
    customCfwContext *ctx = getCustomCfwContext();
    if (ctx == 0) return 0;
    ctx->diag_seen = 1;
    if (!has_fid) { ctx->fid_resync = 1; return 0; }  /* keyframe rebaselines next delta */

    /* duplicate: this fid is still in the recent ring -> flag and tell caller to skip */
    for (uint32_t i = 0; i < CFW_FID_RING; i++)
        if (ctx->recent_fids[i] == fid) { ctx->f_dup = 1; return 1; }

    if (!ctx->fid_resync) {
        uint16_t d = (uint16_t)(fid - ctx->last_fid);
        if (d >= 0x8000u) ctx->f_reorder = 1;   /* went backward (and not a recent dup) */
        else if (d > 1) ctx->f_skip = 1;         /* forward gap */
    }
    ctx->fid_resync = 0;
    ctx->last_fid = fid;
    if (fid > ctx->high_fid) ctx->high_fid = fid;
    ctx->recent_fids[ctx->recent_pos] = fid;
    ctx->recent_pos = (uint8_t)((ctx->recent_pos + 1) % CFW_FID_RING);
    return 0;
}

/* Append (l,t,w,h) to the per-frame updated-rect list, if there's room. */
static void rl_add(cfw_rectlist *rl, uint32_t l, uint32_t t, uint32_t w, uint32_t h) {
    if (rl && rl->n < CFW_RECT_MAX) {
        rl->r[rl->n].l = (uint16_t)l; rl->r[rl->n].t = (uint16_t)t;
        rl->r[rl->n].w = (uint16_t)w; rl->r[rl->n].h = (uint16_t)h;
        rl->n++;
    }
}

static void append_free_kib(char *out, uint32_t free_bytes, uint32_t maxlen) {
    if (free_bytes == TLSF_FREE_INVALID)
        strlcat(out, "?", maxlen);
    else
        u_to_dec(out, free_bytes >> 10, maxlen); /* round down: never overstate */
}

/* Overlay, as a Terminus 6x12 text line across the top-left of the frame (white on a
 * black bar), the diagnostic flags that are set followed by the PREVIOUS message's
 * timings. Flags: REORDER, SKIP, DUP, SNAPOF, ALLOC (the last is set by any failed
 * CFW-owned allocation); normally all should stay clear, so this reads "OK". The timings
 * are microseconds: `w` = the whole image_worker, `p` = the present_shadow step within
 * it (packed framebuffer copy + cache clean), e.g. "OK w834us p210us". The trailing
 * `f13/20/27=A/B/Ck` values are free KiB (rounded down) in the TLSF arenas beginning at
 * 0x201350a8, 0x202020a8, and 0x202728a8 respectively. Suppressed when diag_hide is set
 * (mode 7). Drawn into the physical packed-4bpp framebuffer. */
static void cfw_draw_flags(uint8_t *disp, uint32_t w, uint32_t h) {
    customCfwContext *ctx = getCustomCfwContext();
    if (ctx == 0 || ctx->diag_hide) return;

    char line[96]; line[0] = 0;
    uint32_t num_flags = 0;
    #define ADD_FLAG(cond, name) do {                                         \
        if (cond) {                                                           \
            strlcat(line, name, sizeof(line));                                \
            num_flags++;                                                      \
        }                                                                     \
    } while (0)
    ADD_FLAG(ctx->f_reorder, "REORDER ");
    ADD_FLAG(ctx->f_skip,    "SKIP ");
    ADD_FLAG(ctx->f_dup,     "DUP ");
    ADD_FLAG(ctx->f_snap_of, "SNAPOF ");
    ADD_FLAG(cfw_alloc_diag() & 1u, "ALLOC ");
    #undef ADD_FLAG
    if (num_flags == 0) strlcat(line, "OK ", sizeof(line));

    /* previous message's durations: whole worker, then just the present step */
    strlcat(line, "w", sizeof(line));
    u_to_dec(line, ctx->last_worker_us, sizeof(line));
    strlcat(line, "us p", sizeof(line));
    u_to_dec(line, ctx->last_present_us, sizeof(line));
    strlcat(line, "us", sizeof(line));

    uint32_t free_13 = heap_object_free(0x20000358u, 0x201350a8u, 0x000cd000u);
    uint32_t free_20 =
        *(volatile uint32_t *)0x20076e08u == 0x202020a8u
            ? tlsf_arena_free(0x202020a8u, 0x00070800u)
            : TLSF_FREE_INVALID;
    /* The stock 0x2000033c descriptor is initialized with 0x2d000 bytes. The
     * CFW patch reduces it to 0x2cc00, reserving the final 1 KiB for CFW state. */
    uint32_t free_27 = heap_object_free(0x2000033cu, 0x202728a8u, 0x0002cc00u);
    strlcat(line, " f13/20/27=", sizeof(line));
    append_free_kib(line, free_13, sizeof(line));
    strlcat(line, "/", sizeof(line));
    append_free_kib(line, free_20, sizeof(line));
    strlcat(line, "/", sizeof(line));
    append_free_kib(line, free_27, sizeof(line));
    strlcat(line, "k", sizeof(line));

    draw_string(disp, w, h, IMAGE_X + 2, IMAGE_Y + 2, line, 15, 0);
}
