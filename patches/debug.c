#include "debug.h"

/* --- glasses-side timing: Arm DWT cycle counter, clock derived by calibration -----
 * CYCCNT is a free-running core-cycle counter (~4 ns). To use it we (1) UNLOCK the DWT
 * via its CoreSight software-lock register (write 0xC5ACCE55 to LAR at DWT_base+0xFB0)
 * — without this, writes to DWT->CTRL are ignored and CYCCNT stays 0 (observed on hw);
 * then (2) set DEMCR.TRCENA and DWT->CTRL.CYCCNTENA. We re-assert all three cheaply per
 * measurement (only the idle SWO-trace block touches DEMCR).
 *
 * To convert cycles->us we need the core clock. The guessed global 0x200774e0 reads 0
 * on hardware (it's only written on a DVFS event, if ever), so instead we CALIBRATE:
 * measure how many CYCCNT cycles elapse across one edge of the firmware's 1 ms OS tick
 * (RAM 0x20077e4c, SysTick chain) — that IS cycles-per-ms. Cached in the ctx; a bounded
 * spin falls back to 250 MHz if the tick never advances. All divides are 32-bit
 * (hardware UDIV) — a 64-bit divide would emit an external __aeabi_uldivmod build.py
 * rejects. (Limitation: cached across DVFS; a clock switch makes the figure ~stale.) */
#define DWT_DEMCR   (*(volatile uint32_t *)0xE000EDFCU)  /* CoreDebug->DEMCR (TRCENA bit24) */
#define DWT_LAR     (*(volatile uint32_t *)0xE0001FB0U)  /* DWT CoreSight Lock Access Reg */
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000U)  /* DWT->CTRL (CYCCNTENA bit0) */
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004U)  /* DWT->CYCCNT (core cycles) */
#define FW_CORE_HZ  (*(volatile uint32_t *)0x200774e0U)  /* guessed core-clock global (reads 0 on hw) */
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

/* Timer-task writer; publish a single aligned average for the display task.
 * Divide each sample first so even ten UINT32_MAX samples cannot overflow.
 * Until the ring fills, average only the paints that have actually completed. */
static void cfw_record_timer_paint(customCfwContext *ctx, uint32_t us) {
    ctx->timer_paint_us[ctx->timer_paint_next]=us;
    ctx->timer_paint_next=(ctx->timer_paint_next+1u)%CFW_TIMER_PAINT_SAMPLES;
    if (ctx->timer_paint_count<CFW_TIMER_PAINT_SAMPLES) ctx->timer_paint_count++;
    uint32_t average=0, remainder=0, count=ctx->timer_paint_count;
    for (uint32_t i=0; i<count; i++) {
        average+=ctx->timer_paint_us[i]/count;
        remainder+=ctx->timer_paint_us[i]%count;
    }
    ctx->timer_paint_average_us=average+remainder/count;
}

/* Diagnostic: record whether the frames the worker processes arrive in order /
 * skipped / DUPLICATED (mode-3 frame ids). Sticky flags shown by cfw_draw_flags;
 * these flags describe the private sender's frame IDs. `has_fid`=0 for a mode-6
 * keyframe (no id; it rebaselines the next delta so keyframe gaps aren't "skips").
 * Duplicate IDs are diagnostic only; every accepted message is executed. */
static void cfw_diag(int has_fid, uint16_t fid) {
    customCfwContext *ctx = getCustomCfwContext();
    if (ctx == 0) return;
    ctx->diag_seen = 1;
    if (!has_fid) { ctx->fid_resync = 1; return; }  /* keyframe rebaselines next delta */

    /* duplicate: this fid is still in the recent ring -> flag */
    for (uint32_t i = 0; i < CFW_FID_RING; i++)
        if (ctx->recent_fids[i] == fid) { ctx->f_dup = 1; return; }

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

static void append_heap_kib(char *out, cfw_heap_stats stats, uint32_t maxlen) {
    append_free_kib(out, stats.free_bytes, maxlen);
    strlcat(out, "/", maxlen);
    append_free_kib(out, stats.max_alloc, maxlen);
}

/* Terminus 6x12 diagnostic overlay at the top-left of the packed framebuffer.
 * First line: sticky REORDER/SKIP/DUP/ALLOC flags and previous worker/present
 * durations and rolling last-ten timer-paint average (t10), in microseconds.
 * Second: last received message size and CRC.
 * Third: total free / maximum malloc request for each heap, in whole KiB.
 * Heap snapshots are approximate; failed validation displays ?/?. Suppressed
 * when diag_hide is set (mode 7). */
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
    ADD_FLAG(cfw_alloc_diag() & 1u, "ALLOC ");
    #undef ADD_FLAG
    if (num_flags == 0) strlcat(line, "OK ", sizeof(line));

    /* previous message's durations: whole worker, then just the present step */
    strlcat(line, "w", sizeof(line));
    u_to_dec(line, ctx->last_worker_us, sizeof(line));
    strlcat(line, "us p", sizeof(line));
    u_to_dec(line, ctx->last_present_us, sizeof(line));
    strlcat(line, "us t10 ", sizeof(line));
    u_to_dec(line, ctx->timer_paint_average_us, sizeof(line));
    strlcat(line, "us", sizeof(line));

    draw_string(disp, w, h, IMAGE_X + 2, IMAGE_Y + 2, line, 15, 0);

    /* The BLE task publishes both fields with one aligned 32-bit store. Keep
     * the probe on its own line so sticky flags cannot truncate it. */
    uint32_t probe = ctx->message_probe.snapshot;
    strlcpy(line, "rx ", sizeof(line));
    u_to_dec(line, probe & 0xffffu, sizeof(line));
    strlcat(line, " crc ", sizeof(line));
    char hex[5];
    for (unsigned i = 0; i < 4; ++i) {
        unsigned digit = (probe >> (28 - 4 * i)) & 15u;
        hex[i] = (char)(digit < 10 ? '0' + digit : 'A' + digit - 10);
    }
    hex[4] = 0;
    strlcat(line, hex, sizeof(line));
    draw_string(disp, w, h, IMAGE_X + 2, IMAGE_Y + 14, line, 15, 0);

    cfw_heap_stats heap_13 = heap_object_stats(0x20000358u, 0x2013519cu, 0x000cd000u);
    cfw_heap_stats heap_20 = {TLSF_FREE_INVALID, TLSF_FREE_INVALID};
    if (CFW_HEAP_READ32(0x20077ed0u) == 0x2020219cu)
        heap_20 = tlsf_arena_stats(0x2020219cu, 0x00070800u);
    /* Stock heap 27 is reduced from 0x2d000 to 0x2cc00, reserving the final
     * 1 KiB for CFW state. These are the LVGL, EvenHub, and other heaps. */
    cfw_heap_stats heap_27 = heap_object_stats(0x2000033cu, 0x2027299cu, 0x0002cc00u);
    strlcpy(line, "free/max KiB: LVGL ", sizeof(line));
    append_heap_kib(line, heap_13, sizeof(line));
    strlcat(line, " EvenHub ", sizeof(line));
    append_heap_kib(line, heap_20, sizeof(line));
    strlcat(line, " Other ", sizeof(line));
    append_heap_kib(line, heap_27, sizeof(line));
    draw_string(disp, w, h, IMAGE_X + 2, IMAGE_Y + 26, line, 15, 0);
}
