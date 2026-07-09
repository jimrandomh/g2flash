#include <stdint.h>

/*
 * zlib (DEFLATE) image support for the G2 CFW — multi-mode load wrapper.
 *
 * Replaces the one BMP-loader call FUN_0050164a(state, buf, len) in FUN_004ae69c.
 * Dispatch on the first byte of the reassembled image buffer:
 *
 *   'B' (0x42)  -> raw BMP: decode with our own fast 4bpp decoder (load_bmp_fast).
 *   1           -> [1][zlib]   4bpp BMP: inflate into the recon buffer's tail,
 *                              then decode with load_bmp_fast.
 *   2           -> [2][zlib]   8bpp full frame: inflate straight into the display
 *                              buffer (state+0x8, w*h, 1 byte/pixel), then push.
 *   3           -> [3][l/4][t/2][w/4][h/2][fid16][zlib]  bounding-box delta: composite a
 *                              tight-4bpp rectangle onto a persistent 4bpp shadow of
 *                              the last frame that WE own (customCfwContext.back, NOT
 *                              a firmware buffer — those get recycled), then rebuild
 *                              the WHOLE 8bpp display buffer from the shadow and push.
 *                              Full rebuild (not an in-place partial write) is required
 *                              for correctness — the display buffer is an async source
 *                              that isn't a reliable previous-frame base. Box origin/
 *                              size quantized (left/width *4, so even; top/height *2)
 *                              to one byte each. Needs a prior mode-6 keyframe to seed
 *                              the shadow; rejected if the box isn't wholly in bounds
 *                              or the shadow can't be allocated.
 *   4           -> [4][zlib]   8bpp stereo pair (left frame then right frame, w*h
 *                              each); each lens keeps only its half (FW_SIDE).
 *   5           -> [5][...]    play a UI sound on the arm buzzer (no display change).
 *                              The G2 "speaker" is a PWM piezo buzzer — it can only
 *                              emit square-wave tones, not PCM/WAV — so this drives
 *                              the firmware's own buzzer driver instead of streaming
 *                              samples. Sub-dispatch on src[1]:
 *                                0 [0][type]            -> DRV_BuzzerPlayAfterQueue:
 *                                     play preset voice `type` (0..8) from the flash
 *                                     preset table (single beep / alarm / ringtone).
 *                                1 [1][note][oct][beat] -> DRV_BuzzerPlayNote: one
 *                                     tone. note 1..7, oct 0..3 (freq from the 28-
 *                                     entry note table), beat = duration in ~62ms
 *                                     units. Good for click/beep on tap/notification.
 *                                2 [2]                  -> stop/silence the buzzer.
 *                                3 [3][freqLo][freqHi][duty][msLo][msHi] -> raw tone:
 *                                     program the PWM to an ARBITRARY frequency
 *                                     (1..20000 Hz, 16-bit LE) at `duty` percent
 *                                     (0..100) for `ms` milliseconds (16-bit LE).
 *                                     Bypasses the 7-note x 4-octave lookup table
 *                                     entirely (that table is just a convenience);
 *                                     the hardware timer takes any Hz. Auto-stops
 *                                     by arming the buzzer's own osTimer with a
 *                                     null note list so the driver's timer callback
 *                                     shuts the PWM off after `ms`. Enables fine /
 *                                     microtonal pitch, chirps and pitch sweeps
 *                                     (send a run of these), and sub-62ms durations.
 *                              The preset/note/stop entries are self-contained fw
 *                              entries that queue into the buzzer's osTimer; the
 *                              raw-tone entry drives the low-level PWM start and
 *                              arms that same osTimer for auto-stop. None spin or
 *                              block here. Returns 0 (success).
 *   6           -> [6][zlib]   headerless 4bpp full frame: inflate the tightly
 *                              packed 4bpp pixels (h * ceil(w/2) bytes, top-down)
 *                              into the persistent CFW shadow (seeding it for mode-3
 *                              deltas), then expand into the 8bpp display buffer, push.
 *   7           -> [7][sub]    diagnostic control (no display change): 0 clears the
 *                              overlay flags, 1 hides the overlay, 2 shows it.
 *   8           -> [8][count][len16][submsg]...  multi-segment: apply each sub-message
 *                              to the shadow with the panel push DEFERRED, then present
 *                              once — an atomic multi-op update (e.g. scroll = rect-copy
 *                              + delta). Bounded to an uncompressed 4bpp BMP's size; no
 *                              nesting. Intended for shadow ops (modes 3/6/9).
 *   9           -> [9][srcrect][dstrect]  rect-copy inside the 4bpp shadow (full uint16
 *                              L/T/W/H each; same size; may overlap), then present.
 *                              Pairs with a delta (usually via mode 8) to scroll.
 *   anything else / too short  -> load_bmp_fast (rejects cleanly if not a BMP).
 *
 * The HIGH BIT of the mode byte is a "lenses differ" flag; most modes ignore it. For
 * mode 3 it carries two boxes (left then right, same size) sharing one zlib payload —
 * a stereo shift without duplicating pixels; each lens draws at its own box. For mode 9
 * it carries two rect-sets (left then right); each lens uses its own.
 *
 * The image dimensions come from the container state (state+0x40/0x42), so no
 * header is needed on 8bpp payloads — the sender just deflates w*h raw bytes
 * (mode 2); mode 3 carries its own 4-byte box header, and mode 6 tight 4bpp.
 *
 * Every invocation (any mode) first kicks the EvenHub keepalive: stock firmware
 * resets the ticks-since-heartbeat counter only on the sid-0x0c heartbeat msg, so
 * a client streaming image updates to maximize throughput would otherwise trip the
 * "Connection lost" teardown. See FW_KEEPALIVE_RESET at the top of load_image_z.
 *
 * BMP handling (modes 'B' and 1) does NOT use the stock loader FUN_0050164a: that
 * decoder runs two non-inlined function calls PER PIXEL (palette pack + luminance
 * blend) plus a whole-buffer CRC, which costs more CPU than the airtime it saves.
 * load_bmp_fast instead does a direct 4bpp-nibble -> 8bpp (nibble*17) expand,
 * ignoring the palette (the sender only ever uses the standard gray ramp). The
 * stock loader is kept only as a fallback for non-4bpp or mismatched-size BMPs.
 *
 * Modes 2/3/4/6 and the BMP decoder replicate the stock loader's tail to get the
 * buffer onto the panel: dcache-clean the display buffer, set the LVGL image
 * descriptor (state+0x24..0x34), lv_image_set_src, lv_obj_invalidate.
 *
 * Self-contained: no external symbols, no writable globals. Firmware entry points
 * are called by absolute constant address (movw/movt + blx, no relocation). The
 * only placement-dependent constants are ZWRAP_ALLOC_ADDR / ZWRAP_FREE_ADDR,
 * filled in on the 2nd build pass (zwrap_alloc/zwrap_free are emitted first so
 * their offsets, hence addresses, are stable).
 */

typedef void *(*malloc_fn)(uint32_t);
typedef void (*free_fn)(void *);
typedef int (*inflateInit2_fn)(void *strm, int windowBits, const char *ver, int ssize);
typedef int (*inflate_fn)(void *strm, int flush);
typedef int (*inflateEnd_fn)(void *strm);
typedef int (*loadbmp_fn)(void *state, void *bmp, uint32_t len);
typedef void (*cacheflush_fn)(void *desc);          /* desc = uint32[2]{ptr,size} */
typedef void (*lv_set_src_fn)(uint32_t obj, void *desc);
typedef void (*lv_invalidate_fn)(uint32_t obj);
typedef uint32_t (*lens_side_fn)(void);             /* 2 = LEFT lens, 1 = RIGHT lens */
typedef void (*buzz_preset_fn)(uint32_t type);      /* DRV_BuzzerPlayAfterQueue */
typedef void (*buzz_note_fn)(uint32_t note, uint32_t tone, uint32_t beat); /* DRV_BuzzerPlayNote */
typedef void (*buzz_reset_fn)(void);                /* buzzer stop/reset */
typedef void (*buzz_raw_fn)(uint32_t freq, uint32_t duty);   /* reset+power+PWM(freq,duty) */
typedef int  (*timer_start_fn)(uint32_t handle, uint32_t ms); /* osTimer start (one-shot) */
typedef uint32_t (*timer_new_fn)(void *cb, uint32_t type, void *arg, void *attr); /* osTimerNew-> handle */
typedef int  (*timer_stop_fn)(uint32_t handle);     /* osTimer stop */
typedef void (*keepalive_reset_fn)(void);           /* zero the EvenHub keepalive counter */
typedef uint8_t *(*lookup_fn)(uint32_t container_id); /* container id -> spec-list node (or 0) */
typedef int  (*complete_emit_fn)(uint32_t id, void *hdr, int kind, uint32_t p4); /* completion emit */

/* firmware entry points (Thumb bit set for blx via constant pointer) */
#define FW_MALLOC  ((malloc_fn)0x00472b6fU)         /* FUN_00472b6e malloc(size) */
#define FW_FREE    ((free_fn)0x00472bb3U)           /* FUN_00472bb2 free(ptr) */
#define FW_INIT2   ((inflateInit2_fn)0x005be643U)   /* FUN_005be642 inflateInit2_ */
#define FW_INFLATE ((inflate_fn)0x005be711U)        /* FUN_005be710 inflate */
#define FW_END     ((inflateEnd_fn)0x005be607U)     /* FUN_005be606 inflateEnd */
#define FW_LOADBMP ((loadbmp_fn)0x0050164bU)        /* FUN_0050164a BMP decoder */
#define FW_FLUSH   ((cacheflush_fn)0x00472f87U)     /* FUN_00472f86 dcache clean range */
#define FW_SETSRC  ((lv_set_src_fn)0x004b0f01U)     /* FUN_004b0f00 lv_image_set_src */
#define FW_INVAL   ((lv_invalidate_fn)0x004405f7U)  /* FUN_004405f6 lv_obj_invalidate */
#define FW_SIDE    ((lens_side_fn)0x0045a8edU)       /* FUN_0045a8ec -> 2=left, 1=right */
#define FW_BUZZ_PRESET ((buzz_preset_fn)0x004e97efU) /* FUN_004e97ee DRV_BuzzerPlayAfterQueue(type 0..8) */
#define FW_BUZZ_NOTE   ((buzz_note_fn)0x004e988dU)   /* FUN_004e988c DRV_BuzzerPlayNote(note,tone,beat) */
#define FW_BUZZ_RESET  ((buzz_reset_fn)0x004e9759U)  /* FUN_004e9758 buzzer stop/reset */
#define FW_BUZZ_RAW    ((buzz_raw_fn)0x004e991dU)     /* FUN_004e991c reset+power+PWM(freq,duty) */
#define FW_TIMER_START ((timer_start_fn)0x004484ebU)  /* FUN_004484ea osTimerStart(handle,ms) */
#define FW_TIMER_NEW   ((timer_new_fn)0x00448403U)    /* FUN_00448402 osTimerNew(cb,type,arg,attr) */
#define FW_TIMER_STOP  ((timer_stop_fn)0x0044852bU)   /* FUN_0044852a osTimerStop(handle) */
#define FW_KEEPALIVE_RESET ((keepalive_reset_fn)0x00505cc3U) /* FUN_00505cc2: EvenHub keepalive
                                                     * counter (@0x200744e8) = 0. This is the exact
                                                     * leaf the stock sid-0x0c heartbeat handler in
                                                     * FUN_004ae69c calls; it takes no args and reads
                                                     * the counter pointer from its own literal pool. */
#define FW_LOOKUP        ((lookup_fn)0x00505cd7U)    /* FUN_00505cd6(id) -> spec node; state=*(node+0x10) */
#define FW_COMPLETE_EMIT ((complete_emit_fn)0x004ff44bU) /* FUN_004ff44a: stock image-complete emitter */
#define BUZZ_TIMER_ADDR 0x20074440U                   /* RAM: buzzer osTimer handle (DAT_004e996c) */
#define ZLIB_VER   ((const char *)0x007885e4U)      /* "1.1.4" */

#ifndef ZWRAP_ALLOC_ADDR
#define ZWRAP_ALLOC_ADDR 0u   /* pass-2 placeholders */
#define ZWRAP_FREE_ADDR  0u
#endif

/* seq_tick's own absolute address, baked in on the 2nd build pass (same reason as
 * ZWRAP_*: we hand it to osTimerNew as a fn-ptr value, which can't be a relocation
 * in position-independent .text). seq_tick is defined early (right after zwrap_*)
 * so its offset is identical across both passes. */
#ifndef SEQ_TICK_ADDR
#define SEQ_TICK_ADDR 0u
#endif

/* z_stream (zlib 1.1.4, sizeof = 0x38) field offsets */
#define ZS_NEXT_IN   0x00
#define ZS_AVAIL_IN  0x04
#define ZS_NEXT_OUT  0x0c
#define ZS_AVAIL_OUT 0x10
#define ZS_TOTAL_OUT 0x14
#define ZS_ZALLOC    0x20
#define ZS_ZFREE     0x24
#define ZS_OPAQUE    0x28
#define ZS_SIZE      0x38

#define XOR_CHUNK 256   /* mode-4 inflate scratch (stack); any size works */

/* Persistent CFW-owned state that must survive image-container teardown/rebuild.
 * The image container (its display buffer A @ state+0x8 and recon buffer B @
 * state+0xc) is freed and reallocated constantly, and the allocator recycles the
 * two freed blocks, so NO firmware container buffer is a reliable "previous
 * frame" for a mode-3 delta. We therefore keep our own back buffer (a 4bpp shadow
 * of the last frame), and to survive rebuilds we anchor a pointer to this struct
 * in a spare word of the BLE-RX task context (ble_msgrx, base = *0x004a069c ->
 * 0x20003fdc; only its +0x8/+0xc are used by the firmware, and it is never freed).
 * We only ever touch that word from inside our image handler, i.e. only after a
 * custom-firmware image message has arrived — so if the word is not actually free
 * the damage is confined to CFW use and a reboot recovers. The struct's `magic`
 * guards against warm-reset garbage; the slot ptr is range-checked before deref. */
#define CFW_FID_RING  16     /* recent mode-3 frame ids kept for duplicate detection */
#define CFW_SNAP_RING 12     /* in-flight compressed-message snapshots (per producer race depth) */
#define CFW_SEQ_MAX   48     /* max steps in a buzzer tone sequence (mode-5 kind 4) */

/* One snapshotted compressed image message. Taken at reconstruction-complete (both
 * lenses), consumed FIFO in the deferred handler. Keyed by the owning image-state
 * pointer so multiple containers (e.g. faceclaw's 4 tiles) don't cross-feed. */
typedef struct {
    uint8_t *state;      /* owning image-state (key); 0 = empty slot */
    uint8_t *buf;        /* malloc'd copy of the reassembled message */
    uint32_t len;
    uint32_t seq;        /* push order, for per-state FIFO */
} cfw_snap;

typedef struct {
    uint32_t magic;      /* CFW_CTX_MAGIC when valid */
    /* --- snapshot FIFO: fixes the producer/consumer race on the shared recon buffer.
     * snapshot_side() copies each completed message here (both lenses); image_deferred
     * drains this lens's pending snapshots and runs the worker on each, ignoring the
     * live (possibly-overwritten) recon buffer B. (The 4bpp shadow of the last frame,
     * needed by mode-3 deltas, lives in each container's own recon-buffer tail — see
     * cfw_back_buffer — so it's per-container and costs no extra RAM.) */
    cfw_snap snaps[CFW_SNAP_RING];
    uint32_t snap_seq;   /* next push sequence number */
    /* --- diagnostics, overlaid as gray boxes (verify the fix; should stay clear). Mode
     * 7 clears the flags / toggles the overlay visibility (diag_hide). --- */
    uint16_t last_fid;   /* last frame id seen (mode-3 messages) */
    uint16_t high_fid;   /* highest frame id seen */
    uint8_t  diag_seen;  /* recorded at least one frame yet */
    uint8_t  fid_resync; /* keyframe rebaselines the next delta's fid (no false skip) */
    uint8_t  diag_hide;  /* 1 = don't draw the flag overlay (default 0 = visible) */
    uint8_t  f_reorder;  /* FLAG: ever saw a frame id go backward */
    uint8_t  f_skip;     /* FLAG: ever saw a frame id gap (skipped) */
    uint8_t  f_dup;      /* FLAG: ever saw a duplicate frame id (in the recent ring) */
    uint8_t  f_snap_of;  /* FLAG: snapshot ring overflowed (dropped an in-flight frame) */
    uint16_t recent_fids[CFW_FID_RING]; /* ring of the last N mode-3 frame ids seen */
    uint8_t  recent_pos; /* next write index into recent_fids */
    /* --- buzzer tone sequencer (mode-5 kind 4). Plays a list of (freq,duty,ms)
     * steps back-to-back on OUR OWN one-shot osTimer — the firmware buzzer timer's
     * callback is the fixed note-walker, which can't emit arbitrary frequencies.
     * State lives in this singleton so it survives the handler return and is
     * reachable from seq_tick (the timer callback, in the RTOS timer thread). --- */
    uint32_t seq_timer;                   /* our osTimer handle; created lazily, reused, never freed */
    uint8_t  seq_count;                   /* steps in the current sequence (0 = idle) */
    uint8_t  seq_cursor;                  /* index of the next step to play */
    uint8_t  seq_steps[CFW_SEQ_MAX * 5];  /* freqLo,freqHi,duty,msLo,msHi per step */
} customCfwContext;

#define CFW_CTX_SLOT  0x20003fdcU    /* ble_msgrx context +0x0 (spare, never freed) */
#define CFW_CTX_MAGIC 0xC0FFEE5AU    /* distinctive; ~0 chance of matching garbage */

void *zwrap_alloc(void *opaque, uint32_t items, uint32_t size) {
    (void)opaque;
    return FW_MALLOC(items * size);
}

void zwrap_free(void *opaque, void *ptr) {
    (void)opaque;
    FW_FREE(ptr);
}

/* Buzzer tone-sequence timer callback (mode-5 kind 4). Plays seq_steps[cursor],
 * advances the cursor, and re-arms this timer for that step's ms; after the final
 * step's ms elapses it powers the PWM off and goes idle. `arg` is the singleton
 * context (passed as the osTimer argument at creation). Runs in the RTOS timer
 * thread — the only shared state is the singleton, guarded by magic + bounds.
 * Non-static (external linkage) and defined early so -O2 keeps it and its .text
 * offset is identical across both build passes; its absolute address is baked in
 * via SEQ_TICK_ADDR on pass 2 (osTimerNew needs it as a fn-ptr value). */
void seq_tick(void *arg) {
    customCfwContext *ctx = (customCfwContext *)arg;
    if (ctx == 0 || ctx->magic != CFW_CTX_MAGIC) return;
    uint32_t c = ctx->seq_cursor;
    if (c >= ctx->seq_count) {           /* final step's ms elapsed -> sequence done */
        ctx->seq_count = 0;
        FW_BUZZ_RESET();                 /* PWM off */
        return;
    }
    const uint8_t *s = &ctx->seq_steps[c * 5];
    uint32_t freq = (uint32_t)s[0] | ((uint32_t)s[1] << 8);
    uint32_t duty = s[2];
    uint32_t ms   = (uint32_t)s[3] | ((uint32_t)s[4] << 8);
    if (freq < 1) freq = 1;
    if (freq > 20000) freq = 20000;
    if (duty > 100) duty = 100;
    if (ms < 1) ms = 1;
    ctx->seq_cursor = (uint8_t)(c + 1);
    if (duty == 0) FW_BUZZ_RESET();      /* duty 0 = rest: silent for ms */
    else FW_BUZZ_RAW(freq, duty);        /* start this tone */
    if (ctx->seq_timer) FW_TIMER_START(ctx->seq_timer, ms);
}

static void push_display(uint8_t *state, uint8_t *disp, uint32_t w, uint32_t h);
static void unpack4bpp(uint8_t *dst, uint32_t dst_stride, const uint8_t *pix,
                       uint32_t w, uint32_t h, uint32_t src_stride, int bottom_up);
static int load_bmp_fast(uint8_t *state, const uint8_t *bmp, uint32_t len);
static customCfwContext *getCustomCfwContext(void);
static uint8_t *cfw_back_buffer(uint8_t *state, uint32_t w, uint32_t h);
static void bzero(uint8_t *buf, uint32_t len);
static int cfw_diag(int has_fid, uint16_t fid);
static void cfw_draw_flags(uint8_t *disp, uint32_t w, uint32_t h);
static uint32_t rd16(const uint8_t *p);
static void present_shadow(uint8_t *state, uint32_t w, uint32_t h);
static void rect_copy_4bpp(uint8_t *buf, uint32_t stride, uint32_t sL, uint32_t sT,
                           uint32_t dL, uint32_t dT, uint32_t bw, uint32_t bh);
static int image_dispatch(uint8_t *state, const uint8_t *src, uint32_t srclen, int present);

/* The image worker: static, called from image_deferred (the deferred consumer, which
 * runs on BOTH lenses via the cross-lens-synchronized completion message). NOTE: image
 * handling lives here / in the deferred path on purpose — the sync-completion path
 * (image_complete) runs on only the RECEIVING lens, so doing the work there leaves the
 * other lens blank. image_worker kicks the keepalive once per top-level message, then
 * defers to image_dispatch (which recurses for multi-segment messages). */
static int image_worker(void *state_, uint8_t *src, uint32_t srclen) {
    /* An inbound image message proves the phone is still connected, so kick the
     * EvenHub keepalive back to life exactly as the stock heartbeat handler does.
     * Stock firmware resets the ticks-since-last-heartbeat counter (@0x200744e8)
     * ONLY on the sid-0x0c heartbeat message; the periodic evenhub_ui_event_handler
     * (FUN_00506460, param_1==4) increments it every tick and, once it passes 899,
     * fires FUN_004fee62(0,0) -> "DISPLAY_AUTO_REFLASH heartbeat timeout" -> the
     * "Connection lost" context teardown. A client streaming image updates to
     * maximize throughput would otherwise have to interleave heartbeats to avoid
     * that teardown; resetting here lets a steady image stream keep the context
     * alive on its own. Placed before any dispatch so every mode (image, delta,
     * stereo, sound, BMP) counts as liveness. Runs on the same evenhub task that
     * owns the counter, so no locking is needed. */
    FW_KEEPALIVE_RESET();
    return image_dispatch((uint8_t *)state_, src, srclen, 1);
}

/* Dispatch one message. `present`=1 means push the result to the panel now; a
 * multi-segment message (mode 8) dispatches each sub-message with present=0 (so they
 * only mutate the shadow) and then presents once, giving an atomic multi-op update
 * (e.g. scroll = rect-copy + delta). The high bit of the mode byte is the "lenses
 * differ" flag; most modes ignore it. */
static int image_dispatch(uint8_t *state, const uint8_t *src, uint32_t srclen, int present) {
    if (src == 0 || srclen < 1) return load_bmp_fast(state, src, srclen);

    int lenses_differ = src[0] & 0x80;             /* high bit: per-lens variant */
    uint8_t mode = src[0] & 0x7f;
    if (mode == 0x42) return load_bmp_fast(state, src, srclen);       /* raw BMP */

    if (mode == 5) {
        /* play a UI sound on the buzzer; no display change. [5][kind][args...].
         * kinds 0-3 use firmware entry points that copy their args into fw-owned
         * storage (preset table is flash; PlayNote copies into an 8-byte scratch;
         * raw uses a one-shot on the buzzer's own timer), so the recon buffer is
         * free to be reused immediately. kind 4 (tone sequence) steps through our
         * own osTimer whose callback (seq_tick) reads the sequence out of the
         * persistent CFW context. */
        uint8_t kind = (srclen >= 2) ? src[1] : 0xffu;
        customCfwContext *ctx = getCustomCfwContext();

        /* Any new sound supersedes an in-flight tone sequence — otherwise seq_tick
         * would keep reprogramming the PWM underneath it. Stop our sequencer first
         * (same handler thread; mirrors the firmware's stop-before-restart order). */
        if (ctx && ctx->seq_count) {
            if (ctx->seq_timer) FW_TIMER_STOP(ctx->seq_timer);
            ctx->seq_count = 0;
        }

        if (kind == 0 && srclen >= 3) {                 /* preset 0..8 */
            if (src[2] <= 8) FW_BUZZ_PRESET(src[2]);
        } else if (kind == 1 && srclen >= 5) {          /* single tone */
            uint8_t note = src[2], oct = src[3], beat = src[4];
            /* note 1..7 x oct 0..3 keeps the freq-table index in [0,27] so the
             * driver's `1000000 / (0xffff - table[idx])` can never divide by 0 */
            if (note >= 1 && note <= 7 && oct <= 3 && beat != 0)
                FW_BUZZ_NOTE(note, oct, beat);
        } else if (kind == 2) {                         /* stop / silence */
            FW_BUZZ_RESET();
        } else if (kind == 3 && srclen >= 7) {          /* raw tone: freq/duty/ms */
            uint32_t freq = (uint32_t)src[2] | ((uint32_t)src[3] << 8);
            uint32_t duty = src[4];
            uint32_t ms   = (uint32_t)src[5] | ((uint32_t)src[6] << 8);
            if (freq < 1) freq = 1;                     /* freq 0 -> bad PWM period */
            if (freq > 20000) freq = 20000;             /* hw range per AT^BUZZER */
            if (duty > 100) duty = 100;                 /* duty is a 0..100 percent */
            if (ms < 1) ms = 1;
            FW_BUZZ_RAW(freq, duty);                    /* reset+power+PWM; note list now null */
            uint32_t h = *(volatile uint32_t *)BUZZ_TIMER_ADDR;
            if (h) FW_TIMER_START(h, ms);               /* callback stops PWM after ms */
        } else if (kind == 4 && srclen >= 3 && ctx) {   /* tone sequence */
            /* [4][nSteps][ (freqLo,freqHi,duty,msLo,msHi) x nSteps ]. Copy the steps
             * into the persistent context, create our one-shot osTimer once (arg =
             * ctx, so seq_tick can find the state), and kick it — seq_tick plays
             * step 0 and chains the rest, auto-stopping after the last step's ms. */
            uint32_t avail = (srclen - 3) / 5;
            uint32_t n = src[2];
            if (n > avail) n = avail;
            if (n > CFW_SEQ_MAX) n = CFW_SEQ_MAX;
            for (uint32_t i = 0; i < n * 5; i++) ctx->seq_steps[i] = src[3 + i];
            ctx->seq_count = (uint8_t)n;
            ctx->seq_cursor = 0;
            if (n) {
                if (ctx->seq_timer == 0)
                    ctx->seq_timer = FW_TIMER_NEW((void *)SEQ_TICK_ADDR, 0, ctx, 0);
                if (ctx->seq_timer) FW_TIMER_START(ctx->seq_timer, 1); /* kick: seq_tick runs step 0 */
                else { ctx->seq_count = 0; FW_BUZZ_RESET(); }          /* timer create failed */
            }
        }
        return 0;
    }

    if (mode == 7) {
        /* Diagnostic control (no display change). [7][sub]:
         *   0 -> clear the sticky flags and frame-order tracking (use between tests)
         *   1 -> hide the flag overlay      2 -> show the flag overlay
         * Runs on both lenses via the normal snapshot/deferred path, so it clears/toggles
         * both eyes. */
        customCfwContext *ctx = getCustomCfwContext();
        uint8_t sub = (srclen >= 2) ? src[1] : 0xffu;
        if (ctx) {
            if (sub == 0) {
                ctx->f_reorder = ctx->f_skip = ctx->f_dup = ctx->f_snap_of = 0;
                ctx->diag_seen = ctx->fid_resync = 0;
                ctx->last_fid = ctx->high_fid = 0;
                for (uint32_t i = 0; i < CFW_FID_RING; i++) ctx->recent_fids[i] = 0xffff;
                ctx->recent_pos = 0;
            } else if (sub == 1) {
                ctx->diag_hide = 1;
            } else if (sub == 2) {
                ctx->diag_hide = 0;
            }
        }
        return 0;
    }

    /* geometry: needed by the drawing modes (8/9 and the strm modes below) */
    uint32_t w = *(uint16_t *)(state + 0x40);
    uint32_t h = *(uint16_t *)(state + 0x42);
    uint32_t wh = w * h;

    if (mode == 8) {
        /* Multi-segment: [8][count][len16][submsg]... — dispatch each sub with
         * present=0 (mutate the shadow only), then present once, giving an atomic
         * multi-op update (e.g. scroll = rect-copy + delta, no intermediate flash).
         * Sized no larger than an uncompressed 4bpp BMP for this container; no nesting
         * (a sub-message may not itself be a multi-segment message). Intended for
         * shadow ops (modes 3/6/9); a direct-to-A sub would be clobbered by the final
         * present. */
        if (!present) return -1;                       /* only valid at top level */
        if (srclen < 2) return -1;
        uint32_t bmp_max = 118 + ((((w + 1) >> 1) + 3) & ~3u) * h;
        if (srclen > bmp_max) return -1;
        uint32_t count = src[1];
        uint32_t pos = 2;
        for (uint32_t i = 0; i < count; i++) {
            if (pos + 2 > srclen) return -1;
            uint32_t seglen = rd16(src + pos);
            pos += 2;
            if (seglen < 1 || pos + seglen > srclen) return -1;
            image_dispatch(state, src + pos, seglen, 0);   /* shadow-only, no present */
            pos += seglen;
        }
        present_shadow(state, w, h);                   /* one atomic present */
        return 0;
    }

    if (mode == 9) {
        /* Rect-copy within the 4bpp shadow: move a block from a source rect to a
         * destination rect (full uint16 coords; the rects may overlap). Both rects must
         * be the same size and wholly in bounds. With the lenses-differ flag there are
         * two rect-sets (left then right) and each lens uses its own. rect_copy_4bpp
         * takes a whole-byte fast path when left/width are even, else a nibble path.
         * Pairs with a follow-up delta (usually in one mode-8 message) to scroll. */
        const uint8_t *r = src + 1;
        uint32_t need = lenses_differ ? 32u : 16u;     /* 8 bytes per rect, 2 or 4 rects */
        if (srclen < 1 + need) return -1;
        if (lenses_differ && FW_SIDE() != 2) r += 16;  /* right lens uses the 2nd set */
        uint32_t sL = rd16(r),     sT = rd16(r + 2),  sW = rd16(r + 4),  sH = rd16(r + 6);
        uint32_t dL = rd16(r + 8), dT = rd16(r + 10), dW = rd16(r + 12), dH = rd16(r + 14);
        if (sW == 0 || sH == 0 || sW != dW || sH != dH) return -1;    /* copy = same size */
        if (sL + sW > w || sT + sH > h || dL + dW > w || dT + dH > h) return -1;  /* bounds */
        uint8_t *shadow = cfw_back_buffer(state, w, h);
        if (shadow == 0) return -1;
        rect_copy_4bpp(shadow, (w + 1) >> 1, sL, sT, dL, dT, sW, sH);
        if (present) present_shadow(state, w, h);
        return 0;
    }

    if (mode < 1 || mode > 6 || srclen < 3) return load_bmp_fast(state, src, srclen);

    const uint8_t *zsrc = src + 1;
    uint32_t zlen = srclen - 1;

    uint8_t strm[ZS_SIZE];
    for (uint32_t i = 0; i < ZS_SIZE; i++) strm[i] = 0;
    *(const uint8_t **)(strm + ZS_NEXT_IN) = zsrc;
    *(uint32_t *)(strm + ZS_AVAIL_IN) = zlen;
    *(uint32_t *)(strm + ZS_ZALLOC) = ZWRAP_ALLOC_ADDR;
    *(uint32_t *)(strm + ZS_ZFREE) = ZWRAP_FREE_ADDR;
    *(uint32_t *)(strm + ZS_OPAQUE) = 0;

    if (mode == 1) {
        /* 4bpp BMP: inflate the whole BMP into a scratch buffer, then decode with
         * load_bmp_fast. src is a snapshot (message-sized), not the recon buffer, so
         * we can't scratch in its tail — malloc it (mode 1 isn't the fast path). */
        uint32_t row_stride = (((w + 1) >> 1) + 3) & ~3u;        /* BMP 4-byte-padded */
        uint32_t out_max = 118 + row_stride * h + 64;
        uint8_t *dst = (uint8_t *)FW_MALLOC(out_max);
        if (dst == 0) return -1;
        *(uint8_t **)(strm + ZS_NEXT_OUT) = dst;
        *(uint32_t *)(strm + ZS_AVAIL_OUT) = out_max;
        int ret = -1;
        if (FW_INIT2(strm, 15, ZLIB_VER, ZS_SIZE) == 0) {
            int r = FW_INFLATE(strm, 4);              /* Z_FINISH (whole BMP fits) */
            uint32_t out = *(uint32_t *)(strm + ZS_TOTAL_OUT);
            FW_END(strm);
            if (r == 1) ret = load_bmp_fast(state, dst, out);
        } else {
            FW_END(strm);
        }
        FW_FREE(dst);
        return ret;
    }

    if (mode == 6) {
        /* Full headerless 4bpp frame. Inflate it into the persistent shadow (this
         * container's recon-buffer tail) that mode-3 deltas composite onto, so a
         * mode-6 keyframe seeds a stable base, then present (unless deferred by a
         * multi-segment wrapper). */
        cfw_diag(0, 0);                               /* keyframe: rebaseline delta fid */
        uint32_t out_max = ((w + 1) >> 1) * h;                   /* tight 4bpp */
        uint8_t *dst = cfw_back_buffer(state, w, h);
        if (dst == 0) return -1;                      /* no recon buffer -> can't proceed */
        *(uint8_t **)(strm + ZS_NEXT_OUT) = dst;
        *(uint32_t *)(strm + ZS_AVAIL_OUT) = out_max;
        int ok = 0;
        if (FW_INIT2(strm, 15, ZLIB_VER, ZS_SIZE) == 0) {
            int r = FW_INFLATE(strm, 4);              /* Z_FINISH (whole frame fits) */
            ok = (r == 1 && *(uint32_t *)(strm + ZS_TOTAL_OUT) == out_max);
            FW_END(strm);
        } else {
            FW_END(strm);
        }
        if (!ok) return -1;
        if (present) present_shadow(state, w, h);
        return 0;
    }

    if (mode == 3) {
        /* Bounding-box delta, composited onto a PERSISTENT 4bpp shadow of the last
         * frame kept in this container's recon-buffer tail (see cfw_back_buffer),
         * then the WHOLE 8bpp display buffer is rebuilt from the shadow and pushed.
         *
         * The stale-base race that used to force a CFW-owned shadow is now fixed at the
         * source: the worker runs on a per-frame SNAPSHOT (drained in order by
         * image_deferred), not the live recon buffer, so successive deltas compose onto
         * the shadow in the right order. The shadow itself is stable in B's tail (B lives
         * for the streaming session and its head only holds a small message).
         *
         * Rebuilding the entire frame (not an in-place partial write to A) is still
         * required: A is an async LVGL source, so a full write is what lands correctly
         * regardless of the render/scan-out pipeline.
         *
         *   [3][left/4][top/2][width/4][height/2][fid_lo][fid_hi][zlib box pixels]
         * left/width are *4 (=> multiples of 4 => even) so left>>1 and bw>>1 are whole
         * byte offsets: each box row inflates straight into its place in the 4bpp
         * shadow as a plain byte run, no nibble shifting. fid is a uint16 per-frame
         * counter (diagnostics). Rejected (old frame kept) if the box isn't wholly in
         * bounds. The sender must have sent a mode-6 keyframe before/among deltas.
         *
         * lenses-differ variant: [3|80][Lbox 4][Rbox 4][fid 2][shared zlib]. Both boxes
         * must be the same size; each lens draws the SAME decompressed pixels at its own
         * box — a stereo shift (e.g. a raised dialog) with the pixel data sent once. */
        uint32_t box_off, fid_off, z_off;
        if (lenses_differ) {
            if (srclen < 12) return -1;               /* mode + 2 boxes + fid + some zlib */
            if (src[3] != src[7] || src[4] != src[8]) return -1;   /* boxes must match size */
            box_off = (FW_SIDE() == 2) ? 1 : 5;       /* left set / right set */
            fid_off = 9;
            z_off   = 11;
        } else {
            if (srclen < 8) return -1;                /* 4 box hdr + 2 fid + some zlib */
            box_off = 1;
            fid_off = 5;
            z_off   = 7;
        }
        uint32_t left = (uint32_t)src[box_off]     * 4;
        uint32_t top  = (uint32_t)src[box_off + 1] * 2;
        uint32_t bw   = (uint32_t)src[box_off + 2] * 4;
        uint32_t bh   = (uint32_t)src[box_off + 3] * 2;
        uint16_t fid  = (uint16_t)rd16(src + fid_off);
        if (bw == 0 || bh == 0 || left + bw > w || top + bh > h) return -1;

        /* Duplicate frame id (re-processed message) -> skip: re-applying a delta
         * out of order corrupts the shadow. Leaves the current frame on screen. */
        if (cfw_diag(1, fid)) return 0;               /* dup fid -> skip (records order/skip) */

        uint32_t sstride = (w + 1) >> 1;              /* 4bpp shadow row stride */
        uint8_t *shadow = cfw_back_buffer(state, w, h);  /* persistent last frame (recon tail) */
        if (shadow == 0) return -1;                   /* no stable base -> keyframe resyncs */
        uint32_t rowbytes = bw >> 1;                  /* whole bytes (bw even) */

        *(const uint8_t **)(strm + ZS_NEXT_IN) = src + z_off;   /* zlib past box(es) + fid */
        *(uint32_t *)(strm + ZS_AVAIL_IN) = srclen - z_off;
        int ok = 0;
        if (FW_INIT2(strm, 15, ZLIB_VER, ZS_SIZE) == 0) {
            ok = 1;
            for (uint32_t y = 0; y < bh; y++) {
                /* inflate this box row straight into its slot in the shadow */
                *(uint8_t **)(strm + ZS_NEXT_OUT) = shadow + (top + y) * sstride + (left >> 1);
                *(uint32_t *)(strm + ZS_AVAIL_OUT) = rowbytes;
                int r = FW_INFLATE(strm, 0);          /* Z_NO_FLUSH */
                if (*(uint32_t *)(strm + ZS_AVAIL_OUT) != 0) { ok = 0; break; }  /* short row */
                if (r == 1) { ok = (y + 1 == bh); break; }  /* stream end: only ok on last row */
                if (r != 0) { ok = 0; break; }              /* inflate error */
            }
            FW_END(strm);
        } else {
            FW_END(strm);
        }
        if (!ok) return -1;                           /* leave the old frame on screen */

        if (present) present_shadow(state, w, h);     /* rebuild full 8bpp frame + push */
        return 0;
    }

    /* modes 2 & 4: 8bpp straight into the display buffer (separate from recon) */
    uint8_t *disp = *(uint8_t **)(state + 0x8);
    if (FW_INIT2(strm, 15, ZLIB_VER, ZS_SIZE) != 0) {
        FW_END(strm);
        return -1;
    }

    int ok = 0;
    if (mode == 2) {
        /* full frame: inflate directly into the display buffer */
        *(uint8_t **)(strm + ZS_NEXT_OUT) = disp;
        *(uint32_t *)(strm + ZS_AVAIL_OUT) = wh;
        int r = FW_INFLATE(strm, 4);                  /* Z_FINISH */
        ok = (r == 1 && *(uint32_t *)(strm + ZS_TOTAL_OUT) == wh);
    } else {
        /* mode 4: stereo pair [left frame (w*h)][right frame (w*h)]; this lens
         * keeps only its half. FW_SIDE(): 2=left -> first half, else -> second. */
        uint32_t want_lo = (FW_SIDE() == 2) ? 0 : wh;
        uint32_t want_hi = want_lo + wh;
        uint8_t chunk[XOR_CHUNK];
        uint32_t off = 0;
        for (;;) {
            *(uint8_t **)(strm + ZS_NEXT_OUT) = chunk;
            *(uint32_t *)(strm + ZS_AVAIL_OUT) = XOR_CHUNK;
            int r = FW_INFLATE(strm, 0);              /* Z_NO_FLUSH */
            uint32_t got = (uint32_t)(*(uint8_t **)(strm + ZS_NEXT_OUT) - chunk);
            for (uint32_t j = 0; j < got; j++) {
                uint32_t p = off + j;
                if (p >= want_lo && p < want_hi) disp[p - want_lo] = chunk[j];
            }
            off += got;
            if (off >= want_hi) { ok = 1; break; }    /* copied our whole half */
            if (r != 0 || got == 0) break;            /* end/error before full half */
        }
    }
    FW_END(strm);
    if (!ok) return -1;                               /* leave the old frame on screen */

    if (present) push_display(state, disp, w, h);
    return 0;
}

/* Expand this container's 4bpp shadow into the 8bpp display buffer, overlay the
 * diagnostic flags, and push. The single "present" step shared by the shadow-mutating
 * modes (3/6/9) and by the multi-segment batch (which presents once after all subs). */
static void present_shadow(uint8_t *state, uint32_t w, uint32_t h) {
    uint8_t *shadow = cfw_back_buffer(state, w, h);
    if (shadow == 0) return;
    uint8_t *disp = *(uint8_t **)(state + 0x8);
    unpack4bpp(disp, w, shadow, w, h, (w + 1) >> 1, 0);
    cfw_draw_flags(disp, w, h);
    push_display(state, disp, w, h);
}

/* Copy a bw x bh block of 4bpp pixels within one buffer (stride bytes/row) from
 * (sL,sT) to (dL,dT). The rects may overlap: iteration direction is chosen per axis
 * like a 2D memmove (bottom-up when the destination is lower, right-to-left when it is
 * further right). Fast whole-byte path when sL, dL and bw are all even; otherwise a
 * per-pixel nibble path (high nibble = the left / even pixel). */
static void rect_copy_4bpp(uint8_t *buf, uint32_t stride, uint32_t sL, uint32_t sT,
                           uint32_t dL, uint32_t dT, uint32_t bw, uint32_t bh) {
    int rev_y = (dT > sT);
    int rev_x = (dL > sL);
    if ((sL & 1) == 0 && (dL & 1) == 0 && (bw & 1) == 0) {
        uint32_t bytes = bw >> 1;
        for (uint32_t i = 0; i < bh; i++) {
            uint32_t y = rev_y ? (bh - 1 - i) : i;
            uint8_t *srow = buf + (sT + y) * stride + (sL >> 1);
            uint8_t *drow = buf + (dT + y) * stride + (dL >> 1);
            if (rev_x) { for (uint32_t x = bytes; x-- > 0; ) drow[x] = srow[x]; }
            else       { for (uint32_t x = 0; x < bytes; x++) drow[x] = srow[x]; }
        }
    } else {
        for (uint32_t i = 0; i < bh; i++) {
            uint32_t y = rev_y ? (bh - 1 - i) : i;
            uint8_t *srow = buf + (sT + y) * stride;
            uint8_t *drow = buf + (dT + y) * stride;
            for (uint32_t j = 0; j < bw; j++) {
                uint32_t x = rev_x ? (bw - 1 - j) : j;
                uint32_t sx = sL + x, dx = dL + x;
                uint8_t sv = (sx & 1) ? (srow[sx >> 1] & 0x0f) : (uint8_t)(srow[sx >> 1] >> 4);
                uint8_t *db = &drow[dx >> 1];
                if (dx & 1) *db = (uint8_t)((*db & 0xf0) | sv);
                else        *db = (uint8_t)((*db & 0x0f) | (uint8_t)(sv << 4));
            }
        }
    }
}

/* Replicate FUN_0050164a's tail: clean the display buffer out of dcache, set the
 * LVGL image descriptor for an 8bpp (cf=0x619) w*h image, rebind and invalidate.
 * (Defined after load_image_z so the entry offset, hence the bl target, is fixed.) */
static void push_display(uint8_t *state, uint8_t *disp, uint32_t w, uint32_t h) {
    uint32_t wh = w * h;
    uint32_t desc[2];
    desc[0] = (uint32_t)(uintptr_t)disp;
    desc[1] = wh;
    FW_FLUSH(desc);

    *(uint32_t *)(state + 0x24) = 0x619u;                                  /* cf/header */
    *(uint32_t *)(state + 0x2c) = (*(uint32_t *)(state + 0x2c) & 0xffff0000u) | w;
    *(uint32_t *)(state + 0x28) = (h << 16) | w;
    *(uint32_t *)(state + 0x30) = wh;
    *(uint32_t *)(state + 0x34) = (uint32_t)(uintptr_t)disp;

    uint32_t obj = *(uint32_t *)(state + 4);
    FW_SETSRC(obj, state + 0x24);
    FW_INVAL(obj);
}

/* little-endian unaligned reads (byte-wise; -mno-unaligned-access safe) */
static uint32_t rd16(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8); }
static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Expand a w*h block of 4bpp pixels (2 px/byte, high nibble = left pixel) into an
 * 8bpp destination: nibble n (0..15) -> n*17 (== (n<<4)|n) so 0->0, 15->255.
 * `src_stride` is bytes per source row; `dst_stride` is bytes per destination row
 * (= the full display width when writing a sub-rectangle); `bottom_up` flips the
 * source row order (BMP). */
static void unpack4bpp(uint8_t *dst, uint32_t dst_stride, const uint8_t *pix,
                       uint32_t w, uint32_t h, uint32_t src_stride, int bottom_up) {
    for (uint32_t y = 0; y < h; y++) {
        uint32_t srcY = bottom_up ? (h - 1 - y) : y;
        const uint8_t *row = pix + srcY * src_stride;
        uint8_t *out = dst + y * dst_stride;
        for (uint32_t x = 0; x < w; x++) {
            uint8_t b = row[x >> 1];
            uint8_t nib = (x & 1) ? (b & 0x0f) : (uint8_t)(b >> 4);
            out[x] = (uint8_t)(nib * 17);
        }
    }
}

/* Fast replacement for the stock BMP loader FUN_0050164a: decode a 4bpp indexed
 * BMP straight into the 8bpp display buffer via unpack4bpp, ignoring the palette
 * (always the gray ramp) and skipping the per-pixel color calls + CRC pass. Only
 * width/height/bpp/pixel-offset are read from the header. Falls back to the stock
 * loader for anything that isn't a 4bpp BMP matching the container dimensions. */
static int load_bmp_fast(uint8_t *state, const uint8_t *bmp, uint32_t len) {
    if (bmp == 0 || len < 0x36 || bmp[0] != 0x42 || bmp[1] != 0x4d)  /* "BM" */
        return FW_LOADBMP(state, (void *)bmp, len);
    if (rd16(bmp + 0x1c) != 4)                                       /* not 4bpp */
        return FW_LOADBMP(state, (void *)bmp, len);

    uint32_t dataoff = rd32(bmp + 0x0a);
    int32_t bh_signed = (int32_t)rd32(bmp + 0x16);
    uint32_t w = rd32(bmp + 0x12);
    uint32_t h = (bh_signed < 0) ? (uint32_t)(-bh_signed) : (uint32_t)bh_signed;
    int bottom_up = bh_signed > 0;

    /* Dimensions must match the container's display buffer, else let the stock
     * loader handle (and reject) it — avoids writing past the display buffer. */
    if (w != *(uint16_t *)(state + 0x40) || h != *(uint16_t *)(state + 0x42) ||
        (uint64_t)dataoff >= len)
        return FW_LOADBMP(state, (void *)bmp, len);

    uint32_t stride = (((w + 1) >> 1) + 3) & ~3u;   /* BMP rows padded to 4 bytes */
    uint8_t *disp = *(uint8_t **)(state + 0x8);
    unpack4bpp(disp, w, bmp + dataoff, w, h, stride, bottom_up);
    push_display(state, disp, w, h);
    return 0;
}

/* Fetch (or lazily create) the CFW singleton context. Its pointer lives in a
 * spare word of the BLE-RX task context (CFW_CTX_SLOT); we only ever touch that
 * word from here, i.e. only after a CFW image message has arrived. The slot ptr
 * is range-checked to SRAM and the struct's magic verified before trusting it, so
 * warm-reset garbage can't be mistaken for a live context. Returns 0 if the
 * one-time struct malloc fails. */
static customCfwContext *getCustomCfwContext(void) {
    customCfwContext *ctx = *(customCfwContext **)CFW_CTX_SLOT;
    if (((uintptr_t)ctx & 3) == 0 && (uintptr_t)ctx - 0x20000000u < 0x00800000u &&
        ctx->magic == CFW_CTX_MAGIC)
        return ctx;
    ctx = (customCfwContext *)FW_MALLOC(sizeof(customCfwContext));
    if (ctx) {
        bzero((uint8_t *)ctx, sizeof(customCfwContext));
        ctx->magic = CFW_CTX_MAGIC;
        ctx->diag_hide = 1;    /* overlay off by default; mode 7 sub 2 turns it on */
        for (uint32_t i = 0; i < CFW_FID_RING; i++) ctx->recent_fids[i] = 0xffff;  /* sentinel */
    }
    *(customCfwContext **)CFW_CTX_SLOT = ctx;      /* 0 on OOM: retried next message */
    return ctx;
}

/* Return this container's 4bpp shadow (a persistent copy of the last frame, needed by
 * mode-3 deltas). It lives in the TAIL of the container's own recon buffer B =
 * *(state+0xc), which is w*h bytes but only ever holds a small compressed message in
 * its head — so the top ((w+1)/2)*h bytes are free, cost no extra RAM (important for
 * full-screen frames), and are per-container (no cross-tile aliasing). B is stable for
 * the life of a streaming session; a mode-6 keyframe fully seeds the shadow before any
 * delta uses it. The worker runs on a snapshot (src), so B's head is free to reuse. */
static uint8_t *cfw_back_buffer(uint8_t *state, uint32_t w, uint32_t h) {
    uint8_t *b = *(uint8_t **)(state + 0xc);
    if (b == 0) return 0;
    return b + (w * h - ((w + 1) >> 1) * h);   /* 4bpp shadow at the recon-buffer tail */
}

static void bzero(uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) buf[i] = 0;
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

/* Overlay one rectangle per diagnostic flag across the top-left of the frame:
 * light-gray (0xdd) = flag set, dark-gray (0x33) = unset. Order: reorder, skip,
 * dup, snap-overflow. With the snapshot-FIFO fix all four should stay dark. Suppressed
 * when diag_hide is set (mode 7). Drawn into the 8bpp display buffer. */
static void cfw_draw_flags(uint8_t *disp, uint32_t w, uint32_t h) {
    customCfwContext *ctx = getCustomCfwContext();
    if (ctx == 0 || ctx->diag_hide) return;
    uint8_t flags[4];
    flags[0] = ctx->f_reorder;
    flags[1] = ctx->f_skip;
    flags[2] = ctx->f_dup;
    flags[3] = ctx->f_snap_of;
    const uint32_t rw = 20, rh = 10, gap = 4;
    for (uint32_t i = 0; i < 4; i++) {
        uint8_t v = flags[i] ? 0xddu : 0x33u;
        uint32_t x0 = i * (rw + gap);
        for (uint32_t y = 0; y < rh && y < h; y++)
            for (uint32_t x = 0; x < rw && (x0 + x) < w; x++)
                disp[y * w + (x0 + x)] = v;
    }
}

/* ---- snapshot / restore: fix the producer/consumer race on the shared recon buffer ----
 *
 * Confirmed model: each lens independently reassembles the image into its OWN recon
 * buffer B (frames forwarded lens->lens over the BLE cmdPipe). At reconstruction-
 * complete (BOTH lenses) the code checks lens identity; only the RIGHT lens emits a
 * completion message, which is forwarded so BOTH lenses' DEFERRED handlers run it —
 * that deferred step is the cross-lens TIMING SYNC (both eyes flip together). The bug:
 * B is a single slot, and a new frame's reassembly can overwrite B before the pending
 * deferred handler reads it -> old frame lost (skip), new one read twice (dup).
 *
 * Fix: snapshot the (small) compressed message at completion, on BOTH lenses, into a
 * per-state FIFO (snapshot_side, hooked at the both-lens `bl FUN_0045a8ec`); the
 * deferred handler (image_deferred, both lenses) pops this lens's oldest snapshot and
 * runs the worker on IT, never touching the possibly-overwritten live B. Both lenses
 * do identical work on identical data, so the sync is preserved. */

/* Snapshot the just-completed message (B = *(state+0xc), len = *(state+0x20)) into the
 * per-state FIFO, then return the lens id (real FUN_0045a8ec) so the caller's RIGHT
 * gate still works. Reached via the naked shim snapshot_side, which supplies r7/r8. */
int cfw_snapshot(uint8_t *state, uint32_t container_id) {
    (void)container_id;
    customCfwContext *ctx = getCustomCfwContext();
    if (ctx && state) {
        uint8_t *b = *(uint8_t **)(state + 0xc);
        uint32_t len = *(uint32_t *)(state + 0x20);
        if (b && len) {
            int slot = -1, oldest_i = 0;
            uint32_t oldest = 0xffffffffu;
            for (int i = 0; i < CFW_SNAP_RING; i++) {
                if (ctx->snaps[i].state == 0) { slot = i; break; }
                if (ctx->snaps[i].seq < oldest) { oldest = ctx->snaps[i].seq; oldest_i = i; }
            }
            if (slot < 0) {                          /* full: evict globally-oldest */
                FW_FREE(ctx->snaps[oldest_i].buf);
                ctx->snaps[oldest_i].state = 0;
                ctx->f_snap_of = 1;
                slot = oldest_i;
            }
            uint8_t *copy = (uint8_t *)FW_MALLOC(len);
            if (copy) {
                for (uint32_t i = 0; i < len; i++) copy[i] = b[i];
                ctx->snaps[slot].state = state;
                ctx->snaps[slot].buf = copy;
                ctx->snaps[slot].len = len;
                ctx->snaps[slot].seq = ctx->snap_seq++;
            }
        }
    }
    return (int)FW_SIDE();   /* real FUN_0045a8ec: 1=RIGHT/2=LEFT, drives the RIGHT gate */
}

/* Naked shim reached by the redirected `bl FUN_0045a8ec` at the completion sites
 * (0x500a04 / 0x500df8, both lenses). r7 = state, r8 = containerId at that point, so
 * pass them to cfw_snapshot and tail-branch — cfw_snapshot returns the lens id, which
 * flows back to the caller for the RIGHT gate. It preserves r4-r11, so state/... survive. */
__attribute__((naked)) int snapshot_side(void) {
    __asm volatile(
        "mov r0, r7\n\t"       /* state */
        "mov r1, r8\n\t"       /* containerId */
        "b   cfw_snapshot\n\t" /* tail-call; resolved intra-.text by build.py */
    );
}

/* Replaces the deferred consumer's worker call (bl at 0x4ae9cc, both lenses). DRAINS all
 * of this lens's pending snapshots for `state` in FIFO (seq) order, running the worker on
 * each (ignoring the live B, which may be overwritten), then frees them. Draining all —
 * not just one — is required because the cross-lens timing sync can COALESCE several
 * completion messages into a single deferred call; handling only one would let the FIFO
 * fall arbitrarily far behind (-> ring overflow). If nothing is pending (a coalesced
 * extra call, whose frames were already drained), do nothing: NOT falling back to the
 * live buffer is what suppresses the spurious dup (that buffer was already shown via its
 * snapshot). Only if we have no context at all do we best-effort the live buffer. */
int image_deferred(uint8_t *state, uint8_t *src, uint32_t len) {
    customCfwContext *ctx = getCustomCfwContext();
    if (ctx == 0) return image_worker(state, src, len);   /* no ctx (OOM): best-effort */
    int r = 0;
    for (;;) {
        int slot = -1;
        uint32_t oldest = 0xffffffffu;
        for (int i = 0; i < CFW_SNAP_RING; i++)
            if (ctx->snaps[i].state == state && ctx->snaps[i].seq < oldest) {
                oldest = ctx->snaps[i].seq; slot = i;
            }
        if (slot < 0) break;                              /* drained all pending for this state */
        uint8_t *buf = ctx->snaps[slot].buf;
        uint32_t blen = ctx->snaps[slot].len;
        ctx->snaps[slot].state = 0;                       /* release the slot before working */
        r = image_worker(state, buf, blen);
        FW_FREE(buf);
    }
    (void)src; (void)len;
    return r;                                             /* 0 if nothing pending (no dup) */
}

