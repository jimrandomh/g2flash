#include <stdint.h>
#include "memory.h"
#include "cfw_context.h"
#include "rle.h"
#include "debug.h"
#include "message_transport.h"

static int image_worker(const uint8_t *src, uint32_t srclen);

/* The stream parser owns data until this synchronous handler returns. */
int cfw_message_received(const uint8_t *data, uint16_t size, uint16_t checksum) {
    customCfwContext *ctx = getCustomCfwContext();
    if (!ctx) return -1;
    ctx->message_probe.snapshot = (uint32_t)size | ((uint32_t)checksum << 16);
    return image_worker(data, size);
}

/*
 * Image/control handlers for the G2 CFW. DEFLATE is handled by the transport.
 *
 * Entered only through private SID-f0 messages. The stock EvenHub image path
 * is unmodified. Dispatch on the first byte of the reconstructed message:
 *   3           -> [3][l/4][t/2][w/4][h/2][fid16][rle]  bounding-box delta: composite a
 *                              tight-4bpp rectangle onto the persistent 640x480
 *                              shadow, then queue a direct physical-framebuffer
 *                              refresh. Box origin/size is quantized (left/width *4,
 *                              top/height *2). Needs a prior mode-6 keyframe.
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
 *   6           -> [6][rle]  headerless 4bpp full frame: RLE-decode the
 *                              tightly packed 640x480 pixels into the persistent CFW
 *                              shadow (seeding it for mode-3 deltas), then queue a
 *                              direct physical-framebuffer refresh.
 *   7           -> [7][sub]    diagnostic control (no display change): 0 clears the
 *                              overlay flags, 1 hides the overlay, 2 shows it.
 *   8           -> [8][count][len16][submsg]...  multi-segment: apply each sub-message
 *                              to the shadow with the panel push DEFERRED, then present
 *                              once — an atomic multi-op update (e.g. scroll = rect-copy
 *                              + delta). Bounded by the private message length; no
 *                              nesting. Intended for shadow ops (modes 3/6/9).
 *   9           -> [9][srcrect][dstrect]  rect-copy inside the 4bpp shadow (full uint16
 *                              L/T/W/H each; same size; may overlap), then present.
 *                              Pairs with a delta (usually via mode 8) to scroll.
 *   10          -> [10][enabled] compass control (no display change): invokes the
 *                              firmware's own compass start/stop routines on the
 *                              right arm. enabled=2 adds [interval16][min-change16],
 *                              both little-endian; interval is clamped to 50..2000 ms
 *                              before configuring the stock compass event filter.
 *                              Navigation heading notifications carry the result plus
 *                              optional sample diagnostics (see compass.c).
 *   11          -> [11] cleanup the custom-app session before disconnect: release
 *                              leases/direct-framebuffer ownership, stop and delete
 *                              CFW timers, stop custom buzzer/compass activity, release
 *                              owned framebuffer shadow, and restore stock behavior. The
 *                              singleton CFW context and sticky allocation flag remain.
 *   12/13/14    -> retired (rejected).
 *   15          -> [15][x16][y16][options8][strlen8][UTF-8 string] draw with the
 *                              stock background 20 px font chain and its default
 *                              pair kerning. Bytes 1..31 adjust x by -10..20 as in
 *                              mode 20; options and clipping also match mode 20.
 *   16          -> [16][op]... ambient light sensor (no display change; master lens
 *                              only, see als_sensor.c). op 0 = QUERY one report; op 1
 *                              [flags][interval16][min-delta16][heartbeat16] = PASSIVE
 *                              START: the CFW polls the OPT3001 itself and the stock
 *                              auto-brightness adjuster never steps the panel; op 2 =
 *                              PASSIVE STOP. Reports arrive as sid-0x09 field 105.
 *   17          -> [17][0] query cached R1 battery (no display change).
 *                              Master replies on sid-0x09 field 106; see ring_battery.c.
 *   18          -> retired write-at-offset message; rejected.
 *   19          -> [19][resource-id16][x16][y16][options8] draw a cached image:
 *                              [width8][height8][4bpp RLE], decoded with clipping.
 *   20          -> [20][font-id16][x16][y16][options8][strlen8][string]
 *                              draw cached glyphs. Options contains a low-nibble
 *                              top color plus transparency (bit 4) and inverse (bit 5).
 *                              The font begins with 96 u16 offsets relative to its
 *                              own start for characters 32..127; zero means absent.
 *                              Bytes 1..31 adjust x by -10..20; each glyph advances
 *                              x by its cached image width.
 *   21          -> [21][count16]{[id16][total32][chunk-offset16][size16][bytes]}
 *                              Upload resource chunks, maximum total 65536 bytes.
 *                              Chunks are contiguous; exact received replays work.
 *                              Whole batch validated first. IDs must be distinct.
 *   22          -> [22][count16]{[id16]} evict distinct IDs (absent is a no-op).
 *                              IDs are 0..511. The 256 KiB cache starts with a
 *                              2 KiB pointer table; freelist/compaction preserves IDs.
 *   anything else / too short  -> reject the custom message.
 *
 * The HIGH BIT of the mode byte is a "lenses differ" flag; most modes ignore it. For
 * mode 3 it carries two boxes (left then right, same size) sharing one RLE payload —
 * a stereo shift without duplicating pixels; each lens draws at its own box. For mode 9
 * it carries two rect-sets (left then right); each lens uses its own.
 *
 * Custom modes 3/6/8/9/15/19/20 use a lazily allocated 153600-byte CFW
 * framebuffer shadow, independent of EvenHub containers.
 *
 * RLE (modes 3 and 6 only): message bodies contain run-length encoded pixels.
 * Transport DEFLATE wraps the entire message (including mode and image headers).
 * RLE runs over the pixel NIBBLES of tightly packed rows in wire order (high nibble
 * first), including each odd-width row's padding nibble. One token is:
 *
 *   [cnt4|color4]                       cnt 1..15   (1 byte)
 *   [0|color4][cnt8]                    cnt 1..255  (2 bytes)
 *   [0|color4][0][cntLo][cntHi]         cnt 1..65535, little-endian (4 bytes)
 *
 * The low nibble is always the 4bpp color; the high nibble is the repeat count, and 0
 * escapes to the wider forms. 65535 is the longest single run — an encoder splits
 * anything longer into consecutive tokens. A run may cross row boundaries. Tokens
 * are decoded directly from the validated message without image scratch allocation,
 * and same-color pixel pairs are written as
 * whole bytes (color*0x11) rather than nibble at a time. A stream that decodes to
 * anything other than exactly rows*rowbytes*2 nibbles is rejected and the previous
 * frame is left on screen.
 *
 * Every invocation (any mode) first kicks the EvenHub keepalive: stock firmware
 * resets the ticks-since-heartbeat counter only on the sid-0x0c heartbeat msg, so
 * a client streaming image updates to maximize throughput would otherwise trip the
 * "Connection lost" teardown. See FW_KEEPALIVE_RESET in image_worker_locked.
 *
 * Shadow modes serialize with the stock display semaphore. display_copy_hook
 * copies packed 4bpp into the physical framebuffer before the panel refresh.
 *
 * Self-contained: no external symbols, no writable globals. Firmware entry points
 * are called by absolute constant address (movw/movt + blx, no relocation).
 * Addresses of OUR OWN functions (the z_stream zalloc/zfree pair, the seq_tick
 * osTimer callback) are taken with plain `&fn`: under -fropi clang materializes an
 * intra-CU function address PC-relatively (movw/movt of a resolved constant +
 * `add rX, pc`, Thumb bit included) with no relocation at all, so it needs no load
 * address at build time and stays correct wherever the blob is placed.
 */

typedef void (*cacheflush_fn)(void *desc);          /* desc = uint32[2]{ptr,size} */
typedef uint32_t (*lens_side_fn)(void);             /* 2 = LEFT lens, 1 = RIGHT lens */
typedef void (*buzz_preset_fn)(uint32_t type);      /* DRV_BuzzerPlayAfterQueue */
typedef void (*buzz_note_fn)(uint32_t note, uint32_t tone, uint32_t beat); /* DRV_BuzzerPlayNote */
typedef void (*buzz_reset_fn)(void);                /* buzzer stop/reset */
typedef void (*buzz_raw_fn)(uint32_t freq, uint32_t duty);   /* reset+power+PWM(freq,duty) */
typedef int  (*timer_start_fn)(uint32_t handle, uint32_t ms); /* osTimer start (one-shot) */
typedef uint32_t (*timer_new_fn)(void *cb, uint32_t type, void *arg, void *attr); /* osTimerNew-> handle */
typedef int  (*timer_stop_fn)(uint32_t handle);     /* osTimer stop */
typedef int  (*timer_delete_fn)(uint32_t handle);   /* osTimer delete */
typedef void (*app_start_fn)(unsigned app_id, void *arg, unsigned arg_len, void *cb);
typedef void (*keepalive_reset_fn)(void);           /* zero the EvenHub keepalive counter */
typedef void (*display_gate_fn)(void);               /* display semaphore take/give */
typedef int  (*display_queue_fn)(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
typedef void (*display_copy_fn)(void);               /* stock 576x288 -> 640x480 packed copy */
typedef int (*compass_control_fn)(void);              /* stock Start/StopIMUCompassFunc */
typedef int (*compass_config_fn)(uint32_t, const uint32_t *); /* sensor-hub FuncConfig */

/* firmware entry points (Thumb bit set for blx via constant pointer) */
#define FW_FLUSH   ((cacheflush_fn)0x0047e317U)     /* FUN_0047e316 dcache clean range */
#define FW_SIDE    ((lens_side_fn)0x00465d4dU)       /* FUN_00465d4c -> 2=left, 1=right */
#define FW_BUZZ_PRESET ((buzz_preset_fn)0x0051bccfU) /* FUN_0051bcce DRV_BuzzerPlayAfterQueue(type 0..8) */
#define FW_BUZZ_NOTE   ((buzz_note_fn)0x0051bd6dU)   /* FUN_0051bd6c DRV_BuzzerPlayNote(note,tone,beat) */
#define FW_BUZZ_RESET  ((buzz_reset_fn)0x0051bc39U)  /* FUN_0051bc38 buzzer stop/reset */
#define FW_BUZZ_RAW    ((buzz_raw_fn)0x0051bdfdU)     /* FUN_0051bdfc reset+power+PWM(freq,duty) */
#define FW_TIMER_START ((timer_start_fn)0x00442c4dU)  /* FUN_00442c4c osTimerStart(handle,ms) */
#define FW_TIMER_NEW   ((timer_new_fn)0x00442b65U)    /* FUN_00442b64 osTimerNew(cb,type,arg,attr) */
#define FW_TIMER_STOP  ((timer_stop_fn)0x00442c8dU)   /* FUN_00442c8c osTimerStop(handle) */
#define FW_TIMER_DELETE ((timer_delete_fn)0x00442cf3U) /* FUN_00442cf2 osTimerDelete(handle) */
#define FW_APP_START ((app_start_fn)0x0046a673U)       /* FUN_0046a672 REQUEST_DISPLAY_START_UP */
#define FW_KEEPALIVE_RESET ((keepalive_reset_fn)0x004f7cdfU) /* FUN_004f7cde: EvenHub keepalive
                                                     * counter (@0x20078454) = 0. This is the exact
                                                     * leaf the stock sid-0x0c heartbeat handler in
                                                     * the EvenHub UI event handler calls; it takes no args and reads
                                                     * the counter pointer from its own literal pool. */
#define FW_DISPLAY_WAIT   ((display_gate_fn)0x0047a31fU)  /* FUN_0047a31e: take display semaphore */
#define FW_DISPLAY_SIGNAL ((display_gate_fn)0x0047a36bU)  /* FUN_0047a36a: give display semaphore */
#define FW_DISPLAY_QUEUE  ((display_queue_fn)0x0047ac1fU) /* FUN_0047ac1e: queue type-3 refresh */
#define FW_DISPLAY_COPY   ((display_copy_fn)0x00470aa1U)  /* FUN_00470aa0: stock packed-buffer copy */
#define FW_COMPASS_START  ((compass_control_fn)0x0056311bU) /* FUN_0056311a StartIMUCompassFunc */
#define FW_COMPASS_STOP   ((compass_control_fn)0x005631a3U) /* FUN_005631a2 StopIMUCompassFunc */
#define FW_COMPASS_CONFIG ((compass_config_fn)0x004bb011U) /* FUN_004bb010: FuncConfig(type,config) */
#define FW_DISPLAY_FB     (*(uint8_t * volatile *)0x200008b8U) /* stock copier's 640x480 destination */
#define BUZZ_TIMER_ADDR 0x20077834U                   /* RAM: buzzer osTimer handle global */

#define PANEL_W 640u
#define PANEL_H 480u
#define PANEL_STRIDE (PANEL_W / 2u)
#define PANEL_BYTES (PANEL_STRIDE * PANEL_H)
#define IMAGE_W PANEL_W
#define IMAGE_H PANEL_H
#define IMAGE_X 0u
#define IMAGE_Y 0u

/* Buzzer tone-sequence timer callback (mode-5 kind 4). Plays seq_steps[cursor],
 * advances the cursor, and re-arms this timer for that step's ms; after the final
 * step's ms elapses it powers the PWM off and goes idle. `arg` is the singleton
 * context (passed as the osTimer argument at creation). Runs in the RTOS timer
 * thread — the only shared state is the singleton, guarded by magic + bounds.
 * Non-static (external linkage) so -O2 keeps it despite having no direct caller —
 * osTimerNew only ever receives it as a fn-ptr value. */
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

/* Keep the callback-address relocation close to seq_tick: the Thumb MOVW/MOVT
 * relocation addend is signed 16-bit even though the resulting address is 32-bit. */
__attribute__((noinline)) uint32_t cfw_create_buzzer_timer(customCfwContext *ctx) {
    return FW_TIMER_NEW((void *)&seq_tick, 0, ctx, 0);
}

static uint8_t *cfw_shadow_buffer(void);
static int is_shadow_message(const uint8_t *src, uint32_t srclen);
static int cfw_cleanup_session(void);
static void mic_cleanup_session(void);   /* mic_control.c (same TU): mic hw + lease teardown */
static void als_cleanup_session(void);   /* als_sensor.c (same TU): passive ALS teardown */
int ring_battery_control(const uint8_t *src, uint32_t srclen); /* mode 17 */
int als_control(const uint8_t *src, uint32_t srclen); /* als_sensor.c: mode 16 */

static int decode_image_rle(const uint8_t *src, uint32_t size, uint8_t *base, uint32_t stride, uint32_t rowbytes, uint32_t rows);
static void present_shadow(uint32_t w, uint32_t h, cfw_rectlist *rl);
static int image_dispatch(const uint8_t *src, uint32_t srclen, int present, cfw_rectlist *rl);


/* True for top-level messages that need exclusive ownership of the stock display
 * gate. Mode 8 mutates/presents the custom shadow; mode 11 uses the gate as a
 * barrier so no direct-framebuffer job can still reference session-owned state. */
static int is_shadow_message(const uint8_t *src, uint32_t srclen) {
    if (src == 0 || srclen == 0) return 0;
    uint8_t mode = src[0] & 0x7fu;
    return mode == 3 || mode == 6 || mode == 8 || mode == 9 || mode == 11 ||
           mode == 15 || mode == 19 || mode == 20;
}

/* Commands can arrive on both BLE and bridge receive tasks.
 * Serialize handlers (including cache/control ops),
 * then use the display gate separately to protect the asynchronous panel copy. */
#define CFW_IMAGE_MUTEX_NEW ((uint32_t (*)(void *))0x00442ef7u)
#define CFW_IMAGE_MUTEX_TAKE ((int (*)(uint32_t, uint32_t))0x00442f91u)
#define CFW_IMAGE_MUTEX_GIVE ((int (*)(uint32_t))0x00442ff7u)
#define CFW_IMAGE_MUTEX_DELETE ((int (*)(uint32_t))0x00443049u)
static int image_worker_locked(const uint8_t *src, uint32_t size);
static int image_worker(const uint8_t *src, uint32_t size) {
    customCfwContext *ctx = getCustomCfwContext();
    if (!ctx) return -1;
    uint32_t mutex = __atomic_load_n(&ctx->image_mutex, __ATOMIC_ACQUIRE);
    if (!mutex) {
        mutex = CFW_IMAGE_MUTEX_NEW(0);
        if (!mutex) return -1;
        uint32_t expected = 0;
        if (!__atomic_compare_exchange_n(&ctx->image_mutex, &expected, mutex,
                                         0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
            CFW_IMAGE_MUTEX_DELETE(mutex);
            mutex = expected;
        }
    }
    if (CFW_IMAGE_MUTEX_TAKE(mutex, 0xffffffffu) != 0) return -1;
    int result = image_worker_locked(src, size);
    CFW_IMAGE_MUTEX_GIVE(mutex);
    return result;
}

/* Private receive calls this dispatcher under the image mutex. Each receiving lens kicks the keepalive once per top-level
 * command, then image_dispatch recurses for multi-segment messages. */
static int image_worker_locked(const uint8_t *src, uint32_t srclen) {
    /* An inbound image message proves the phone is still connected, so kick the
     * EvenHub keepalive back to life exactly as the stock heartbeat handler does.
     * Stock firmware resets the ticks-since-last-heartbeat counter (@0x20078454)
     * ONLY on the sid-0x0c heartbeat message; the periodic evenhub_ui_event_handler
     * periodic EvenHub event handler increments it every tick and, once it passes 899,
     * fires the display auto-reflash heartbeat-timeout path, which closes the
     * "Connection lost" context teardown. A client streaming image updates to
     * maximize throughput would otherwise have to interleave heartbeats to avoid
     * that teardown; resetting here lets a steady image stream keep the context
     * alive on its own. The private transport still keeps the existing layout
     * alive while it is present; the reset helper only updates the global counter. */
    FW_KEEPALIVE_RESET();

    /* Time this whole message. The display-task overlay can run before this worker
     * stores the new value, so its worker duration may lag by one update. */
    cfw_rectlist rl;                               /* per-frame updated-rect list (stack) */
    rl.n = 0;
    rl.direct_submitted = 0;
    rl.direct_failed = 0;

    /* Shadow updates bypass LVGL, but still use the stock display task to refresh
     * the panel. Take its gate before touching the shared shadow and leave it held
     * through the queued refresh; the stock task signals it after display_copy_hook.
     * This prevents the next pipelined delta from changing the shadow while the hook
     * is copying it. Non-image control messages never take the gate. */
    customCfwContext *ctx = getCustomCfwContext();
    int gated = is_shadow_message(src, srclen);
    if (gated) {
        if (ctx == 0) return -1;
        FW_DISPLAY_WAIT();
        if (ctx->direct_pending) return -1;          /* timed out; caller does not own gate */
    }

    uint32_t t;
    cfw_time_start(&t);
    int r = image_dispatch(src, srclen, 1, &rl);
    if (rl.direct_failed) r = -1;
    uint32_t us = cfw_time_end(&t);

    if (gated && !rl.direct_submitted) FW_DISPLAY_SIGNAL();
    if (ctx) ctx->last_worker_us = us;
    return r;
}

/* Dispatch one message. `present`=1 means push the result to the panel now; a
 * multi-segment message (mode 8) dispatches each sub-message with present=0 (so they
 * only mutate the shadow) and then presents once, giving an atomic multi-op update
 * (e.g. scroll = rect-copy + delta). The high bit of the mode byte is the "lenses
 * differ" flag; most modes ignore it. */
static int image_dispatch(const uint8_t *src, uint32_t srclen, int present, cfw_rectlist *rl) {
    if (src == 0 || srclen < 1) return -1;

    int lenses_differ = src[0] & 0x80;             /* high bit: per-lens variant */
    uint8_t mode = src[0] & 0x7f;

    if (mode == 5) {
        /* play a UI sound on the buzzer; no display change. [5][kind][args...].
         * kinds 0-3 use firmware entry points that copy their args into fw-owned
         * storage (preset table is flash; PlayNote copies into an 8-byte scratch;
         * raw uses a one-shot on the buzzer's own timer), so the input buffer is
         * free to be released immediately. kind 4 (tone sequence) steps through our
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
            memcpy(ctx->seq_steps, src + 3, n * 5);
            ctx->seq_count = (uint8_t)n;
            ctx->seq_cursor = 0;
            if (n) {
                if (ctx->seq_timer == 0)
                    ctx->seq_timer = cfw_create_buzzer_timer(ctx);
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
         * Runs on each selected lens, so it clears/toggles
         * both eyes. */
        customCfwContext *ctx = getCustomCfwContext();
        uint8_t sub = (srclen >= 2) ? src[1] : 0xffu;
        if (ctx) {
            if (sub == 0) {
                ctx->f_reorder = ctx->f_skip = ctx->f_dup = 0;
                cfw_alloc_diag_clear();
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

    if (mode == 10) {
        /* Compass control (no display change):
         *   [10][0] stops
         *   [10][1] starts with the stock 1000 ms / 5 degree configuration
         *   [10][2][interval16][min-change16] starts, then applies the supplied
         *       little-endian configuration through the stock sensor-hub API.
         *       interval is clamped to 50..2000 ms; min-change is passed through.
         * The stock compass implementation owns the sensor setup, calibration,
         * sampling, and heading computation. Heading events normally reach the
         * sid-0x08 notifier only through Navigation's UI handler; mode 10 also
         * enables compass_report_event(), which forwards the sensor-hub report
         * with sample diagnostics without needing Navigation in foreground.
         * This deferred image handler runs on both lenses, but the stock firmware
         * logs that the left arm cannot open the IMU, so invoke it only on right. */
        if (srclen < 2) return -1;
        customCfwContext *ctx = getCustomCfwContext();
        if (ctx == 0) return -1;
        if (src[1] == 0) {
            ctx->compass_forward = 0;
            return FW_SIDE() == 1 ? FW_COMPASS_STOP() : 0;
        }
        uint8_t enabled = src[1];
        if (enabled == 1 || enabled == 2) {
            uint32_t config[2];
            if (enabled == 2) {
                if (srclen < 6) return -1;
                config[0] = (uint32_t)src[2] | ((uint32_t)src[3] << 8);
                config[1] = (uint32_t)src[4] | ((uint32_t)src[5] << 8);
                if (config[0] < 50u) config[0] = 50u;
                if (config[0] > 2000u) config[0] = 2000u;
            }
            ctx->compass_forward = 1;
            if (FW_SIDE() == 1) {
                int r = FW_COMPASS_START();
                if (r == 0 && enabled == 2) {
                    r = FW_COMPASS_CONFIG(2, config);
                    if (r != 0) FW_COMPASS_STOP();
                }
                if (r != 0) ctx->compass_forward = 0;
                return r;
            }
            return 0;
        }
        return -1;
    }

    if (mode == 17) {
        return ring_battery_control(src, srclen);
    }

    if (mode == 16) {
        /* Ambient light sensor query / passive polling control (no display change).
         * Runs on both lenses; als_control itself acts only on the master lens. */
        return als_control(src, srclen);
    }

    if (mode == 11) {
        /* Custom-session cleanup. image_worker owns the display gate here, so a
         * prior direct refresh has completed and the pointers below cannot still
         * be in use by display_copy_hook. Extra bytes are reserved and ignored. */
        return cfw_cleanup_session();
    }

    if (mode == 21) return cfw_resource_upload(src + 1, srclen - 1);
    if (mode == 22) return cfw_resource_evict(src + 1, srclen - 1);

    /* Custom shadow geometry is deliberately independent from the EvenHub carrier. */
    uint32_t w = IMAGE_W;
    uint32_t h = IMAGE_H;

    if (mode == 15 || mode == 19 || mode == 20) {
        uint8_t *shadow = cfw_shadow_buffer();
        if (shadow == 0) return -1;
        int r;
        if (mode == 19)
            r = cfw_texture_draw_image(shadow, (w + 1u) >> 1, w, h,
                                       src + 1, srclen - 1, rl);
        else if (mode == 20)
            r = cfw_texture_draw_string(shadow, (w + 1u) >> 1, w, h,
                                        src + 1, srclen - 1, rl);
        else
            r = cfw_builtin_draw_string(shadow, (w + 1u) >> 1, w, h,
                                        src + 1, srclen - 1, rl);
        if (r != 0) return r;
        if (present) present_shadow(w, h, rl);
        return 0;
    }

    if (mode == 8) {
        /* Multi-segment: [8][count][len16][submsg]... — dispatch each sub with
         * present=0 (mutate the shadow only), then present once, giving an atomic
         * multi-op update (e.g. scroll = rect-copy + delta, no intermediate flash).
         * Bounded by the private message length; no nesting
         * (a sub-message may not itself be a multi-segment message). Only shadow
         * operations (modes 3/6/9/15/19/20) are accepted. */
        if (!present) return -1;                       /* only valid at top level */
        if (srclen < 2) return -1;
        uint32_t count = src[1];
        uint32_t pos = 2;
        for (uint32_t i = 0; i < count; i++) {
            if (pos + 2 > srclen) return -1;
            uint32_t seglen = rd16(src + pos);
            pos += 2;
            if (seglen < 1 || pos + seglen > srclen) return -1;
            uint8_t submode = src[pos] & 0x7fu;
            if (submode != 3 && submode != 6 && submode != 9 &&
                submode != 15 &&
                submode != 19 && submode != 20) return -1;
            if (image_dispatch(src + pos, seglen, 0, rl) != 0) return -1;
            pos += seglen;
        }
        present_shadow(w, h, rl);               /* one atomic present */
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
        uint8_t *shadow = cfw_shadow_buffer();
        if (shadow == 0) return -1;
        rect_copy_4bpp(shadow, (w + 1) >> 1, sL, sT, dL, dT, sW, sH);
        rl_add(rl, dL, dT, dW, dH);                     /* updated region = destination rect */
        if (present) present_shadow(w, h, rl);
        return 0;
    }

    if ((mode != 3 && mode != 6) || srclen < 3)
        return -1;

    if (mode == 6) {
        /* Full headerless 4bpp frame. RLE-decode it into the persistent
         * CFW-owned shadow that mode-3 deltas composite
         * onto, so a mode-6 keyframe seeds a stable base, then present (unless
         * deferred by a multi-segment wrapper). */
        cfw_diag(0, 0);                               /* keyframe: rebaseline delta fid */
        uint32_t stride = (w + 1) >> 1;                          /* tight 4bpp */
        uint8_t *dst = cfw_shadow_buffer();
        if (dst == 0) return -1;                      /* no shadow allocation -> can't proceed */
        if (!decode_image_rle(src + 1, srclen - 1, dst, stride, stride, h)) return -1;
        rl_add(rl, 0, 0, w, h);                       /* keyframe updates the whole screen */
        if (present) present_shadow(w, h, rl);
        return 0;
    }

    if (mode == 3) {
        /* Bounding-box delta, composited onto a PERSISTENT 4bpp shadow of the last
         * frame kept in the CFW-owned allocation (see cfw_shadow_buffer),
         * then the packed shadow is queued for a direct framebuffer refresh.
         *
         * Messages arrive in stream order and the parser owns their input until
         * this handler returns.
         *
         *   [3][left/4][top/2][width/4][height/2][fid_lo][fid_hi][rle(box pixels)]
         * left/width are *4 (=> multiples of 4 => even) so left>>1 and bw>>1 are whole
         * byte offsets: each box row lands in the 4bpp shadow as a plain byte run, no
         * nibble shifting. fid is a uint16 per-frame counter (diagnostics). Rejected
         * (old frame kept) if the box isn't wholly in bounds. The sender must have sent
         * a mode-6 keyframe before/among deltas.
         *
         * lenses-differ variant: [3|80][Lbox 4][Rbox 4][fid 2][shared RLE]. Both boxes
         * must be the same size; each lens draws the SAME decompressed pixels at its own
         * box — a stereo shift (e.g. a raised dialog) with the pixel data sent once. */
        uint32_t box_off, fid_off, z_off;
        if (lenses_differ) {
            if (srclen < 12) return -1;               /* mode + 2 boxes + fid + some RLE */
            if (src[3] != src[7] || src[4] != src[8]) return -1;   /* boxes must match size */
            box_off = (FW_SIDE() == 2) ? 1 : 5;       /* left set / right set */
            fid_off = 9;
            z_off   = 11;
        } else {
            if (srclen < 8) return -1;                /* 4 box hdr + 2 fid + some RLE */
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

        /* Frame IDs are diagnostic only. The stream rejects repeated packets;
         * each accepted record executes even if a phone restart reused its ID. */
        cfw_diag(1, fid);

        uint32_t sstride = (w + 1) >> 1;              /* 4bpp shadow row stride */
        uint8_t *shadow = cfw_shadow_buffer();   /* persistent CFW-owned last frame */
        if (shadow == 0) return -1;                   /* no stable base -> keyframe resyncs */
        uint32_t rowbytes = bw >> 1;                  /* whole bytes (bw even) */

        /* Decode the box straight into its slot in the shadow: rows of rowbytes bytes
         * at the shadow's stride. left/bw are multiples of 4 so every row starts (and
         * ends) on a byte boundary. */
        if (!decode_image_rle(src + z_off, srclen - z_off, shadow + top * sstride + (left >> 1), sstride, rowbytes, bh))
            return -1;                                /* leave the old frame on screen */

        rl_add(rl, left, top, bw, bh);                /* updated region = this lens's box */
        if (present) present_shadow(w, h, rl); /* queue one full packed refresh */
        return 0;
    }

    return -1;
}


/* Publish the CFW-owned packed-4bpp shadow to the stock display task. image_worker
 * already owns the stock display gate, so the shadow cannot change until the task has
 * copied it. The display task consumes this job in display_copy_hook immediately before
 * its normal panel refresh, bypassing LVGL and the stock 576x288 compositor copy. */
static void present_shadow(uint32_t w, uint32_t h, cfw_rectlist *rl) {
    customCfwContext *ctx = getCustomCfwContext();
    uint8_t *shadow = cfw_shadow_buffer();
    if (ctx == 0 || shadow == 0 || w != IMAGE_W || h != IMAGE_H) {
        if (rl) rl->direct_failed = 1;
        return;
    }

    ctx->direct_shadow = shadow;
    ctx->direct_pending = 1;                          /* publish last */
    if (FW_DISPLAY_QUEUE(0, 0, 0, 0, PANEL_W, PANEL_H) != 0) {
        ctx->direct_pending = 0;
        ctx->direct_shadow = 0;
        if (rl) rl->direct_failed = 1;
        return;
    }
    if (rl) rl->direct_submitted = 1;
}


/* Transport has already inflated and checked the message CRC. RLE remains
 * local to image handlers, including nested mode-8 segments. */
static int decode_image_rle(const uint8_t *src, uint32_t size, uint8_t *base,
                            uint32_t stride, uint32_t rowbytes, uint32_t rows) {
    rle_state rs;
    rle_init(&rs, base, stride, rowbytes, rows);
    rle_feed(&rs, src, size);
    return !rs.err && rs.left == 0 && rs.st == 0;
}

/* Return the singleton to its stock-compatible idle state without freeing it.
 * Idempotent: successfully deleted timer handles are cleared immediately, while a timer whose delete command fails remains in the
 * context so a later cleanup can retry it. The sticky allocation diagnostic is
 * deliberately retained so cleanup cannot erase evidence of an earlier OOM. */
static int cfw_cleanup_session(void) {
    customCfwContext *ctx = peekCustomCfwContext();
    if (ctx == 0) return 0;

    /* Publish fail-open ownership first. image_worker holds the display gate,
     * making it safe to discard any direct job/pointer left by this session. */
    ctx->direct_lease_deadline = 0;
    ctx->direct_active = 0;
    ctx->direct_pending = 0;
    ctx->direct_shadow = 0;
    ctx->direct_failed = 0;
    cfw_resource_cache_release(ctx);
    cfw_shadow_release(ctx);

    /* Suppress callbacks before asking the timer service to stop/delete them;
     * a callback already dispatched on the timer thread will then be harmless. */
    ctx->seq_count = 0;
    ctx->seq_cursor = 0;
    if (ctx->seq_timer) {
        FW_TIMER_STOP(ctx->seq_timer);
        if (FW_TIMER_DELETE(ctx->seq_timer) == 0) ctx->seq_timer = 0;
    }
    FW_BUZZ_RESET();

    /* Stop any CFW microphone session (capture hardware, streaming lease, and
     * its watchdog timer) so a departing custom app cannot leave the mics on. */
    mic_cleanup_session();

    /* Give the ambient light sensor back to the stock auto-brightness machine. */
    als_cleanup_session();

    int compass_was_forwarding = ctx->compass_forward != 0;
    ctx->compass_forward = 0;
    if (compass_was_forwarding && FW_SIDE() == 1) FW_COMPASS_STOP();

    int launch_dashboard = ctx->wake_dashboard_pending != 0;
    ctx->wake_lease_deadline = 0;
    ctx->wake_dashboard_pending = 0;
    ctx->wake_nonce = 0;
    if (ctx->wake_fallback_timer) {
        FW_TIMER_STOP(ctx->wake_fallback_timer);
        if (FW_TIMER_DELETE(ctx->wake_fallback_timer) == 0)
            ctx->wake_fallback_timer = 0;
    }

    /* Diagnostics are inert while hidden. Keep their sticky history for later
     * inspection, but make sure no Faceclaw overlay reaches the stock session. */
    ctx->diag_hide = 1;

    if (launch_dashboard) FW_APP_START(1, 0, 0, 0);
    return 0;
}

/* Replaces both display-task calls to the stock 576x288 packed-buffer copier.
 * A pending custom job copies the full 640x480 shadow straight into the
 * physical 640x480 4bpp framebuffer. Once that succeeds, unrelated stock widget
 * repaints are suppressed while Faceclaw's fail-open framebuffer lease is valid:
 * the display task refreshes the already-correct physical buffer instead of
 * overwriting it with stale LVGL content. Lease release/expiry restores the transparent stock pass-through. */
void display_copy_hook(void) {
    customCfwContext *ctx = peekCustomCfwContext();
    if (ctx == 0 || !ctx->direct_pending || ctx->direct_shadow == 0) {
        if (ctx && ctx->direct_active) {
            uint32_t deadline = ctx->direct_lease_deadline;
            if (deadline != 0 && (int32_t)(deadline - FW_MS_TICK) > 0)
                return;                                  /* preserve the physical direct frame */
            ctx->direct_active = 0;                       /* fail open to the stock compositor */
        }
        FW_DISPLAY_COPY();
        return;
    }

    const uint8_t *shadow = ctx->direct_shadow;
    uint8_t *fb = FW_DISPLAY_FB;
    uint32_t t;
    cfw_time_start(&t);
    int ok = fb != 0;
    if (ok) {
        memcpy(fb, shadow, PANEL_BYTES);
        cfw_draw_flags(fb, PANEL_W, PANEL_H);
    }

    ctx->direct_pending = 0;                         /* consume before returning gate */
    ctx->direct_shadow = 0;
    if (ok) {
        uint32_t desc[2] = {(uint32_t)(uintptr_t)fb, PANEL_BYTES};
        FW_FLUSH(desc);
        ctx->direct_active = 1;
    } else {
        ctx->direct_active = 0;
        ctx->direct_failed = 1;
        FW_DISPLAY_COPY();
    }
    ctx->last_present_us = cfw_time_end(&t);
}
