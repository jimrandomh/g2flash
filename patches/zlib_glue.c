#include <stdint.h>
#include "memory.h"
#include "cfw_context.h"
#include "rle.h"
#include "debug.h"
#include "message_transport.h"

static int image_worker(const uint8_t *src, uint32_t srclen, uint8_t origin);

/* The stream parser owns data until this synchronous handler returns. */
int cfw_message_received(const uint8_t *data, uint16_t size, uint16_t checksum, uint8_t origin) {
    customCfwContext *ctx = getCustomCfwContext();
    if (!ctx) return -1;
    ctx->message_probe.snapshot = (uint32_t)size | ((uint32_t)checksum << 16);
    return image_worker(data, size, origin);
}

/* Private SID-f0 image/control messages. DEFLATE is handled by message_transport.c.
 * Unknown, retired, or truncated messages are rejected. Active drawing messages
 * stage screen/resource pixels; only CFW_MSG_PRESENT composes and presents.
 * Keep names and wire IDs in sync with Faceclaw CfwMessageType.kt and cfw-message-type.ts. */
typedef enum {
    /* Retired on the wire; retained as a phone-side optimizer format. Historical layout:
     * [3][l/4][t/2][w/4][h/2][fid16][rle]  bounding-box delta: composite a
     * tight-4bpp rectangle onto the persistent 640x480
     * shadow, then queue a direct physical-framebuffer
     * refresh. Box origin/size is quantized (left/width *4,
     * top/height *2). Needs a prior mode-6 keyframe. */
    CFW_MSG_BOUNDING_BOX = 3,
    /* [5][...]    play a UI sound on the arm buzzer (no display change).
     * The G2 "speaker" is a PWM piezo buzzer — it can only
     * emit square-wave tones, not PCM/WAV — so this drives
     * the firmware's own buzzer driver instead of streaming
     * samples. Sub-dispatch on src[1]:
     * 0 [0][type]            -> DRV_BuzzerPlayAfterQueue:
     * play preset voice `type` (0..8) from the flash
     * preset table (single beep / alarm / ringtone).
     * 1 [1][note][oct][beat] -> DRV_BuzzerPlayNote: one
     * tone. note 1..7, oct 0..3 (freq from the 28-
     * entry note table), beat = duration in ~62ms
     * units. Good for click/beep on tap/notification.
     * 2 [2]                  -> stop/silence the buzzer.
     * 3 [3][freqLo][freqHi][duty][msLo][msHi] -> raw tone:
     * program the PWM to an ARBITRARY frequency
     * (1..20000 Hz, 16-bit LE) at `duty` percent
     * (0..100) for `ms` milliseconds (16-bit LE).
     * Bypasses the 7-note x 4-octave lookup table
     * entirely (that table is just a convenience);
     * the hardware timer takes any Hz. Auto-stops
     * by arming the buzzer's own osTimer with a
     * null note list so the driver's timer callback
     * shuts the PWM off after `ms`. Enables fine /
     * microtonal pitch, chirps and pitch sweeps
     * (send a run of these), and sub-62ms durations.
     * The preset/note/stop entries are self-contained fw
     * entries that queue into the buzzer's osTimer; the
     * raw-tone entry drives the low-level PWM start and
     * arms that same osTimer for auto-stop. None spin or
     * block here. Returns 0 (success).
     * Kind 4: [4][count8]{[frequency16][duty8][duration16]} starts a timer-driven
     * tone sequence, copied into persistent CFW context (at most CFW_SEQ_MAX steps). */
    CFW_MSG_BUZZER = 5,
    /* Retired on the wire; retained as a phone-side optimizer format. Historical layout:
     * [6][rle]  headerless 4bpp full frame: RLE-decode the
     * tightly packed 640x480 pixels into the persistent CFW
     * shadow (seeding it for mode-3 deltas), then queue a
     * direct physical-framebuffer refresh. */
    CFW_MSG_FULL_FRAME = 6,
    /* [7][sub]    diagnostic control (no display change): 0 clears the
     * overlay flags, 1 hides the overlay, 2 shows it. */
    CFW_MSG_DIAGNOSTICS = 7,
    /* Retired on the wire; retained as a phone-side optimizer format. Historical layout:
     * [8][count][len16][submsg]...  multi-segment: apply each sub-message
     * to the shadow with the panel push DEFERRED, then present
     * once — an atomic multi-op update (e.g. scroll = rect-copy
     * + delta). Bounded by the private message length; no
     * nesting. Intended for shadow ops (modes 3/6/9). */
    CFW_MSG_MULTI_SEGMENT = 8,
    /* Retired on the wire; retained as a phone-side optimizer format. Historical layout:
     * [9][srcrect][dstrect]  rect-copy inside the 4bpp shadow (full uint16
     * L/T/W/H each; same size; may overlap), then present.
     * Pairs with a delta (usually via mode 8) to scroll. */
    CFW_MSG_RECT_COPY = 9,
    /* [10][enabled] compass control (no display change): invokes the
     * firmware's own compass start/stop routines on the
     * right arm. enabled=2 adds [interval16][min-change16],
     * both little-endian; interval is clamped to 50..2000 ms
     * before configuring the stock compass event filter.
     * Navigation heading notifications carry the result plus
     * optional sample diagnostics (see compass.c). */
    CFW_MSG_COMPASS = 10,
    /* [11] cleanup the custom-app session before disconnect: release
     * leases/direct-framebuffer ownership, stop and delete
     * CFW timers, stop custom buzzer/compass activity, release
     * owned framebuffer shadow, and restore stock behavior. The
     * singleton CFW context and sticky allocation flag remain. */
    CFW_MSG_CLEANUP = 11,
    /* Retired message ID; rejected. */
    CFW_MSG_RETIRED_12 = 12,
    /* Retired message ID; rejected. */
    CFW_MSG_RETIRED_13 = 13,
    /* Retired message ID; rejected. */
    CFW_MSG_RETIRED_14 = 14,
    /* Retired on the wire; retained as a phone-side optimizer format. Historical layout:
     * [15][x16][y16][options8][strlen8][UTF-8 string] draw with the
     * stock background 20 px font chain and its default
     * pair kerning. Bytes 1..31 adjust x by -10..20 as in
     * mode 20; options and clipping also match mode 20. */
    CFW_MSG_STOCK_FONT_STRING = 15,
    /* [16][op]... ambient light sensor (no display change; master lens
     * only, see als_sensor.c). op 0 = QUERY one report; op 1
     * [flags][interval16][min-delta16][heartbeat16] = PASSIVE
     * START: the CFW polls the OPT3001 itself and the stock
     * auto-brightness adjuster never steps the panel; op 2 =
     * PASSIVE STOP. Reports arrive as sid-0x09 field 105. */
    CFW_MSG_AMBIENT_LIGHT = 16,
    /* [17][0] query cached R1 battery (no display change).
     * Master replies on sid-0x09 field 106; see ring_battery.c. */
    CFW_MSG_RING_BATTERY = 17,
    /* Retired write-at-cache-offset message; rejected. Use CFW_MSG_UPLOAD_RESOURCE. */
    CFW_MSG_RETIRED_CACHE_WRITE = 18,
    /* Retired on the wire; retained as a phone-side optimizer format. Historical layout:
     * [19][resource-id16][x16][y16][options8] draw a cached image:
     * [width8][height8][4bpp RLE], decoded with clipping. */
    CFW_MSG_CACHED_IMAGE = 19,
    /* Retired on the wire; retained as a phone-side optimizer format. Historical layout:
     * [20][font-id16][x16][y16][options8][strlen8][string]
     * draw cached glyphs. Options contains a low-nibble
     * top color plus transparency (bit 4) and inverse (bit 5).
     * The font begins with 96 u16 offsets relative to its
     * own start for characters 32..127; zero means absent.
     * Bytes 1..31 adjust x by -10..20; each glyph advances
     * x by its cached image width. */
    CFW_MSG_CACHED_TEXT = 20,
    /* [21][count16]{[id16][total32][chunk-offset16][size16][bytes]}
     * Upload resource chunks, maximum total 65536 bytes.
     * Chunks are contiguous; exact received replays work.
     * Whole batch validated first. IDs must be distinct. */
    CFW_MSG_UPLOAD_RESOURCE = 21,
    /* [22][count16]{[id16]} Evict distinct resource IDs (absent is a no-op).
     * IDs are 0..511. The 192 KiB cache starts with a 2 KiB pointer table;
     * freelist allocation and compaction preserve resource IDs. */
    CFW_MSG_EVICT_RESOURCE = 22,
    /* [23][request-id16][lenses8][op8][length8][address32][value16]
     * Read panel registers, framebuffer bytes, or a diagnostic snapshot (op 0).
     * Validated by panel.c and queued onto the stock display task; replies retain the request ID. */
    CFW_MSG_PANEL_READ = 23,
    /* [24][request-id16][lenses8][op8][length8][address32][value16]
     * Write a supported panel register or execute a panel control operation.
     * See panel.c for allowed operations, baseline restoration, and watchdog handling. */
    CFW_MSG_PANEL_WRITE = 24,
    /* [25][request-id16][lenses8][pattern8][0][0:u32][value16]
     * Display a diagnostic panel pattern (0..18); pattern 18 takes a 4bpp fill value.
     * Runs on the stock display task with watchdog restoration; see panel.c. */
    CFW_MSG_PANEL_PATTERN = 25,
    /* [26][count16]{[length16][draw-call]} Validate and execute draw calls.
     * Calls target the persistent screen buffer unless a call selects a resource.
     * Does not present; draw opcodes and flags are declared in display_list.c. */
    CFW_MSG_DRAW_CALLS = 26,
    /* [27][resource-id16] Validate and set the root display list.
     * ID 65535 clears the root. The resource graph is validated before changing the root. */
    CFW_MSG_SET_ROOT_DISPLAY_LIST = 27,
    /* [28] Copy the screen buffer to the composition buffer, replay the root
     * display list for this lens, then present to the physical framebuffer under the display gate. */
    CFW_MSG_PRESENT = 28,
    /* [29][count16]{[id16][width16][height16]} Create zeroed raw image surfaces.
     * Each resource is at most 64 KiB. Recreating an identical existing surface preserves its pixels. */
    CFW_MSG_CREATE_SURFACE = 29,
} cfw_message_type;

/* The high bit historically selected separate lens coordinates in bbox/copy messages.
 * Current dispatch ignores it except for panel messages, which use the full byte. */
#define CFW_MSG_TYPE_MASK 127u
#define CFW_MSG_FLAG_LENSES_DIFFER 128u

/* RLE pixels are high-nibble first: [count4|color4], [color4][count8], or
 * [color4][0][count16]. A run may cross rows; 65535 is the maximum run length.
 * Private messages kick the stock EvenHub keepalive on receipt. Composition and
 * cleanup serialize with the stock display gate. Firmware entry points are
 * absolute donor addresses; internal callbacks remain PC-relative under -fropi. */

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
static int cfw_draw_right_lens(void) { return FW_SIDE()==1; }
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

static uint8_t *cfw_composition_buffer(void);
static int is_composition_message(const uint8_t *src, uint32_t srclen);
static int cfw_cleanup_session(void);
static void mic_cleanup_session(void);   /* mic_control.c (same TU): mic hw + lease teardown */
static void als_cleanup_session(void);   /* als_sensor.c (same TU): passive ALS teardown */
int ring_battery_control(const uint8_t *src, uint32_t srclen); /* mode 17 */
int als_control(const uint8_t *src, uint32_t srclen); /* als_sensor.c: mode 16 */

static int decode_image_rle(const uint8_t *src, uint32_t size, uint8_t *base, uint32_t stride, uint32_t rowbytes, uint32_t rows);
static void present_composition(uint32_t w, uint32_t h, cfw_rectlist *rl);
static int image_dispatch(const uint8_t *src, uint32_t srclen, cfw_rectlist *rl);


/* True for top-level messages that need exclusive ownership of the stock display
 * gate. Mode 28 renders/presents the composition buffer; mode 11 uses the gate as a
 * barrier so no direct-framebuffer job can still reference session-owned state. */
static int is_composition_message(const uint8_t *src, uint32_t srclen) {
    if (src == 0 || srclen == 0) return 0;
    cfw_message_type mode = (cfw_message_type)(src[0] & CFW_MSG_TYPE_MASK);
    return mode == CFW_MSG_CLEANUP || mode == CFW_MSG_PRESENT;
}

/* Commands can arrive on both BLE and bridge receive tasks.
 * Serialize handlers (including cache/control ops),
 * then use the display gate separately to protect the asynchronous panel copy. */
#define CFW_IMAGE_MUTEX_NEW ((uint32_t (*)(void *))0x00442ef7u)
#define CFW_IMAGE_MUTEX_TAKE ((int (*)(uint32_t, uint32_t))0x00442f91u)
#define CFW_IMAGE_MUTEX_GIVE ((int (*)(uint32_t))0x00442ff7u)
#define CFW_IMAGE_MUTEX_DELETE ((int (*)(uint32_t))0x00443049u)
static int image_worker_locked(const uint8_t *src, uint32_t size);
static void cfw_record_timer_paint(customCfwContext *ctx, uint32_t us);
static int cfw_animation_present(customCfwContext *ctx, cfw_rectlist *rl, int reset_time);
static void cfw_animation_tick(void *arg);

static void cfw_animation_schedule(customCfwContext *ctx) {
    ctx->animation_running=ctx->animation_pending;
    if (!ctx->animation_running) return;
    if (!ctx->animation_timer) ctx->animation_timer=FW_TIMER_NEW((void *)&cfw_animation_tick,0,ctx,0);
    ctx->animation_due_ms=FW_MS_TICK+45u;
    if (!ctx->animation_timer || FW_TIMER_START(ctx->animation_timer,45)!=0) ctx->animation_running=0;
}

/* Runs on the timer thread. Never wait on a command's mutex: cleanup may be
 * stopping this timer while holding it. A delayed/stale callback checks the
 * current deadline again after taking the mutex. It never renews the lease. */
static void cfw_animation_tick(void *arg) {
    customCfwContext *ctx=(customCfwContext *)arg;
    uint32_t mutex=__atomic_load_n(&ctx->image_mutex,__ATOMIC_ACQUIRE);
    if (!mutex) return;
    if (CFW_IMAGE_MUTEX_TAKE(mutex,0)!=0) {
        if (ctx->animation_timer) FW_TIMER_START(ctx->animation_timer,45);
        return;
    }
    uint32_t now=FW_MS_TICK;
    if (!ctx->animation_running || !cfw_fb_lease_active()) {
        ctx->animation_running=0;
    } else if ((int32_t)(now-ctx->animation_due_ms)>=0) {
        if (ctx->direct_pending || ctx->panel.pending) {
            cfw_animation_schedule(ctx);
        } else {
            cfw_rectlist rl;
            rl.n=0;
            rl.direct_submitted=0;
            rl.direct_failed=0;
            FW_DISPLAY_WAIT();
            if (!ctx->direct_pending && !ctx->panel.pending) {
                uint32_t started;
                cfw_time_start(&started);
                int result=cfw_animation_present(ctx,&rl,0);
                uint32_t us=cfw_time_end(&started);
                if (result!=0) ctx->animation_running=0;
                else cfw_record_timer_paint(ctx,us);
                if (!rl.direct_submitted) FW_DISPLAY_SIGNAL();
            } else cfw_animation_schedule(ctx);
        }
    } else if (ctx->animation_timer) {
        FW_TIMER_START(ctx->animation_timer,ctx->animation_due_ms-now);
    }
    CFW_IMAGE_MUTEX_GIVE(mutex);
}
static int image_worker(const uint8_t *src, uint32_t size, uint8_t origin) {
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
    int result = size && src[0] >= CFW_MSG_PANEL_READ && src[0] <= CFW_MSG_PANEL_PATTERN
        ? panel_queue(src, size, origin) : image_worker_locked(src, size);
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
    int gated = is_composition_message(src, srclen);
    if (gated) {
        if (ctx == 0) return -1;
        FW_DISPLAY_WAIT();
        if (ctx->direct_pending || ctx->panel.pending) return -1;          /* timed out; caller does not own gate */
    }

    uint32_t t;
    cfw_time_start(&t);
    int r = image_dispatch(src, srclen, &rl);
    if (rl.direct_failed) r = -1;
    uint32_t us = cfw_time_end(&t);

    if (gated && !rl.direct_submitted) FW_DISPLAY_SIGNAL();
    if (ctx) ctx->last_worker_us = us;
    return r;
}

/* Drawing stages screen/resource pixels; only mode 28 composes and presents. */
static int image_dispatch(const uint8_t *src, uint32_t srclen, cfw_rectlist *rl) {
    if (src == 0 || srclen < 1) return -1;

    cfw_message_type mode = (cfw_message_type)(src[0] & CFW_MSG_TYPE_MASK);

    /* A new staged frame must not leak through an animation redraw before
     * its PRESENT. All of these paths execute under image_mutex. */
    if (mode==CFW_MSG_DRAW_CALLS || mode==CFW_MSG_SET_ROOT_DISPLAY_LIST ||
        mode==CFW_MSG_UPLOAD_RESOURCE || mode==CFW_MSG_EVICT_RESOURCE || mode==CFW_MSG_CREATE_SURFACE) {
        customCfwContext *ctx=peekCustomCfwContext();
        if (ctx) ctx->animation_running=0;
    }

    if (mode == CFW_MSG_BUZZER) {
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

    if (mode == CFW_MSG_DIAGNOSTICS) {
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
                ctx->timer_paint_count = ctx->timer_paint_next = 0;
                ctx->timer_paint_average_us = 0;
            } else if (sub == 1) {
                ctx->diag_hide = 1;
            } else if (sub == 2) {
                ctx->diag_hide = 0;
            }
        }
        return 0;
    }

    if (mode == CFW_MSG_COMPASS) {
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

    if (mode == CFW_MSG_RING_BATTERY) {
        return ring_battery_control(src, srclen);
    }

    if (mode == CFW_MSG_AMBIENT_LIGHT) {
        /* Ambient light sensor query / passive polling control (no display change).
         * Runs on both lenses; als_control itself acts only on the master lens. */
        return als_control(src, srclen);
    }

    if (mode == CFW_MSG_CLEANUP) {
        /* Custom-session cleanup. image_worker owns the display gate here, so a
         * prior direct refresh has completed and the pointers below cannot still
         * be in use by display_copy_hook. Extra bytes are reserved and ignored. */
        return cfw_cleanup_session();
    }

    if (mode == CFW_MSG_UPLOAD_RESOURCE) return cfw_resource_upload(src + 1, srclen - 1);
    if (mode == CFW_MSG_EVICT_RESOURCE) return cfw_resource_evict(src + 1, srclen - 1);

    if (mode == CFW_MSG_CREATE_SURFACE) return cfw_resource_create(src+1,srclen-1);
    if (mode == CFW_MSG_DRAW_CALLS || mode == CFW_MSG_SET_ROOT_DISPLAY_LIST || mode == CFW_MSG_PRESENT) {
        if (!cfw_fb_lease_active()) return -1;
        customCfwContext *ctx=getCustomCfwContext();
        uint8_t *screen=cfw_screen_buffer();
        if(!ctx || !screen) return -1;
        ctx->draw_elapsed_ms=FW_MS_TICK-ctx->animation_origin_ms;
        cfw_draw_target target={screen,640,480,320,0};
        if(mode == CFW_MSG_DRAW_CALLS) {
            if(cfw_draw_run(ctx,src+1,srclen-1,target,0,0)) return -1;
            return cfw_draw_run(ctx,src+1,srclen-1,target,1,0);
        }
        if(mode == CFW_MSG_SET_ROOT_DISPLAY_LIST) {
            if(srclen!=3) return -1;
            uint32_t id=rd16(src+1);
            if(cfw_draw_root(ctx,id,target,0,0)) return -1;
            ctx->root_display_list=id==65535u?0:(uint16_t)(id+1);
            return 0;
        }
        if(srclen!=1) return -1;
        return cfw_animation_present(ctx,rl,1);
    }
    return -1;
}

/* Caller owns both image_mutex and the display gate. Only an inbound PRESENT
 * resets time; timer callbacks render from the same screen with a later time. */
static int cfw_animation_present(customCfwContext *ctx, cfw_rectlist *rl, int reset_time) {
    ctx->animation_running=0;
    ctx->animation_pending=0;
    uint32_t now=FW_MS_TICK;
    if (reset_time) ctx->animation_origin_ms=now;
    ctx->draw_elapsed_ms=now-ctx->animation_origin_ms;
    if (!cfw_fb_lease_active() || !ctx->screen_buffer) return -1;
    uint8_t *composition=cfw_composition_buffer();
    if (!composition) return -1;
    cfw_draw_target target={composition,640,480,320,0};
    uint32_t root=ctx->root_display_list?ctx->root_display_list-1u:CFW_DRAW_SCREEN;
    if (cfw_draw_root(ctx,root,target,0,0)) return -1;
    memcpy(composition,ctx->screen_buffer,CFW_FRAMEBUFFER_BYTES);
    if (cfw_draw_root(ctx,root,target,1,0)) return -1;
    present_composition(640,480,rl);
    if (rl->direct_failed) return -1;
    cfw_animation_schedule(ctx);
    return 0;
}

/* Publish the CFW-owned packed-4bpp shadow to the stock display task. image_worker
 * already owns the stock display gate, so the shadow cannot change until the task has
 * copied it. The display task consumes this job in display_copy_hook immediately before
 * its normal panel refresh, bypassing LVGL and the stock 576x288 compositor copy. */
static void present_composition(uint32_t w, uint32_t h, cfw_rectlist *rl) {
    customCfwContext *ctx = getCustomCfwContext();
    uint8_t *shadow = cfw_composition_buffer();
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
 * local to image handlers, including nested display lists. */
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
    ctx->animation_running=0;
    ctx->animation_pending=0;
    if (ctx->animation_timer) FW_TIMER_STOP(ctx->animation_timer);
    ctx->timer_paint_count=ctx->timer_paint_next=0;
    ctx->timer_paint_average_us=0;
    /* Keep the handle for reuse: a callback already dispatched may still be
     * retrying the mutex. The context and this one timer live for the boot. */
    ctx->direct_lease_deadline = 0;
    ctx->direct_active = 0;
    ctx->direct_pending = 0;
    ctx->direct_shadow = 0;
    ctx->direct_failed = 0;
    cfw_resource_cache_release(ctx);
    cfw_framebuffers_release(ctx);

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
    panel_service();
    customCfwContext *ctx = peekCustomCfwContext();
    if (ctx && ctx->panel.pattern) {
        ctx->direct_pending = 0;
        ctx->direct_shadow = 0;
        return;
    }
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
