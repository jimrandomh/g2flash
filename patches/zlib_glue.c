#include <stdint.h>

/*
 * zlib (DEFLATE) image support for the G2 CFW — multi-mode load wrapper.
 *
 * Replaces the one BMP-loader call FUN_0050164a(state, buf, len) in FUN_004ae69c.
 * Dispatch on the first byte of the reassembled image buffer:
 *
 *   'B' (0x42)  -> raw BMP: decode with our own fast 4bpp decoder (load_bmp_fast).
 *   3           -> [3][l/4][t/2][w/4][h/2][fid16][zlib(rle)]  bounding-box delta: composite a
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
 *   6           -> [6][zlib(rle)]  headerless 4bpp full frame: inflate + RLE-decode the
 *                              tightly packed 640x480 pixels into the persistent CFW
 *                              shadow (seeding it for mode-3 deltas), then queue a
 *                              direct physical-framebuffer refresh.
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
 *   10          -> [10][enabled] compass control (no display change): invokes the
 *                              firmware's own compass start/stop routines on the
 *                              right arm. Stock navigation notifications carry the
 *                              resulting heading/calibration events back to the phone.
 *   11          -> [11] cleanup the custom-app session before disconnect: release
 *                              leases/direct-framebuffer ownership, stop and delete
 *                              CFW timers, stop custom buzzer/compass activity, release
 *                              queued snapshot ranges, and restore stock behavior. The
 *                              singleton CFW context and sticky allocation flag remain.
 *   12          -> [12][offset32][length16] read a bounded chunk of XIP font
 *                              slot 0, or [12][slot][offset32][length16] to
 *                              select slot 0/1; return it on sid 0xe0.
 *   13          -> [13] snapshot the two four-entry runtime font chains and
 *                              the first 64 bytes addressed by each record;
 *                              return the diagnostic data on sid 0xe0.
 *   anything else / too short  -> load_bmp_fast (rejects cleanly if not a BMP).
 *
 * The HIGH BIT of the mode byte is a "lenses differ" flag; most modes ignore it. For
 * mode 3 it carries two boxes (left then right, same size) sharing one zlib payload —
 * a stereo shift without duplicating pixels; each lens draws at its own box. For mode 9
 * it carries two rect-sets (left then right); each lens uses its own.
 *
 * Custom modes 3/6/8/9 operate on the full 640x480 physical image. The EvenHub
 * container remains 576x288 and supplies two 165888-byte allocations: its display
 * buffer A holds the 153600-byte packed-4bpp shadow. Completed compressed messages
 * are packed into the unused tail of reconstruction buffer B until the deferred
 * consumer runs, avoiding a separate heap allocation per in-flight frame. Direct
 * mode does not otherwise use A, so this separation reaches the full panel without
 * another large allocation.
 *
 * RLE (modes 3 and 6 only): those two modes do not deflate the packed 4bpp bytes
 * directly — the pixels are first run-length encoded and the RLE STREAM is what gets
 * deflated, so the on-wire payload is zlib(rle(pixels)) and the firmware inflates then
 * RLE-decodes. RLE runs over the pixel NIBBLES of the tightly packed rows in wire
 * order (high nibble = left pixel), including the pad nibble that ends each row when
 * the width is odd — i.e. exactly the byte buffer that used to be deflated, read as
 * 2*len nibbles. One token is:
 *
 *   [cnt4|color4]                       cnt 1..15   (1 byte)
 *   [0|color4][cnt8]                    cnt 1..255  (2 bytes)
 *   [0|color4][0][cntLo][cntHi]         cnt 1..65535, little-endian (4 bytes)
 *
 * The low nibble is always the 4bpp color; the high nibble is the repeat count, and 0
 * escapes to the wider forms. 65535 is the longest single run — an encoder splits
 * anything longer into consecutive tokens. A run may cross row boundaries. Decoding is
 * streamed straight out of inflate through a small stack chunk (no scratch allocation,
 * tokens may straddle chunk boundaries), and same-color pixel pairs are written as
 * whole bytes (color*0x11) rather than nibble at a time. A stream that decodes to
 * anything other than exactly rows*rowbytes*2 nibbles is rejected and the previous
 * frame is left on screen.
 *
 * Every invocation (any mode) first kicks the EvenHub keepalive: stock firmware
 * resets the ticks-since-heartbeat counter only on the sid-0x0c heartbeat msg, so
 * a client streaming image updates to maximize throughput would otherwise trip the
 * "Connection lost" teardown. See FW_KEEPALIVE_RESET at the top of load_image_z.
 *
 * BMP handling (mode 'B') does NOT use the stock loader FUN_0050164a: that
 * decoder runs two non-inlined function calls PER PIXEL (palette pack + luminance
 * blend) plus a whole-buffer CRC, which costs more CPU than the airtime it saves.
 * load_bmp_fast instead does a direct 4bpp-nibble -> 8bpp (nibble*17) expand,
 * ignoring the palette (the sender only ever uses the standard gray ramp). The
 * stock loader is kept only as a fallback for non-4bpp or mismatched-size BMPs.
 *
 * Raw BMP remains on the legacy LVGL path for backwards compatibility. Custom
 * shadow modes bypass LVGL and the stock 576x288-to-640x480 copy: they serialize
 * with the stock display semaphore, and display_copy_hook copies packed 4bpp
 * directly into the physical framebuffer before the normal panel refresh.
 *
 * Self-contained: no external symbols, no writable globals. Firmware entry points
 * are called by absolute constant address (movw/movt + blx, no relocation).
 * Addresses of OUR OWN functions (the z_stream zalloc/zfree pair, the seq_tick
 * osTimer callback) are taken with plain `&fn`: under -fropi clang materializes an
 * intra-CU function address PC-relatively (movw/movt of a resolved constant +
 * `add rX, pc`, Thumb bit included) with no relocation at all, so it needs no load
 * address at build time and stays correct wherever the blob is placed.
 */

typedef void *(*malloc_fn)(uint32_t);
typedef void (*free_fn)(void *);
typedef void *(*heap_malloc_fn)(uint32_t descriptor, uint32_t size);
typedef void (*heap_free_fn)(uint32_t descriptor, void *ptr);
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
typedef int  (*timer_delete_fn)(uint32_t handle);   /* osTimer delete */
typedef void (*app_start_fn)(unsigned app_id, void *arg, unsigned arg_len, void *cb);
typedef void (*keepalive_reset_fn)(void);           /* zero the EvenHub keepalive counter */
typedef uint8_t *(*lookup_fn)(uint32_t container_id); /* container id -> spec-list node (or 0) */
typedef int  (*complete_emit_fn)(uint32_t id, void *hdr, int kind, uint32_t p4); /* completion emit */
typedef void (*display_gate_fn)(void);               /* display semaphore take/give */
typedef int  (*display_queue_fn)(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);
typedef void (*display_copy_fn)(void);               /* stock 576x288 -> 640x480 packed copy */
typedef int (*compass_control_fn)(void);              /* stock Start/StopIMUCompassFunc */
typedef int (*display_event_forward_fn)(uint32_t, uint32_t, void *); /* display event -> active UI */
typedef int (*compass_notify_fn)(uint32_t);           /* stock sid-0x08 compass notifier */
typedef int (*font_send_fn)(int type, int sid, unsigned char *buf, unsigned len);

/* firmware entry points (Thumb bit set for blx via constant pointer) */
#define FW_MALLOC  ((malloc_fn)0x00474cd3U)         /* FUN_00474cd2 malloc(size) */
#define FW_FREE    ((free_fn)0x00474d17U)           /* FUN_00474d16 free(ptr) */
#define FW_HEAP_MALLOC ((heap_malloc_fn)0x00484181U) /* FUN_00484180 generic heap malloc */
#define FW_HEAP_FREE   ((heap_free_fn)0x0048429fU)   /* FUN_0048429e generic heap free */
#define FW_HEAP_13_DESCRIPTOR 0x20000354U            /* TLSF arena @ 0x2013be70, 0xcd000 B */
#define FW_INIT2   ((inflateInit2_fn)0x005beac3U)   /* FUN_005beac2 inflateInit2_ */
#define FW_INFLATE ((inflate_fn)0x005beb91U)        /* FUN_005beb90 inflate */
#define FW_END     ((inflateEnd_fn)0x005bea87U)     /* FUN_005bea86 inflateEnd */
#define FW_LOADBMP ((loadbmp_fn)0x004dc5afU)        /* FUN_004dc5ae set_image_data / BMP decoder */
#define FW_FLUSH   ((cacheflush_fn)0x0047510fU)     /* FUN_0047510e dcache clean range */
#define FW_SETSRC  ((lv_set_src_fn)0x00498681U)     /* FUN_00498680 lv_image_set_src */
#define FW_INVAL   ((lv_invalidate_fn)0x00440657U)  /* FUN_00440656 lv_obj_invalidate */
#define FW_SIDE    ((lens_side_fn)0x0045a569U)       /* FUN_0045a568 -> 2=left, 1=right */
#define FW_BUZZ_PRESET ((buzz_preset_fn)0x00502b5bU) /* FUN_00502b5a DRV_BuzzerPlayAfterQueue(type 0..8) */
#define FW_BUZZ_NOTE   ((buzz_note_fn)0x00502bf9U)   /* FUN_00502bf8 DRV_BuzzerPlayNote(note,tone,beat) */
#define FW_BUZZ_RESET  ((buzz_reset_fn)0x00502ac5U)  /* FUN_00502ac4 buzzer stop/reset */
#define FW_BUZZ_RAW    ((buzz_raw_fn)0x00502c89U)     /* FUN_00502c88 reset+power+PWM(freq,duty) */
#define FW_TIMER_START ((timer_start_fn)0x00449499U)  /* FUN_00449498 osTimerStart(handle,ms) */
#define FW_TIMER_NEW   ((timer_new_fn)0x004493b1U)    /* FUN_004493b0 osTimerNew(cb,type,arg,attr) */
#define FW_TIMER_STOP  ((timer_stop_fn)0x004494d9U)   /* FUN_004494d8 osTimerStop(handle) */
#define FW_TIMER_DELETE ((timer_delete_fn)0x0044953fU) /* FUN_0044953e osTimerDelete(handle) */
#define FW_APP_START ((app_start_fn)0x00464b2fU)       /* FUN_00464b2e REQUEST_DISPLAY_START_UP */
#define FW_KEEPALIVE_RESET ((keepalive_reset_fn)0x004e0cbbU) /* FUN_004e0cba: EvenHub keepalive
                                                     * counter (@0x200745ac) = 0. This is the exact
                                                     * leaf the stock sid-0x0c heartbeat handler in
                                                     * the EvenHub UI event handler calls; it takes no args and reads
                                                     * the counter pointer from its own literal pool. */
#define FW_LOOKUP        ((lookup_fn)0x004e0ccfU)    /* FUN_004e0cce(id) -> spec node; state=*(node+0x10) */
#define FW_COMPLETE_EMIT ((complete_emit_fn)0x004da383U) /* FUN_004da382: stock image-complete emitter */
#define FW_DISPLAY_WAIT   ((display_gate_fn)0x0047381fU)  /* FUN_0047381e: take display semaphore */
#define FW_DISPLAY_SIGNAL ((display_gate_fn)0x0047386bU)  /* FUN_0047386a: give display semaphore */
#define FW_DISPLAY_QUEUE  ((display_queue_fn)0x00474067U) /* FUN_00474066: queue type-3 refresh */
#define FW_DISPLAY_COPY   ((display_copy_fn)0x0046ca15U)  /* FUN_0046ca14: stock packed-buffer copy */
#define FW_COMPASS_START  ((compass_control_fn)0x005455e5U) /* FUN_005455e4 StartIMUCompassFunc */
#define FW_COMPASS_STOP   ((compass_control_fn)0x0054566dU) /* FUN_0054566c StopIMUCompassFunc */
#define FW_DISPLAY_EVENT_FORWARD ((display_event_forward_fn)0x0045f8fdU) /* FUN_0045f8fc */
#define FW_COMPASS_NOTIFY ((compass_notify_fn)0x0058705dU) /* FUN_0058705c navigation_notify_compass_changed_cmd */
#define FW_FONT_SEND ((font_send_fn)0x00475b15U) /* FUN_00475b14 generic aa12 sender */
#define FW_DISPLAY_FB     (*(uint8_t * volatile *)0x200007b8U) /* stock copier's 640x480 destination */
#define BUZZ_TIMER_ADDR 0x20074504U                   /* RAM: buzzer osTimer handle (buzzer osTimer handle global) */
#define ZLIB_VER   ((const char *)0x0078d654U)      /* "1.1.4" */

#define PANEL_W 640u
#define PANEL_H 480u
#define PANEL_STRIDE (PANEL_W / 2u)
#define PANEL_BYTES (PANEL_STRIDE * PANEL_H)
#define IMAGE_W PANEL_W
#define IMAGE_H PANEL_H
#define IMAGE_STRIDE (IMAGE_W / 2u)
#define IMAGE_BYTES (IMAGE_STRIDE * IMAGE_H)
#define IMAGE_X 0u
#define IMAGE_Y 0u

/* --- glasses-side timing: Arm DWT cycle counter, clock derived by calibration -----
 * CYCCNT is a free-running core-cycle counter (~4 ns). To use it we (1) UNLOCK the DWT
 * via its CoreSight software-lock register (write 0xC5ACCE55 to LAR at DWT_base+0xFB0)
 * — without this, writes to DWT->CTRL are ignored and CYCCNT stays 0 (observed on hw);
 * then (2) set DEMCR.TRCENA and DWT->CTRL.CYCCNTENA. We re-assert all three cheaply per
 * measurement (only the idle SWO-trace block touches DEMCR).
 *
 * To convert cycles->us we need the core clock. The guessed global 0x20074254 reads 0
 * on hardware (it's only written on a DVFS event, if ever), so instead we CALIBRATE:
 * measure how many CYCCNT cycles elapse across one edge of the firmware's 1 ms OS tick
 * (RAM 0x20074a34, SysTick chain) — that IS cycles-per-ms. Cached in the ctx; a bounded
 * spin falls back to 250 MHz if the tick never advances. All divides are 32-bit
 * (hardware UDIV) — a 64-bit divide would emit an external __aeabi_uldivmod build.py
 * rejects. (Limitation: cached across DVFS; a clock switch makes the figure ~stale.) */
#define DWT_DEMCR   (*(volatile uint32_t *)0xE000EDFCU)  /* CoreDebug->DEMCR (TRCENA bit24) */
#define DWT_LAR     (*(volatile uint32_t *)0xE0001FB0U)  /* DWT CoreSight Lock Access Reg */
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000U)  /* DWT->CTRL (CYCCNTENA bit0) */
#define DWT_CYCCNT  (*(volatile uint32_t *)0xE0001004U)  /* DWT->CYCCNT (core cycles) */
#define FW_MS_TICK  (*(volatile uint32_t *)0x20074a34U)  /* firmware 1 ms OS tick (SysTick chain) */
#define FW_CORE_HZ  (*(volatile uint32_t *)0x20074254U)  /* guessed core-clock global (reads 0 on hw) */
#define DWT_UNLOCK_KEY 0xC5ACCE55U

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

#define RLE_CHUNK 256   /* mode-3/6 inflate scratch feeding the RLE decoder (stack) */

/* Persistent CFW-owned state that must survive image-container teardown/rebuild.
 * The image container (its display buffer A @ state+0x8 and recon buffer B @
 * state+0xc) is freed and reallocated on rebuild. The packed shadow lives in A
 * only for the lifetime of the current streaming layout and must be seeded by a
 * mode-6 keyframe after every rebuild. The bookkeeping that does
 * need to survive rebuilds is anchored by a pointer in 1 KiB of SRAM explicitly
 * removed from the top of the stock primary TLSF arena by patch_compress.py. The
 * stock arena is [0x20279670,0x202a6670); the patched size is 0x2cc00, reserving
 * [0x202a6270,0x202a6670) for CFW. Its first word holds the context pointer and
 * its second holds a magic-guarded sticky allocation-failure diagnostic. This is
 * deliberately carved out rather than
 * inferred padding: 0x20003ffc, used before EVENCFW/11, is actually the +0 callback
 * of the BLE-RX lifecycle object and stock code can BLX through it. The struct's
 * `magic` guards against warm-reset garbage; the slot ptr is range-checked before
 * dereference. */
#define CFW_FID_RING  16     /* recent mode-3 frame ids kept for duplicate detection */
#define CFW_SNAP_RING 12     /* in-flight compressed-message snapshots (per producer race depth) */
#define CFW_SNAP_BUSY_SEQ 0xffffffffU /* range is reserved by the deferred worker */
#define CFW_SEQ_MAX   48     /* max steps in a buzzer tone sequence (mode-5 kind 4) */
#define CFW_FONT_REPLY_HEADER 12U
#define CFW_FONT_SEND_MAX 4096U
#define CFW_FONT_CHUNK_MAX (CFW_FONT_SEND_MAX - CFW_FONT_REPLY_HEADER)
#define CFW_FONT_XIP_BASE_20 0x80100000U
#define CFW_FONT_XIP_BASE_22 0x80700000U
#define CFW_FONT_XIP_SLOT_SIZE (CFW_FONT_XIP_BASE_22 - CFW_FONT_XIP_BASE_20)
#define CFW_FONT_HEADER_MAGIC 0x5A5A5A5AU
#define CFW_FONT_PROBE_BASE 0x20002C00U
#define CFW_FONT_PROBE_SNAPSHOT_SIZE 0xA8U
#define CFW_FONT_PROBE_CONFIG_0 0x48U
#define CFW_FONT_PROBE_CONFIG_1 0x78U
#define CFW_FONT_PROBE_RECORD_SIZE 12U
#define CFW_FONT_PROBE_RECORDS 8U
#define CFW_FONT_PROBE_AUX_SIZE 64U
#define CFW_FONT_PROBE_HEADER 8U

/* One snapshotted compressed image message. Taken at reconstruction-complete (both
 * lenses), consumed FIFO in the deferred handler. Keyed by the owning image-state
 * pointer so multiple containers (e.g. faceclaw's 4 tiles) don't cross-feed. */
typedef struct {
    uint8_t *state;      /* owning image-state (key); 0 = empty slot */
    uint8_t *buf;        /* copy packed into this state's reconstruction-buffer tail */
    uint32_t len;
    volatile uint32_t seq; /* push order, or CFW_SNAP_BUSY_SEQ while being consumed */
} cfw_snap;

typedef struct {
    uint32_t magic;      /* CFW_CTX_MAGIC when valid */
    /* --- snapshot FIFO: fixes the producer/consumer race on the shared recon buffer.
     * snapshot_side() copies each completed message here (both lenses); image_deferred
     * drains this lens's pending snapshots and runs the worker on each, ignoring the
     * live (possibly-overwritten) recon buffer B. (The 4bpp shadow of the last frame,
     * needed by mode-3 deltas, reuses each container's display buffer A — see
     * cfw_shadow_buffer — so it's per-container and costs no extra RAM.) */
    cfw_snap snaps[CFW_SNAP_RING];
    uint32_t snap_seq;   /* next push sequence number */
    /* --- diagnostics, overlaid as a text line (verify the fix; should stay clear). Mode
     * 7 clears the flags / toggles the overlay visibility (diag_hide). --- */
    uint16_t last_fid;   /* last frame id seen (mode-3 messages) */
    uint16_t high_fid;   /* highest frame id seen */
    uint8_t  diag_seen;  /* recorded at least one frame yet */
    uint8_t  fid_resync; /* keyframe rebaselines the next delta's fid (no false skip) */
    uint8_t  diag_hide;  /* 1 = don't draw the flag overlay (default 0 = visible) */
    uint32_t last_worker_us;  /* image_worker() duration of the PREVIOUS message (overlay) */
    uint32_t last_present_us; /* present_shadow() duration of the PREVIOUS present (overlay) */
    uint32_t cyc_per_ms;      /* calibrated DWT cycles per 1 ms OS tick (0 = not yet done) */
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
    /* --- Faceclaw wake takeover. A volatile, fail-open ownership lease lets
     * Faceclaw defer the stock dashboard only while its phone process is
     * demonstrably alive. See settings_ext.c for the private sid-0x09 control
     * protocol and the double-tap / Even AI entry hooks. */
    uint32_t wake_lease_deadline;          /* FW_MS_TICK deadline; 0 = no owner */
    uint32_t wake_fallback_timer;          /* one-shot stock-dashboard fallback */
    uint16_t wake_nonce;                   /* current pending wake, 0 = none */
    uint8_t  wake_dashboard_pending;       /* dashboard request held for Faceclaw */
    volatile uint8_t compass_forward;      /* mode 10: forward global heading events to BLE */
    uint8_t  wake_notify_buf[16];          /* stable storage for sid-0x09 notify */
    uint8_t  wear_notify_buf[12];          /* stable storage for sid-0x10 wear notify */
    /* Lazily allocated on the first mode-12/13 request. The stock sender's
     * copy/queue lifetime is opaque, so it remains allocated until reboot. */
    uint8_t *font_reply_buf;
    /* Direct-framebuffer job. The EvenHub worker holds the stock display gate
     * before it mutates the shadow and until the display task consumes this
     * pointer, so no second snapshot or full-size display buffer is required. */
    const uint8_t *direct_shadow;
    volatile uint8_t direct_pending;
    uint8_t direct_failed;
    uint8_t direct_active;                    /* physical framebuffer currently owns the image */
    uint32_t direct_lease_deadline;            /* fail-open repaint-guard deadline */
} customCfwContext;

#define CFW_CTX_SLOT  0x202a6270U    /* first word of the CFW-reserved TLSF tail */
#define CFW_ALLOC_DIAG_SLOT 0x202a6274U /* second word: magic | sticky failure bit */
#define CFW_ALLOC_DIAG_MAGIC 0xA110CA7EU
#define CFW_CTX_MAGIC 0xC0FFEE66U    /* bumped for the context layout change */

/* Keep the allocation diagnostic outside customCfwContext so failure to allocate
 * that context is itself observable. The magic rejects uninitialized/warm-reset
 * SRAM; bit 0 is sticky until mode 7/subcommand 0 clears the diagnostics. */
static uint32_t cfw_alloc_diag(void) {
    volatile uint32_t *slot = (volatile uint32_t *)CFW_ALLOC_DIAG_SLOT;
    uint32_t v = *slot;
    if ((v & ~1u) != CFW_ALLOC_DIAG_MAGIC) {
        v = CFW_ALLOC_DIAG_MAGIC;
        *slot = v;
    }
    return v;
}

static void cfw_alloc_diag_clear(void) {
    *(volatile uint32_t *)CFW_ALLOC_DIAG_SLOT = CFW_ALLOC_DIAG_MAGIC;
}

__attribute__((noinline))
static void *cfw_malloc(uint32_t size) {
    cfw_alloc_diag();
    void *p = FW_MALLOC(size);
    if (p == 0)
        *(volatile uint32_t *)CFW_ALLOC_DIAG_SLOT = CFW_ALLOC_DIAG_MAGIC | 1u;
    return p;
}

/* Allocate from the independent 820 KiB TLSF arena at 0x2013be70. Go through
 * the stock generic heap coordinator rather than calling TLSF directly so the
 * descriptor's mutex, current-byte counter, and peak-byte counter stay valid. */
__attribute__((noinline))
static void *cfw_heap13_malloc(uint32_t size) {
    cfw_alloc_diag();
    void *p = FW_HEAP_MALLOC(FW_HEAP_13_DESCRIPTOR, size);
    if (p == 0)
        *(volatile uint32_t *)CFW_ALLOC_DIAG_SLOT = CFW_ALLOC_DIAG_MAGIC | 1u;
    return p;
}

__attribute__((noinline))
static void cfw_heap13_free(void *ptr) {
    FW_HEAP_FREE(FW_HEAP_13_DESCRIPTOR, ptr);
}

void *zwrap_alloc(void *opaque, uint32_t items, uint32_t size) {
    (void)opaque;
    return cfw_heap13_malloc(items * size);
}

void zwrap_free(void *opaque, void *ptr) {
    (void)opaque;
    cfw_heap13_free(ptr);
}

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

static void push_display(uint8_t *state, uint8_t *disp, uint32_t w, uint32_t h);
static void unpack4bpp(uint8_t *dst, uint32_t dst_stride, const uint8_t *pix,
                       uint32_t w, uint32_t h, uint32_t src_stride, int bottom_up);
static int load_bmp_fast(uint8_t *state, const uint8_t *bmp, uint32_t len);
static customCfwContext *peekCustomCfwContext(void);
static customCfwContext *getCustomCfwContext(void);
static uint8_t *cfw_shadow_buffer(uint8_t *state);
static void bzero(uint8_t *buf, uint32_t len);
static void cfw_snap_clear(cfw_snap *snap);
static int is_shadow_message(const uint8_t *src, uint32_t srclen);
static int cfw_diag(int has_fid, uint16_t fid);
static void cfw_draw_flags(uint8_t *disp, uint32_t w, uint32_t h);
static uint32_t rd16(const uint8_t *p);
static uint32_t rd32(const uint8_t *p);
static uint8_t *cfw_font_reply_buffer(customCfwContext *ctx);
static int font_read_reply(const uint8_t *src, uint32_t srclen);
static int font_probe_reply(const uint8_t *src, uint32_t srclen);
static int cfw_cleanup_session(void);

/* Per-frame list of updated rectangles (pixel coords), assembled on the stack of the
 * top-level image_worker and threaded through image_dispatch; present_shadow outlines
 * them in the display buffer when the debug overlay is on, to visualize update regions. */
#define CFW_RECT_MAX 16
typedef struct { uint16_t l, t, w, h; } cfw_rect;
typedef struct {
    uint32_t n;
    uint8_t direct_submitted;
    cfw_rect r[CFW_RECT_MAX];
} cfw_rectlist;
static void rl_add(cfw_rectlist *rl, uint32_t l, uint32_t t, uint32_t w, uint32_t h);

static int inflate_rle(uint8_t *strm, uint8_t *base, uint32_t stride,
                       uint32_t rowbytes, uint32_t rows);
static void present_shadow(uint8_t *state, uint32_t w, uint32_t h, cfw_rectlist *rl);
static void rect_copy_4bpp(uint8_t *buf, uint32_t stride, uint32_t sL, uint32_t sT,
                           uint32_t dL, uint32_t dT, uint32_t bw, uint32_t bh);
static int image_dispatch(uint8_t *state, const uint8_t *src, uint32_t srclen,
                          int present, cfw_rectlist *rl);
static uint32_t strlcpy(char *dst, const char *src, uint32_t len);
static uint32_t strnlen(const char *s, uint32_t maxlen);
static uint32_t strlcat(char *dst, const char *str, uint32_t len);


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

/* True for top-level messages that need exclusive ownership of the stock display
 * gate. Mode 8 mutates/presents the custom shadow; mode 11 uses the gate as a
 * barrier so no direct-framebuffer job can still reference session-owned state. */
static int is_shadow_message(const uint8_t *src, uint32_t srclen) {
    if (src == 0 || srclen == 0) return 0;
    uint8_t mode = src[0] & 0x7fu;
    return mode == 3 || mode == 6 || mode == 8 || mode == 9 || mode == 11;
}

/* The image worker: static, called from image_deferred (the deferred consumer, which
 * runs on BOTH lenses via the cross-lens-synchronized completion message). NOTE: image
 * handling lives here / in the deferred path on purpose — the sync-completion path
 * (image_complete) runs on only the RECEIVING lens, so doing the work there leaves the
 * other lens blank. image_worker kicks the keepalive once per top-level message, then
 * defers to image_dispatch (which recurses for multi-segment messages). */
static int image_worker(void *state_, uint8_t *src, uint32_t srclen) {
    /* An inbound image message proves the phone is still connected, so kick the
     * EvenHub keepalive back to life exactly as the stock heartbeat handler does.
     * Stock firmware resets the ticks-since-last-heartbeat counter (@0x200745ac)
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

    /* Time this whole message. The display-task overlay can run before this worker
     * stores the new value, so its worker duration may lag by one update. */
    cfw_rectlist rl;                               /* per-frame updated-rect list (stack) */
    rl.n = 0;
    rl.direct_submitted = 0;

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
    int r = image_dispatch((uint8_t *)state_, src, srclen, 1, &rl);
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
static int image_dispatch(uint8_t *state, const uint8_t *src, uint32_t srclen,
                          int present, cfw_rectlist *rl) {
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
                    ctx->seq_timer = FW_TIMER_NEW((void *)&seq_tick, 0, ctx, 0);
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
        /* Compass control (no display change): [10][0] stops, [10][1] starts.
         * The stock compass implementation owns the sensor setup, calibration,
         * sampling, and heading computation. Heading events normally reach the
         * sid-0x08 notifier only through Navigation's UI handler; mode 10 also
         * enables compass_event_forward(), which taps the earlier global display
         * event so Faceclaw does not need the stock Navigation app in foreground.
         * This deferred image handler runs on both lenses, but the stock firmware
         * logs that the left arm cannot open the IMU, so invoke it only on right. */
        if (srclen < 2) return -1;
        customCfwContext *ctx = getCustomCfwContext();
        if (ctx == 0) return -1;
        if (src[1] == 0) {
            ctx->compass_forward = 0;
            return FW_SIDE() == 1 ? FW_COMPASS_STOP() : 0;
        }
        if (src[1] == 1) {
            ctx->compass_forward = 1;
            if (FW_SIDE() == 1) {
                int r = FW_COMPASS_START();
                if (r != 0) ctx->compass_forward = 0;
                return r;
            }
            return 0;
        }
        return -1;
    }

    if (mode == 11) {
        /* Custom-session cleanup. image_worker owns the display gate here, so a
         * prior direct refresh has completed and the pointers below cannot still
         * be in use by display_copy_hook. Extra bytes are reserved and ignored. */
        return cfw_cleanup_session();
    }

    if (mode == 12) {
        return font_read_reply(src, srclen);
    }

    if (mode == 13) {
        return font_probe_reply(src, srclen);
    }

    /* Custom shadow geometry is deliberately independent from the EvenHub carrier. */
    uint32_t w = IMAGE_W;
    uint32_t h = IMAGE_H;

    if (mode == 8) {
        /* Multi-segment: [8][count][len16][submsg]... — dispatch each sub with
         * present=0 (mutate the shadow only), then present once, giving an atomic
         * multi-op update (e.g. scroll = rect-copy + delta, no intermediate flash).
         * Sized no larger than an uncompressed 4bpp logical image; no nesting
         * (a sub-message may not itself be a multi-segment message). Only shadow
         * operations (modes 3/6/9) are accepted. */
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
            uint8_t submode = src[pos] & 0x7fu;
            if (submode != 3 && submode != 6 && submode != 9) return -1;
            if (image_dispatch(state, src + pos, seglen, 0, rl) != 0) return -1;
            pos += seglen;
        }
        present_shadow(state, w, h, rl);               /* one atomic present */
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
        uint8_t *shadow = cfw_shadow_buffer(state);
        if (shadow == 0) return -1;
        rect_copy_4bpp(shadow, (w + 1) >> 1, sL, sT, dL, dT, sW, sH);
        rl_add(rl, dL, dT, dW, dH);                     /* updated region = destination rect */
        if (present) present_shadow(state, w, h, rl);
        return 0;
    }

    if ((mode != 3 && mode != 6) || srclen < 3)
        return load_bmp_fast(state, src, srclen);

    const uint8_t *zsrc = src + 1;
    uint32_t zlen = srclen - 1;

    uint8_t strm[ZS_SIZE];
    for (uint32_t i = 0; i < ZS_SIZE; i++) strm[i] = 0;
    *(const uint8_t **)(strm + ZS_NEXT_IN) = zsrc;
    *(uint32_t *)(strm + ZS_AVAIL_IN) = zlen;
    *(uint32_t *)(strm + ZS_ZALLOC) = (uint32_t)(uintptr_t)&zwrap_alloc;
    *(uint32_t *)(strm + ZS_ZFREE) = (uint32_t)(uintptr_t)&zwrap_free;
    *(uint32_t *)(strm + ZS_OPAQUE) = 0;

    if (mode == 6) {
        /* Full headerless 4bpp frame. Inflate + RLE-decode it into the persistent
         * shadow (this container's display allocation A) that mode-3 deltas composite
         * onto, so a mode-6 keyframe seeds a stable base, then present (unless
         * deferred by a multi-segment wrapper). */
        cfw_diag(0, 0);                               /* keyframe: rebaseline delta fid */
        uint32_t stride = (w + 1) >> 1;                          /* tight 4bpp */
        uint8_t *dst = cfw_shadow_buffer(state);
        if (dst == 0) return -1;                      /* no shadow allocation -> can't proceed */
        if (!inflate_rle(strm, dst, stride, stride, h)) return -1;
        rl_add(rl, 0, 0, w, h);                       /* keyframe updates the whole screen */
        if (present) present_shadow(state, w, h, rl);
        return 0;
    }

    if (mode == 3) {
        /* Bounding-box delta, composited onto a PERSISTENT 4bpp shadow of the last
         * frame kept in this container's display allocation (see cfw_shadow_buffer),
         * then the packed shadow is queued for a direct framebuffer refresh.
         *
         * The stale-base race that used to force a CFW-owned shadow is now fixed at the
         * source: the worker runs on a per-frame SNAPSHOT (drained in order by
         * image_deferred), not the live recon prefix, so successive deltas compose onto
         * the shadow in the right order. The shadow is stable in A; snapshots occupy
         * only B's unused tail until their deferred workers finish.
         *
         *   [3][left/4][top/2][width/4][height/2][fid_lo][fid_hi][zlib(rle(box pixels))]
         * left/width are *4 (=> multiples of 4 => even) so left>>1 and bw>>1 are whole
         * byte offsets: each box row lands in the 4bpp shadow as a plain byte run, no
         * nibble shifting. fid is a uint16 per-frame counter (diagnostics). Rejected
         * (old frame kept) if the box isn't wholly in bounds. The sender must have sent
         * a mode-6 keyframe before/among deltas.
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
        uint8_t *shadow = cfw_shadow_buffer(state);   /* persistent last frame (buffer A) */
        if (shadow == 0) return -1;                   /* no stable base -> keyframe resyncs */
        uint32_t rowbytes = bw >> 1;                  /* whole bytes (bw even) */

        *(const uint8_t **)(strm + ZS_NEXT_IN) = src + z_off;   /* zlib past box(es) + fid */
        *(uint32_t *)(strm + ZS_AVAIL_IN) = srclen - z_off;
        /* Decode the box straight into its slot in the shadow: rows of rowbytes bytes
         * at the shadow's stride. left/bw are multiples of 4 so every row starts (and
         * ends) on a byte boundary. */
        if (!inflate_rle(strm, shadow + top * sstride + (left >> 1), sstride, rowbytes, bh))
            return -1;                                /* leave the old frame on screen */

        rl_add(rl, left, top, bw, bh);                /* updated region = this lens's box */
        if (present) present_shadow(state, w, h, rl); /* queue one full packed refresh */
        return 0;
    }

    return -1;
}

/* Allocate the shared mode-12/13 sender buffer only if font data is actually
 * requested. It deliberately persists until reboot: the firmware sender's
 * ownership/copy point is not known, and font downloads are normally a rare
 * install-time operation. */
static uint8_t *cfw_font_reply_buffer(customCfwContext *ctx) {
    if (ctx->font_reply_buf == 0)
        ctx->font_reply_buf = (uint8_t *)cfw_malloc(CFW_FONT_SEND_MAX);
    return ctx->font_reply_buf;
}

/* Mode 12 exports only the two stock XIP font slots, never arbitrary memory:
 *
 *   legacy request = [12][offset u32 LE][length u16 LE] (slot 0)
 *   request        = [12][slot][offset u32 LE][length u16 LE]
 *   response = [12][status][offset u32 LE][total u32 LE][length u16 LE][data]
 *
 * Status 0 is success, 1 is a malformed request, 2 is an invalid font header,
 * 3 is an invalid offset/length, and 4 is an unknown slot. Only the right/master
 * lens sends; the synchronized left-lens handler remains silent. Stock firmware
 * initializes headers at 0x80100000 (SourceHanSansSC Light 20) and 0x80700000
 * (Light 22), a 6 MiB stride. The header has no byte-length field; offset 0x3c
 * belongs to its font configuration. Therefore `total` is the bounded 6 MiB slot
 * size. Each request is separately bounds-checked. The font data is capped at
 * 4084 bytes so the 12-byte response header plus data fits the stock sender's
 * 4096-byte limit. The seven-byte request remains compatible with EVENCFW/10-11.
 */
static int font_read_reply(const uint8_t *src, uint32_t srclen) {
    if (FW_SIDE() != 1) return 0;
    customCfwContext *ctx = getCustomCfwContext();
    if (ctx == 0) return -1;

    uint8_t *reply = cfw_font_reply_buffer(ctx);
    if (reply == 0) return -1;
    uint32_t status = 0;
    uint32_t slot = srclen == 8 ? src[1] : 0;
    uint32_t offset = srclen == 8 ? rd32(src + 2) :
                      (srclen == 7 ? rd32(src + 1) : 0);
    uint32_t length = srclen == 8 ? rd16(src + 6) :
                      (srclen == 7 ? rd16(src + 5) : 0);
    uint32_t total = CFW_FONT_XIP_SLOT_SIZE;
    uint32_t font_base = slot == 1 ? CFW_FONT_XIP_BASE_22 : CFW_FONT_XIP_BASE_20;
    const volatile uint8_t *font = (const volatile uint8_t *)font_base;

    if (srclen != 7 && srclen != 8) {
        status = 1;
    } else if (slot > 1) {
        status = 4;
    } else if (*(const volatile uint32_t *)font_base != CFW_FONT_HEADER_MAGIC) {
        status = 2;
    } else if (length == 0 || length > CFW_FONT_CHUNK_MAX ||
               offset > total || length > total - offset) {
        status = 3;
    }

    reply[0] = 12;
    reply[1] = (uint8_t)status;
    reply[2] = (uint8_t)offset;
    reply[3] = (uint8_t)(offset >> 8);
    reply[4] = (uint8_t)(offset >> 16);
    reply[5] = (uint8_t)(offset >> 24);
    reply[6] = (uint8_t)total;
    reply[7] = (uint8_t)(total >> 8);
    reply[8] = (uint8_t)(total >> 16);
    reply[9] = (uint8_t)(total >> 24);
    reply[10] = status == 0 ? (uint8_t)length : 0;
    reply[11] = status == 0 ? (uint8_t)(length >> 8) : 0;
    if (status == 0) {
        for (uint32_t i = 0; i < length; i++) {
            reply[CFW_FONT_REPLY_HEADER + i] = font[offset + i];
        }
    }
    FW_FONT_SEND(
        1, 0xe0, reply, CFW_FONT_REPLY_HEADER + (status == 0 ? length : 0)
    );
    return 0;
}

/* Snapshot the live font-manager state that is absent from the static firmware
 * image. The 2.2.6.10 manager owns one contiguous 0xa8-byte RAM region:
 *
 *   +0x00  background XIP-backed lv_font_t descriptor (0x24 bytes)
 *   +0x24  foreground XIP-backed lv_font_t descriptor (0x24 bytes)
 *   +0x48  background chain: four 12-byte configuration records
 *   +0x78  foreground chain: four 12-byte configuration records
 *
 * For each configuration record, append the first 64 bytes at its +4 pointer.
 * A type-0 record therefore yields its native lv_font_t; a type-1 record yields
 * its FreeType face path. Reads are allowed only from the mapped main image,
 * SRAM, or the two known XIP font slots, preventing malformed runtime state from
 * turning this diagnostic into an arbitrary peripheral read. Wire format:
 *
 *   [13][status][version=1][count=8][snapshot_len16][aux_len16]
 *   [0xa8-byte snapshot][8 * 64-byte pointed-to prefixes]
 *
 * Status 0 is success and 1 is a malformed request. Only the right/master lens
 * sends, matching mode 12. The response fits well below FW_FONT_SEND's 4096-byte
 * ceiling and reuses the persistent mode-12 response buffer.
 */
static int font_probe_reply(const uint8_t *src, uint32_t srclen) {
    (void)src;
    if (FW_SIDE() != 1) return 0;
    customCfwContext *ctx = getCustomCfwContext();
    if (ctx == 0) return -1;

    uint8_t *reply = cfw_font_reply_buffer(ctx);
    if (reply == 0) return -1;
    uint32_t status = srclen == 1 ? 0 : 1;
    uint32_t payload_len = status == 0 ?
        CFW_FONT_PROBE_SNAPSHOT_SIZE +
        CFW_FONT_PROBE_RECORDS * CFW_FONT_PROBE_AUX_SIZE : 0;

    reply[0] = 13;
    reply[1] = (uint8_t)status;
    reply[2] = 1;
    reply[3] = CFW_FONT_PROBE_RECORDS;
    reply[4] = (uint8_t)CFW_FONT_PROBE_SNAPSHOT_SIZE;
    reply[5] = (uint8_t)(CFW_FONT_PROBE_SNAPSHOT_SIZE >> 8);
    reply[6] = (uint8_t)CFW_FONT_PROBE_AUX_SIZE;
    reply[7] = (uint8_t)(CFW_FONT_PROBE_AUX_SIZE >> 8);

    if (status == 0) {
        const volatile uint8_t *snapshot =
            (const volatile uint8_t *)CFW_FONT_PROBE_BASE;
        for (uint32_t i = 0; i < CFW_FONT_PROBE_SNAPSHOT_SIZE; i++) {
            reply[CFW_FONT_PROBE_HEADER + i] = snapshot[i];
        }

        uint32_t aux_base = CFW_FONT_PROBE_HEADER + CFW_FONT_PROBE_SNAPSHOT_SIZE;
        for (uint32_t record = 0; record < CFW_FONT_PROBE_RECORDS; record++) {
            uint32_t chain_record = record & 3u;
            uint32_t config_offset =
                (record < 4 ? CFW_FONT_PROBE_CONFIG_0 : CFW_FONT_PROBE_CONFIG_1) +
                chain_record * CFW_FONT_PROBE_RECORD_SIZE;
            uint32_t pointer =
                *(const volatile uint32_t *)(CFW_FONT_PROBE_BASE + config_offset + 4u);
            uint32_t aux_offset = aux_base + record * CFW_FONT_PROBE_AUX_SIZE;
            int readable =
                (pointer >= 0x00437FE0U && pointer <= 0x007942E4U) ||
                (pointer >= 0x20000000U && pointer <= 0x202A6630U) ||
                (pointer >= CFW_FONT_XIP_BASE_20 && pointer <= 0x80CFFFC0U);
            const volatile uint8_t *pointed = (const volatile uint8_t *)pointer;
            for (uint32_t i = 0; i < CFW_FONT_PROBE_AUX_SIZE; i++) {
                reply[aux_offset + i] = readable ? pointed[i] : 0;
            }
        }
    }

    FW_FONT_SEND(1, 0xe0, reply, CFW_FONT_PROBE_HEADER + payload_len);
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

/* Publish this container's packed-4bpp shadow to the stock display task. image_worker
 * already owns the stock display gate, so the shadow cannot change until the task has
 * copied it. The display task consumes this job in display_copy_hook immediately before
 * its normal panel refresh, bypassing LVGL and the stock 576x288 compositor copy. */
static void present_shadow(uint8_t *state, uint32_t w, uint32_t h, cfw_rectlist *rl) {
    customCfwContext *ctx = getCustomCfwContext();
    uint8_t *shadow = cfw_shadow_buffer(state);
    if (ctx == 0 || shadow == 0 || w != IMAGE_W || h != IMAGE_H) return;

    ctx->direct_shadow = shadow;
    ctx->direct_pending = 1;                          /* publish last */
    if (FW_DISPLAY_QUEUE(0, 0, 0, 0, PANEL_W, PANEL_H) != 0) {
        ctx->direct_pending = 0;
        ctx->direct_shadow = 0;
        return;
    }
    if (rl) rl->direct_submitted = 1;
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

/* ---- RLE over 4bpp pixels (the inner layer of modes 3 and 6) ----------------
 *
 * See the RLE paragraph at the top of the file for the token format. The decoder is a
 * byte-at-a-time state machine so it can be driven straight from inflate's output in
 * small chunks (a token may straddle a chunk boundary), writing pixels into a
 * rectangular 4bpp destination: `rows` rows of `rowbytes` bytes, row r at
 * base + r*stride. For mode 6 that's the whole shadow; for mode 3 it's the box within
 * it (rowbytes < stride). Runs cross rows freely — the nibble stream is the wire
 * order, not per-row.
 *
 * `left` counts the nibbles still expected; the decode is complete only when it hits
 * exactly 0 with no token half-parsed. Overrunning it (or running past the last row)
 * sets `err` and the caller drops the frame. */
typedef struct {
    uint8_t *base;
    uint32_t stride;      /* bytes between rows in the destination */
    uint32_t rowbytes;    /* bytes per row actually written */
    uint32_t rows;
    uint32_t r;           /* current row */
    uint32_t bpos;        /* byte offset within the current row */
    uint32_t hi;          /* 1 = next nibble is the high (left) one */
    uint32_t left;        /* nibbles still expected */
    uint32_t st;          /* token parser: 0 = opcode, 1 = cnt8, 2 = cntLo, 3 = cntHi */
    uint32_t cnt;
    uint8_t  color;
    uint8_t  err;
} rle_state;

static void rle_init(rle_state *s, uint8_t *base, uint32_t stride,
                     uint32_t rowbytes, uint32_t rows) {
    s->base = base; s->stride = stride; s->rowbytes = rowbytes; s->rows = rows;
    s->r = 0; s->bpos = 0; s->hi = 1;
    s->left = rowbytes * rows * 2;
    s->st = 0; s->cnt = 0; s->color = 0; s->err = 0;
}

/* Write `n` pixels of color `v`. Aligned whole-byte spans are filled a byte at a time
 * (both nibbles at once, v*0x11); only a leading/trailing odd nibble is read-modify-
 * written. */
static void rle_emit(rle_state *s, uint8_t v, uint32_t n) {
    if (n > s->left) { s->err = 1; return; }        /* overruns the frame -> malformed */
    s->left -= n;
    uint8_t pair = (uint8_t)(v * 0x11u);
    while (n) {
        if (s->r >= s->rows) { s->err = 1; return; }
        uint8_t *row = s->base + s->r * s->stride;
        if (!s->hi) {                                /* finish the byte we're inside */
            row[s->bpos] = (uint8_t)((row[s->bpos] & 0xf0u) | v);
            s->bpos++; s->hi = 1; n--;
        } else {
            uint32_t avail = s->rowbytes - s->bpos;  /* whole bytes left in this row */
            uint32_t bytes = n >> 1;
            if (bytes > avail) bytes = avail;
            for (uint32_t i = 0; i < bytes; i++) row[s->bpos + i] = pair;
            s->bpos += bytes; n -= bytes * 2;
            if (n && s->bpos < s->rowbytes) {        /* odd tail: open the next byte */
                row[s->bpos] = (uint8_t)((row[s->bpos] & 0x0fu) | (uint8_t)(v << 4));
                s->hi = 0; n--;
            }
        }
        if (s->hi && s->bpos >= s->rowbytes) { s->bpos = 0; s->r++; }   /* next row */
    }
}

/* Feed `n` bytes of RLE stream. Stops early (leaving err set) on a malformed stream. */
static void rle_feed(rle_state *s, const uint8_t *p, uint32_t n) {
    for (uint32_t i = 0; i < n && !s->err; i++) {
        uint8_t b = p[i];
        if (s->st == 0) {
            s->color = (uint8_t)(b & 0x0fu);
            uint32_t c = b >> 4;
            if (c) rle_emit(s, s->color, c);         /* short form: count in the high nibble */
            else s->st = 1;                          /* escape: count follows */
        } else if (s->st == 1) {
            if (b) { rle_emit(s, s->color, b); s->st = 0; }
            else s->st = 2;                          /* second escape: 16-bit count follows */
        } else if (s->st == 2) {
            s->cnt = b; s->st = 3;
        } else {
            s->cnt |= (uint32_t)b << 8;
            if (s->cnt == 0) s->err = 1;             /* a 0-length run can't be encoded */
            else rle_emit(s, s->color, s->cnt);
            s->st = 0;
        }
    }
}

/* Inflate an already-primed z_stream (NEXT_IN/AVAIL_IN set by the caller) and RLE-decode
 * its output into the rectangular 4bpp destination, streaming through a stack chunk so
 * no scratch buffer is allocated for either layer. Returns 1 only when the zlib stream
 * ends AND the RLE stream filled the destination exactly. */
static int inflate_rle(uint8_t *strm, uint8_t *base, uint32_t stride,
                       uint32_t rowbytes, uint32_t rows) {
    if (FW_INIT2(strm, 15, ZLIB_VER, ZS_SIZE) != 0) { FW_END(strm); return 0; }
    rle_state rs;
    rle_init(&rs, base, stride, rowbytes, rows);
    uint8_t chunk[RLE_CHUNK];
    int ok = 0;
    for (;;) {
        *(uint8_t **)(strm + ZS_NEXT_OUT) = chunk;
        *(uint32_t *)(strm + ZS_AVAIL_OUT) = RLE_CHUNK;
        int r = FW_INFLATE(strm, 0);                 /* Z_NO_FLUSH */
        uint32_t got = (uint32_t)(*(uint8_t **)(strm + ZS_NEXT_OUT) - chunk);
        rle_feed(&rs, chunk, got);
        if (rs.err) break;                           /* malformed RLE */
        if (r == 1) { ok = (rs.left == 0 && rs.st == 0); break; }   /* Z_STREAM_END */
        if (r != 0 || got == 0) break;               /* inflate error, or no progress */
    }
    FW_END(strm);
    return ok;
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
    /* A legacy BMP deliberately hands presentation back to LVGL/the stock
     * compositor, so subsequent widget repaints must not preserve a prior direct
     * frame even if Faceclaw's ownership lease is still alive. */
    customCfwContext *ctx = peekCustomCfwContext();
    if (ctx) ctx->direct_active = 0;

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

/* Wrapper for the one global display-dispatch call handling sensor event 9 /
 * UI event 0x41 (IMU_COMPASS_DIRECTION). The stock call is always preserved.
 * Navigation normally consumes this event and invokes FW_COMPASS_NOTIFY itself,
 * but its handler is absent while EvenHub/Faceclaw is active. Mode 10 marks the
 * CFW context so we invoke that same stock notifier directly with the already-
 * computed heading. This runs only on the right arm and allocates no CFW state. */
int compass_event_forward(uint32_t display, uint32_t event, void *value) {
    int r = FW_DISPLAY_EVENT_FORWARD(display, event, value);
    customCfwContext *ctx = peekCustomCfwContext();
    if (event == 0x41 && value != 0 && FW_SIDE() == 1 && ctx && ctx->compass_forward) {
        int32_t heading = *(int32_t *)value;
        if (heading >= 0) FW_COMPASS_NOTIFY((uint32_t)heading);
    }
    return r;
}

/* Return the full-panel packed-4bpp shadow stored in this container's display
 * allocation A. The 576x288 carrier allocates 165888 bytes for A; the 640x480
 * shadow needs 153600, leaving 12288 bytes unused. Buffer B is an independent,
 * same-sized allocation shared by live reconstruction at its head and snapshots
 * packed into its tail. */
static uint8_t *cfw_shadow_buffer(uint8_t *state) {
    uint8_t *a = *(uint8_t **)(state + 0x8);
    if (a == 0) return 0;
    uint32_t carrier_w = *(uint16_t *)(state + 0x40);
    uint32_t carrier_h = *(uint16_t *)(state + 0x42);
    uint32_t capacity = carrier_w * carrier_h;
    if (capacity < IMAGE_BYTES) return 0;
    return a;
}

static void bzero(uint8_t *buf, uint32_t len) {
    for (uint32_t i = 0; i < len; i++) buf[i] = 0;
}

/* Return the singleton to its stock-compatible idle state without freeing it.
 * Idempotent: successfully deleted timer handles and released snapshot slots are
 * cleared immediately, while a timer whose delete command fails remains in the
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

    /* Suppress callbacks before asking the timer service to stop/delete them;
     * a callback already dispatched on the timer thread will then be harmless. */
    ctx->seq_count = 0;
    ctx->seq_cursor = 0;
    if (ctx->seq_timer) {
        FW_TIMER_STOP(ctx->seq_timer);
        if (FW_TIMER_DELETE(ctx->seq_timer) == 0) ctx->seq_timer = 0;
    }
    FW_BUZZ_RESET();

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

    for (uint32_t i = 0; i < CFW_SNAP_RING; i++) {
        cfw_snap_clear(&ctx->snaps[i]);
    }
    ctx->snap_seq = 0;

    /* Diagnostics are inert while hidden. Keep their sticky history for later
     * inspection, but make sure no Faceclaw overlay reaches the stock session. */
    ctx->diag_hide = 1;

    if (launch_dashboard) FW_APP_START(1, 0, 0, 0);
    return 0;
}

static void copy_panel(uint8_t *fb, const uint8_t *shadow) {
    uint32_t *dst = (uint32_t *)(void *)fb;
    const uint32_t *src = (const uint32_t *)(const void *)shadow;
    for (uint32_t i = 0; i < PANEL_BYTES / 4u; i++) dst[i] = src[i];
}

/* Replaces both display-task calls to the stock 576x288 packed-buffer copier.
 * A pending custom job copies the full 640x480 shadow straight into the
 * physical 640x480 4bpp framebuffer. Once that succeeds, unrelated stock widget
 * repaints are suppressed while Faceclaw's fail-open framebuffer lease is valid:
 * the display task refreshes the already-correct physical buffer instead of
 * overwriting it with stale LVGL content. Lease release/expiry and legacy BMP
 * presentation restore the transparent stock pass-through. */
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
        copy_panel(fb, shadow);
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

/* ---- Terminus 6x12 bitmap font + text overlay ------------------------------
 *
 * Printable ASCII 32..126 from faceclaw/app/fonts/terminus/ter-u12n.bdf, indexed
 * by (ch - FONT_FIRST). One glyph is 12 rows x 6 columns: one byte per row with the
 * six pixels in the top six bits (bit 7 = leftmost column) — the raw BDF layout.
 * 95*12 = 1140 bytes of read-only data, carried inside the injected blob thanks to
 * build.py's -fropi rodata support (string/table literals resolve PC-relative, so
 * no linker/loader fixups are needed). */
#define FONT_W 6
#define FONT_H 12
#define FONT_FIRST 32
#define FONT_LAST  126
static const unsigned char font6x12[FONT_LAST - FONT_FIRST + 1][FONT_H] = {
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  /*  32 ' ' */
    { 0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x20, 0x20, 0x00, 0x00 },  /*  33 '!' */
    { 0x00, 0x50, 0x50, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  /*  34 '"' */
    { 0x00, 0x00, 0x50, 0x50, 0xf8, 0x50, 0x50, 0xf8, 0x50, 0x50, 0x00, 0x00 },  /*  35 '#' */
    { 0x00, 0x00, 0x20, 0x70, 0xa8, 0xa0, 0x70, 0x28, 0xa8, 0x70, 0x20, 0x00 },  /*  36 '$' */
    { 0x00, 0x00, 0x48, 0xa8, 0x50, 0x10, 0x20, 0x28, 0x54, 0x48, 0x00, 0x00 },  /*  37 '%' */
    { 0x00, 0x00, 0x20, 0x50, 0x50, 0x20, 0x68, 0x90, 0x90, 0x68, 0x00, 0x00 },  /*  38 '&' */
    { 0x00, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  /*  39 "'" */
    { 0x00, 0x00, 0x10, 0x20, 0x40, 0x40, 0x40, 0x40, 0x20, 0x10, 0x00, 0x00 },  /*  40 '(' */
    { 0x00, 0x00, 0x40, 0x20, 0x10, 0x10, 0x10, 0x10, 0x20, 0x40, 0x00, 0x00 },  /*  41 ')' */
    { 0x00, 0x00, 0x00, 0x00, 0x50, 0x20, 0xf8, 0x20, 0x50, 0x00, 0x00, 0x00 },  /*  42 '*' */
    { 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0xf8, 0x20, 0x20, 0x00, 0x00, 0x00 },  /*  43 '+' */
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x40, 0x00 },  /*  44 ',' */
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00 },  /*  45 '-' */
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00 },  /*  46 '.' */
    { 0x00, 0x00, 0x08, 0x08, 0x10, 0x10, 0x20, 0x20, 0x40, 0x40, 0x00, 0x00 },  /*  47 '/' */
    { 0x00, 0x00, 0x70, 0x88, 0x98, 0xa8, 0xc8, 0x88, 0x88, 0x70, 0x00, 0x00 },  /*  48 '0' */
    { 0x00, 0x00, 0x20, 0x60, 0x20, 0x20, 0x20, 0x20, 0x20, 0x70, 0x00, 0x00 },  /*  49 '1' */
    { 0x00, 0x00, 0x70, 0x88, 0x88, 0x08, 0x10, 0x20, 0x40, 0xf8, 0x00, 0x00 },  /*  50 '2' */
    { 0x00, 0x00, 0x70, 0x88, 0x08, 0x30, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00 },  /*  51 '3' */
    { 0x00, 0x00, 0x08, 0x18, 0x28, 0x48, 0x88, 0xf8, 0x08, 0x08, 0x00, 0x00 },  /*  52 '4' */
    { 0x00, 0x00, 0xf8, 0x80, 0x80, 0xf0, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00 },  /*  53 '5' */
    { 0x00, 0x00, 0x70, 0x80, 0x80, 0xf0, 0x88, 0x88, 0x88, 0x70, 0x00, 0x00 },  /*  54 '6' */
    { 0x00, 0x00, 0xf8, 0x08, 0x08, 0x10, 0x10, 0x20, 0x20, 0x20, 0x00, 0x00 },  /*  55 '7' */
    { 0x00, 0x00, 0x70, 0x88, 0x88, 0x70, 0x88, 0x88, 0x88, 0x70, 0x00, 0x00 },  /*  56 '8' */
    { 0x00, 0x00, 0x70, 0x88, 0x88, 0x88, 0x78, 0x08, 0x08, 0x70, 0x00, 0x00 },  /*  57 '9' */
    { 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00 },  /*  58 ':' */
    { 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x20, 0x20, 0x40, 0x00 },  /*  59 ';' */
    { 0x00, 0x00, 0x00, 0x08, 0x10, 0x20, 0x40, 0x20, 0x10, 0x08, 0x00, 0x00 },  /*  60 '<' */
    { 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x00, 0x00 },  /*  61 '=' */
    { 0x00, 0x00, 0x00, 0x40, 0x20, 0x10, 0x08, 0x10, 0x20, 0x40, 0x00, 0x00 },  /*  62 '>' */
    { 0x00, 0x00, 0x70, 0x88, 0x88, 0x10, 0x20, 0x00, 0x20, 0x20, 0x00, 0x00 },  /*  63 '?' */
    { 0x00, 0x00, 0x70, 0x88, 0x98, 0xa8, 0xa8, 0x98, 0x80, 0x78, 0x00, 0x00 },  /*  64 '@' */
    { 0x00, 0x00, 0x70, 0x88, 0x88, 0x88, 0xf8, 0x88, 0x88, 0x88, 0x00, 0x00 },  /*  65 'A' */
    { 0x00, 0x00, 0xf0, 0x88, 0x88, 0xf0, 0x88, 0x88, 0x88, 0xf0, 0x00, 0x00 },  /*  66 'B' */
    { 0x00, 0x00, 0x70, 0x88, 0x80, 0x80, 0x80, 0x80, 0x88, 0x70, 0x00, 0x00 },  /*  67 'C' */
    { 0x00, 0x00, 0xe0, 0x90, 0x88, 0x88, 0x88, 0x88, 0x90, 0xe0, 0x00, 0x00 },  /*  68 'D' */
    { 0x00, 0x00, 0xf8, 0x80, 0x80, 0xf0, 0x80, 0x80, 0x80, 0xf8, 0x00, 0x00 },  /*  69 'E' */
    { 0x00, 0x00, 0xf8, 0x80, 0x80, 0xf0, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 },  /*  70 'F' */
    { 0x00, 0x00, 0x70, 0x88, 0x80, 0x80, 0xb8, 0x88, 0x88, 0x70, 0x00, 0x00 },  /*  71 'G' */
    { 0x00, 0x00, 0x88, 0x88, 0x88, 0xf8, 0x88, 0x88, 0x88, 0x88, 0x00, 0x00 },  /*  72 'H' */
    { 0x00, 0x00, 0x70, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x70, 0x00, 0x00 },  /*  73 'I' */
    { 0x00, 0x00, 0x38, 0x10, 0x10, 0x10, 0x10, 0x90, 0x90, 0x60, 0x00, 0x00 },  /*  74 'J' */
    { 0x00, 0x00, 0x88, 0x90, 0xa0, 0xc0, 0xc0, 0xa0, 0x90, 0x88, 0x00, 0x00 },  /*  75 'K' */
    { 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xf8, 0x00, 0x00 },  /*  76 'L' */
    { 0x00, 0x00, 0x88, 0xd8, 0xa8, 0xa8, 0x88, 0x88, 0x88, 0x88, 0x00, 0x00 },  /*  77 'M' */
    { 0x00, 0x00, 0x88, 0x88, 0xc8, 0xa8, 0x98, 0x88, 0x88, 0x88, 0x00, 0x00 },  /*  78 'N' */
    { 0x00, 0x00, 0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00, 0x00 },  /*  79 'O' */
    { 0x00, 0x00, 0xf0, 0x88, 0x88, 0x88, 0xf0, 0x80, 0x80, 0x80, 0x00, 0x00 },  /*  80 'P' */
    { 0x00, 0x00, 0x70, 0x88, 0x88, 0x88, 0x88, 0x88, 0xa8, 0x70, 0x08, 0x00 },  /*  81 'Q' */
    { 0x00, 0x00, 0xf0, 0x88, 0x88, 0x88, 0xf0, 0xa0, 0x90, 0x88, 0x00, 0x00 },  /*  82 'R' */
    { 0x00, 0x00, 0x70, 0x88, 0x80, 0x70, 0x08, 0x08, 0x88, 0x70, 0x00, 0x00 },  /*  83 'S' */
    { 0x00, 0x00, 0xf8, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00 },  /*  84 'T' */
    { 0x00, 0x00, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00, 0x00 },  /*  85 'U' */
    { 0x00, 0x00, 0x88, 0x88, 0x88, 0x50, 0x50, 0x50, 0x20, 0x20, 0x00, 0x00 },  /*  86 'V' */
    { 0x00, 0x00, 0x88, 0x88, 0x88, 0x88, 0xa8, 0xa8, 0xd8, 0x88, 0x00, 0x00 },  /*  87 'W' */
    { 0x00, 0x00, 0x88, 0x88, 0x50, 0x20, 0x20, 0x50, 0x88, 0x88, 0x00, 0x00 },  /*  88 'X' */
    { 0x00, 0x00, 0x88, 0x88, 0x50, 0x50, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00 },  /*  89 'Y' */
    { 0x00, 0x00, 0xf8, 0x08, 0x10, 0x20, 0x40, 0x80, 0x80, 0xf8, 0x00, 0x00 },  /*  90 'Z' */
    { 0x00, 0x00, 0x70, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x70, 0x00, 0x00 },  /*  91 '[' */
    { 0x00, 0x00, 0x40, 0x40, 0x20, 0x20, 0x10, 0x10, 0x08, 0x08, 0x00, 0x00 },  /*  92 '\\' */
    { 0x00, 0x00, 0x70, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x70, 0x00, 0x00 },  /*  93 ']' */
    { 0x00, 0x20, 0x50, 0x88, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  /*  94 '^' */
    { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x00 },  /*  95 '_' */
    { 0x40, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  /*  96 '`' */
    { 0x00, 0x00, 0x00, 0x00, 0x70, 0x08, 0x78, 0x88, 0x88, 0x78, 0x00, 0x00 },  /*  97 'a' */
    { 0x00, 0x00, 0x80, 0x80, 0xf0, 0x88, 0x88, 0x88, 0x88, 0xf0, 0x00, 0x00 },  /*  98 'b' */
    { 0x00, 0x00, 0x00, 0x00, 0x70, 0x88, 0x80, 0x80, 0x88, 0x70, 0x00, 0x00 },  /*  99 'c' */
    { 0x00, 0x00, 0x08, 0x08, 0x78, 0x88, 0x88, 0x88, 0x88, 0x78, 0x00, 0x00 },  /* 100 'd' */
    { 0x00, 0x00, 0x00, 0x00, 0x70, 0x88, 0xf8, 0x80, 0x80, 0x78, 0x00, 0x00 },  /* 101 'e' */
    { 0x00, 0x00, 0x18, 0x20, 0x70, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00 },  /* 102 'f' */
    { 0x00, 0x00, 0x00, 0x00, 0x78, 0x88, 0x88, 0x88, 0x88, 0x78, 0x08, 0x70 },  /* 103 'g' */
    { 0x00, 0x00, 0x80, 0x80, 0xf0, 0x88, 0x88, 0x88, 0x88, 0x88, 0x00, 0x00 },  /* 104 'h' */
    { 0x00, 0x20, 0x20, 0x00, 0x60, 0x20, 0x20, 0x20, 0x20, 0x70, 0x00, 0x00 },  /* 105 'i' */
    { 0x00, 0x08, 0x08, 0x00, 0x18, 0x08, 0x08, 0x08, 0x08, 0x08, 0x48, 0x30 },  /* 106 'j' */
    { 0x00, 0x00, 0x40, 0x40, 0x48, 0x50, 0x60, 0x60, 0x50, 0x48, 0x00, 0x00 },  /* 107 'k' */
    { 0x00, 0x00, 0x60, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x70, 0x00, 0x00 },  /* 108 'l' */
    { 0x00, 0x00, 0x00, 0x00, 0xf0, 0xa8, 0xa8, 0xa8, 0xa8, 0xa8, 0x00, 0x00 },  /* 109 'm' */
    { 0x00, 0x00, 0x00, 0x00, 0xf0, 0x88, 0x88, 0x88, 0x88, 0x88, 0x00, 0x00 },  /* 110 'n' */
    { 0x00, 0x00, 0x00, 0x00, 0x70, 0x88, 0x88, 0x88, 0x88, 0x70, 0x00, 0x00 },  /* 111 'o' */
    { 0x00, 0x00, 0x00, 0x00, 0xf0, 0x88, 0x88, 0x88, 0x88, 0xf0, 0x80, 0x80 },  /* 112 'p' */
    { 0x00, 0x00, 0x00, 0x00, 0x78, 0x88, 0x88, 0x88, 0x88, 0x78, 0x08, 0x08 },  /* 113 'q' */
    { 0x00, 0x00, 0x00, 0x00, 0xb8, 0xc0, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00 },  /* 114 'r' */
    { 0x00, 0x00, 0x00, 0x00, 0x78, 0x80, 0x70, 0x08, 0x08, 0xf0, 0x00, 0x00 },  /* 115 's' */
    { 0x00, 0x00, 0x20, 0x20, 0x70, 0x20, 0x20, 0x20, 0x20, 0x18, 0x00, 0x00 },  /* 116 't' */
    { 0x00, 0x00, 0x00, 0x00, 0x88, 0x88, 0x88, 0x88, 0x88, 0x78, 0x00, 0x00 },  /* 117 'u' */
    { 0x00, 0x00, 0x00, 0x00, 0x88, 0x88, 0x50, 0x50, 0x20, 0x20, 0x00, 0x00 },  /* 118 'v' */
    { 0x00, 0x00, 0x00, 0x00, 0x88, 0x88, 0xa8, 0xa8, 0xa8, 0x70, 0x00, 0x00 },  /* 119 'w' */
    { 0x00, 0x00, 0x00, 0x00, 0x88, 0x50, 0x20, 0x20, 0x50, 0x88, 0x00, 0x00 },  /* 120 'x' */
    { 0x00, 0x00, 0x00, 0x00, 0x88, 0x88, 0x88, 0x88, 0x88, 0x78, 0x08, 0x70 },  /* 121 'y' */
    { 0x00, 0x00, 0x00, 0x00, 0xf8, 0x10, 0x20, 0x40, 0x80, 0xf8, 0x00, 0x00 },  /* 122 'z' */
    { 0x00, 0x00, 0x18, 0x20, 0x20, 0x40, 0x20, 0x20, 0x20, 0x18, 0x00, 0x00 },  /* 123 '{' */
    { 0x00, 0x00, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00 },  /* 124 '|' */
    { 0x00, 0x00, 0x60, 0x10, 0x10, 0x08, 0x10, 0x10, 0x10, 0x60, 0x00, 0x00 },  /* 125 '}' */
    { 0x00, 0x48, 0xa8, 0x90, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 },  /* 126 '~' */
};

/* Set one pixel in the physical packed-4bpp framebuffer. */
static void set_pixel4(uint8_t *disp, uint32_t w, uint32_t x, uint32_t y, uint8_t v) {
    uint8_t *p = disp + y * ((w + 1u) >> 1) + (x >> 1);
    if ((x & 1u) == 0) *p = (uint8_t)((*p & 0x0fu) | ((v & 0x0fu) << 4));
    else                *p = (uint8_t)((*p & 0xf0u) |  (v & 0x0fu));
}

/* Draw one glyph's set pixels at (x0,y0) in color `fg` (4bpp nibble); unset pixels
 * are left untouched. Fully clipped to the packed w*h buffer; a non-printable char draws
 * nothing. */
static void draw_glyph(uint8_t *disp, uint32_t w, uint32_t h,
                       int x0, int y0, char ch, uint8_t fg) {
    unsigned uc = (unsigned char)ch;
    if (uc < FONT_FIRST || uc > FONT_LAST) return;
    const unsigned char *g = font6x12[uc - FONT_FIRST];
    for (int ry = 0; ry < FONT_H; ry++) {
        int y = y0 + ry;
        if (y < 0 || (uint32_t)y >= h) continue;
        unsigned bits = g[ry];
        for (int rx = 0; rx < FONT_W; rx++) {
            int x = x0 + rx;
            if ((bits & (0x80u >> rx)) && x >= 0 && (uint32_t)x < w)
                set_pixel4(disp, w, (uint32_t)x, (uint32_t)y, fg);
        }
    }
}

/* Draw a NUL-terminated ASCII string at (x0,y0), advancing FONT_W per char, in
 * color `fg` (4bpp nibble). If bg >= 0, first fill a 1px-padded background box
 * (font height x string width) in color `bg` so the text stays legible over any
 * underlying image. */
static void draw_string(uint8_t *disp, uint32_t w, uint32_t h,
                        int x0, int y0, const char *s, uint8_t fg, int bg) {
    int len = 0;
    for (const char *p = s; *p; p++) len++;
    if (bg >= 0) {
        for (int ry = -1; ry <= FONT_H; ry++) {
            int y = y0 + ry;
            if (y < 0 || (uint32_t)y >= h) continue;
            for (int rx = -1; rx <= len * FONT_W; rx++) {
                int x = x0 + rx;
                if (x >= 0 && (uint32_t)x < w)
                    set_pixel4(disp, w, (uint32_t)x, (uint32_t)y, (uint8_t)bg);
            }
        }
    }
    int x = x0;
    for (const char *p = s; *p; p++, x += FONT_W)
        draw_glyph(disp, w, h, x, y0, *p, fg);
}

// Convert an unsigned int to string, and append it to a string. Truncated if
// the resulting combined string is longer than maxlen.
static void u_to_dec(char *out, uint32_t v, uint32_t maxlen) {
    uint32_t pos = strnlen(out, maxlen);
    uint32_t div = 1;
    while (v / div >= 10) div *= 10;   // largest power of 10 <= v
    do {
        if (pos >= maxlen-1) break;
        out[pos++] = (char)('0' + (v / div));
        v %= div; div /= 10;
    } while (div);
    out[pos] = 0;
}

/* The stock TLSF build is the 32-bit, 4-byte-aligned configuration. A pool made
 * by tlsf_create_with_pool() starts after its 0xc74-byte control structure. Its
 * physical block chain has a size/status word at the pool address, then another
 * size/status word every (block size + 4) bytes, and ends with a zero-size word
 * at arena_end - 4. Sum the payload capacity of free blocks, validating every
 * step so an uninitialized pool or a concurrent split/coalesce produces "?"
 * instead of an out-of-arena read or a bogus free-space value.
 *
 * TLSF is not itself thread-safe. These diagnostics deliberately do not take the
 * allocator mutexes: they run in the display path, must not stall it, and a
 * validated approximate snapshot is preferable to introducing lock ordering
 * into that path. The aligned 32-bit metadata reads are atomic on this core. */
#define TLSF_CONTROL_BYTES 0x0c74U
#define TLSF_BLOCK_MIN     12U
#define TLSF_FREE_INVALID  0xffffffffU

static uint32_t tlsf_arena_free(uint32_t arena, uint32_t arena_size) {
    uint32_t arena_end = arena + arena_size;
    if ((arena & 3u) || arena_end < arena || arena_size < TLSF_CONTROL_BYTES + 8u)
        return TLSF_FREE_INVALID;

    uint32_t size_word = arena + TLSF_CONTROL_BYTES;
    uint32_t free_bytes = 0;
    uint32_t max_blocks = (arena_size - TLSF_CONTROL_BYTES - 4u) / 16u + 1u;
    for (uint32_t n = 0; n < max_blocks; n++) {
        if ((size_word & 3u) || size_word > arena_end - 4u)
            return TLSF_FREE_INVALID;

        uint32_t size_flags = *(volatile uint32_t *)(uintptr_t)size_word;
        uint32_t block_size = size_flags & ~3u;
        if (block_size == 0)
            return size_word == arena_end - 4u ? free_bytes : TLSF_FREE_INVALID;
        if (size_word > arena_end - 8u || block_size < TLSF_BLOCK_MIN ||
            block_size > arena_end - size_word - 8u)
            return TLSF_FREE_INVALID;

        if (size_flags & 1u) free_bytes += block_size;
        size_word += block_size + 4u;
    }
    return TLSF_FREE_INVALID;
}

/* Generic heap descriptors made by FUN_0048413c contain their TLSF pointer at
 * +4, arena size at +0x10, and arena base at +0x14. Check all three before
 * trusting the pool. The policy byte at +0x18 belongs to the higher selector. */
static uint32_t heap_object_free(uint32_t descriptor, uint32_t arena,
                                 uint32_t arena_size) {
    volatile uint32_t *heap = (volatile uint32_t *)(uintptr_t)descriptor;
    if (heap[1] != arena || heap[4] != arena_size || heap[5] != arena)
        return TLSF_FREE_INVALID;
    return tlsf_arena_free(arena, arena_size);
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
 * 0x2013be70, 0x20208e70, and 0x20279670 respectively. Suppressed when diag_hide is set
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

    uint32_t free_13 = heap_object_free(0x20000354u, 0x2013be70u, 0x000cd000u);
    uint32_t free_20 =
        *(volatile uint32_t *)0x20074abcu == 0x20208e70u
            ? tlsf_arena_free(0x20208e70u, 0x00070800u)
            : TLSF_FREE_INVALID;
    /* The stock 0x20000338 descriptor is initialized with 0x2d000 bytes. The
     * CFW patch reduces it to 0x2cc00, reserving the final 1 KiB for CFW state. */
    uint32_t free_27 = heap_object_free(0x20000338u, 0x20279670u, 0x0002cc00u);
    strlcat(line, " f13/20/27=", sizeof(line));
    append_free_kib(line, free_13, sizeof(line));
    strlcat(line, "/", sizeof(line));
    append_free_kib(line, free_20, sizeof(line));
    strlcat(line, "/", sizeof(line));
    append_free_kib(line, free_27, sizeof(line));
    strlcat(line, "k", sizeof(line));

    draw_string(disp, w, h, IMAGE_X + 2, IMAGE_Y + 2, line, 15, 0);
}

static uint32_t strlcpy(char *dst, const char *src, uint32_t len) {
  if (len == 0) {
      return 0;
  }
  for (uint32_t i=0; i<len; i++) {
    dst[i] = src[i];
    if (src[i] == 0) {
      return i;
    }
  }
  dst[len-1] = 0;
  return len;
}

static uint32_t strnlen(const char *s, uint32_t maxlen) {
  for (uint32_t i=0; i<maxlen; i++) {
    if (s[i] == 0) {
      return i;
    }
  }
  return maxlen;
}

static uint32_t strlcat(char *dst, const char *src, uint32_t len) {
    uint32_t i = strnlen(dst, len), j = 0;
    for (; i < len; i++, j++) {
        dst[i] = src[j];
        if (src[j] == 0) return i;
    }
    if (len) dst[len-1] = 0;
    return len;
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
 * per-state FIFO (snapshot_side, hooked at the both-lens `bl FUN_0045a8ec`). Snapshot
 * bytes are packed from the END of that state's oversized reconstruction allocation;
 * the beginning remains the stock receiver's live assembly area. The deferred handler
 * (image_deferred, both lenses) consumes the oldest tail range, never the overwritten
 * live prefix. Both lenses do identical work on identical data, preserving sync. */

static void cfw_snap_clear(cfw_snap *snap) {
    snap->state = 0;
    snap->buf = 0;
    snap->len = 0;
    snap->seq = 0;
}

/* Find a len-byte hole as high as possible in B, avoiding all queued ranges for
 * this reconstruction buffer. The new range may overlap the just-completed live
 * prefix: cfw_snapshot uses a backward copy in that case, like memmove. A later
 * reconstruction that reaches a queued range invalidates it below. */
static uint8_t *cfw_snap_tail_alloc(customCfwContext *ctx, uint8_t *state,
                                    uint8_t *b, uint32_t capacity,
                                    uint32_t live_len, uint32_t len) {
    uintptr_t base = (uintptr_t)b;
    uintptr_t live_end = base + live_len;
    uintptr_t end = base + capacity;

    for (uint32_t i = 0; i < CFW_SNAP_RING; i++) {
        cfw_snap *snap = &ctx->snaps[i];
        if (snap->state != state) continue;
        uintptr_t start = (uintptr_t)snap->buf;
        if (start < base || start > end || snap->len > end - start) {
            if (snap->seq != CFW_SNAP_BUSY_SEQ) cfw_snap_clear(snap);
            continue;
        }
        /* The stock receiver has already written the live prefix. If it reached
         * a queued tail range, that old frame is no longer usable. */
        if (start < live_end && snap->seq != CFW_SNAP_BUSY_SEQ) {
            cfw_snap_clear(snap);
            ctx->f_snap_of = 1;
        }
    }

    uintptr_t ceiling = end;
    while (ceiling >= base && len <= ceiling - base) {
        uintptr_t candidate = ceiling - len;
        uintptr_t below = ceiling;
        int overlap = 0;
        for (uint32_t i = 0; i < CFW_SNAP_RING; i++) {
            cfw_snap *snap = &ctx->snaps[i];
            if (snap->state != state || snap->buf == 0) continue;
            uintptr_t start = (uintptr_t)snap->buf;
            uintptr_t finish = start + snap->len;
            if (candidate < finish && start < ceiling) {
                if (start < below) below = start;
                overlap = 1;
            }
        }
        if (!overlap) return (uint8_t *)candidate;
        ceiling = below;
    }
    return 0;
}

/* Snapshot a just-completed CompressMode=0 message (B = *(state+0xc), len =
 * *(state+0x20), capacity = *(state+0x44)) into B's tail FIFO, then return the
 * lens id (real FUN_0045a8ec) so the caller's RIGHT gate still works.
 * Stock-compressed messages bypass this FIFO. Reached via snapshot_side, which
 * supplies r7/r8. */
int cfw_snapshot(uint8_t *state, uint32_t container_id) {
    (void)container_id;
    /* CompressMode is stock-owned on 2.2.6.10: mode 1/2 payloads are RLE/LZ4
     * and are decompressed by evenhub_ui immediately before image_deferred.
     * Do not snapshot their still-compressed reconstruction buffer here; the
     * deferred hook must consume the stock decoder's temporary `src` instead. */
    uint32_t compress_mode = state ? *(uint32_t *)(state + 0x18) : 0;
    customCfwContext *ctx = compress_mode == 0 ? getCustomCfwContext() : 0;
    if (ctx && state) {
        uint8_t *b = *(uint8_t **)(state + 0xc);
        uint32_t len = *(uint32_t *)(state + 0x20);
        if (b && len) {
            uint32_t capacity = *(uint32_t *)(state + 0x44);
            int slot = -1, oldest_i = -1;
            uint32_t oldest = 0xffffffffu;
            for (int i = 0; i < CFW_SNAP_RING; i++) {
                if (ctx->snaps[i].state == 0) { slot = i; break; }
                if (ctx->snaps[i].seq != CFW_SNAP_BUSY_SEQ &&
                    ctx->snaps[i].seq < oldest) {
                    oldest = ctx->snaps[i].seq; oldest_i = i;
                }
            }
            if (slot < 0 && oldest_i >= 0) {          /* full: evict globally-oldest */
                cfw_snap_clear(&ctx->snaps[oldest_i]);
                ctx->f_snap_of = 1;
                slot = oldest_i;
            }

            uint8_t *copy = 0;
            while (slot >= 0 && len <= capacity)
            {
                copy = cfw_snap_tail_alloc(ctx, state, b, capacity, len, len);
                if (copy) break;

                /* Make room by dropping this state's oldest queued (not busy)
                 * range. Other states have independent reconstruction buffers. */
                int victim = -1;
                oldest = 0xffffffffu;
                for (int i = 0; i < CFW_SNAP_RING; i++) {
                    if (ctx->snaps[i].state == state &&
                        ctx->snaps[i].seq != CFW_SNAP_BUSY_SEQ &&
                        ctx->snaps[i].seq < oldest) {
                        oldest = ctx->snaps[i].seq;
                        victim = i;
                    }
                }
                if (victim < 0) break;
                cfw_snap_clear(&ctx->snaps[victim]);
                ctx->f_snap_of = 1;
            }
            if (copy && slot >= 0) {
                /* Tail destinations can overlap the completed live prefix for a
                 * near-capacity message, so copy backward when it is higher. */
                if (copy > b) {
                    for (uint32_t i = len; i-- > 0; ) copy[i] = b[i];
                } else if (copy < b) {
                    for (uint32_t i = 0; i < len; i++) copy[i] = b[i];
                }
                ctx->snaps[slot].state = state;
                ctx->snaps[slot].buf = copy;
                ctx->snaps[slot].len = len;
                uint32_t seq = ctx->snap_seq++;
                if (seq == CFW_SNAP_BUSY_SEQ) seq = ctx->snap_seq++;
                ctx->snaps[slot].seq = seq;
            } else {
                ctx->f_snap_of = 1;
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

/* Replaces the deferred consumer's worker call (bl at 0x496a0e, both lenses). Stock-
 * compressed updates use the stock-decoded call arguments directly. Otherwise DRAINS
 * all of this lens's pending snapshots for `state` in FIFO (seq) order, running the
 * worker on each (ignoring the live B, which may be overwritten), then releases their
 * tail ranges. Draining all —
 * not just one — is required because the cross-lens timing sync can COALESCE several
 * completion messages into a single deferred call; handling only one would let the FIFO
 * fall arbitrarily far behind (-> ring overflow). If nothing is pending (a coalesced
 * extra call, whose frames were already drained), do nothing: NOT falling back to the
 * live buffer is what suppresses the spurious dup (that buffer was already shown via its
 * snapshot). Only if we have no context at all do we best-effort the live buffer. */
int image_deferred(uint8_t *state, uint8_t *src, uint32_t len) {
    /* Stock CompressMode 1/2 is decoded immediately before this hook, into a
     * temporary buffer passed as src/len. The earlier reconstruction snapshot
     * contains compressed bytes, so bypass the CFW FIFO and custom dispatcher
     * and preserve the exact stock set-image-data path. Treat any other nonzero
     * mode the same way: stock already decided whether to decode or use it raw. */
    if (state && *(uint32_t *)(state + 0x18) != 0)
        return FW_LOADBMP(state, src, len);

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
        ctx->snaps[slot].seq = CFW_SNAP_BUSY_SEQ;         /* keep its tail range reserved */
        r = image_worker(state, buf, blen);
        cfw_snap_clear(&ctx->snaps[slot]);                /* range is reusable now */
    }
    (void)src; (void)len;
    return r;                                             /* 0 if nothing pending (no dup) */
}
