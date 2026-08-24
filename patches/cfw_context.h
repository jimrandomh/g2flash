#pragma once
#include <stdint.h>

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
    /* Direct-framebuffer job. The EvenHub worker holds the stock display gate
     * before it mutates the shadow and until the display task consumes this
     * pointer, so no second snapshot or full-size display buffer is required. */
    const uint8_t *direct_shadow;
    volatile uint8_t direct_pending;
    uint8_t direct_failed;
    uint8_t direct_active;                    /* physical framebuffer currently owns the image */
    uint32_t direct_lease_deadline;            /* fail-open repaint-guard deadline */
    /* Phone-owned texture data, allocated lazily on the first mode-12 write and
     * released with the Faceclaw framebuffer lease. Protocol references into
     * this block are uint16 offsets. */
    uint8_t *texture_cache;
    /* --- Microphone control + multi-channel routing (SybilSight "glasses ->
     * microphones"). See the contract comment in mic_control.c; the stock-entry
     * recovery evidence lives in evenRealities-openCFW/g2/docs/research/
     * (g2-service-audio-recovery.md, g2-service-algo-recovery.md,
     * g2-production-mic-recovery.md). Config is advertised/read back over
     * sid-0x09 fields 103/104; capture + streaming are gated behind
     * MIC_FLAG_ARM_HW plus a fail-open renewal lease. Appended at the tail so
     * every existing field offset is unchanged. --- */
    uint8_t  mic_active;                    /* 1 = a CFW mic configuration is in effect */
    uint8_t  mic_source;                    /* 0 = codec DMIC/I2S, 1 = Ambiq PDM mics */
    uint8_t  mic_channels;                  /* requested channel count (1 = mono, 2 = dual) */
    uint8_t  mic_chan_mask;                 /* per-mic enable bitmask (bit0=front, bit1=rear) */
    uint8_t  mic_codec;                     /* requested: 0 = LC3 encoded, 1 = raw PCM passthrough */
    uint8_t  mic_format;                    /* PCM width: 0=16-bit, 1=24-bit, 2=32-bit */
    uint8_t  mic_flags;                     /* MIC_FLAG_* (beamform append, arm hardware) */
    uint8_t  mic_hw_armed;                  /* 1 = capture + tap are live */
    uint16_t mic_rate_hz_div;               /* requested sample rate, units of 100 Hz (160 = 16 kHz) */
    uint16_t mic_bitrate_100;               /* LC3 target bitrate, units of 100 bps (0 = default) */
    uint32_t mic_frames;                    /* stream frames emitted since session start */
    uint32_t mic_lease_deadline;            /* FW_MS_TICK streaming-lease deadline; 0 = none */
    uint32_t mic_watchdog_timer;            /* one-shot osTimer tearing down a lapsed session */
    uint8_t  mic_notify_buf[32];            /* stable storage for the field-104 sid-0x09 notify */
} customCfwContext;

#define CFW_CTX_SLOT  0x202a6270U    /* first word of the CFW-reserved TLSF tail */
#define CFW_ALLOC_DIAG_SLOT 0x202a6274U /* second word: magic | sticky failure bit */
#define CFW_ALLOC_DIAG_MAGIC 0xA110CA7EU
#define CFW_CTX_MAGIC 0xC0FFEE68U    /* bumped for the context layout change (mic fields) */

#define FW_MS_TICK  (*(volatile uint32_t *)0x20074a34U)  /* firmware 1 ms OS tick (SysTick chain) */

static customCfwContext *peekCustomCfwContext(void);
static customCfwContext *getCustomCfwContext(void);
int cfw_fb_lease_active(void);
