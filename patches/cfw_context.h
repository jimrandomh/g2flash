#pragma once
#include <stdint.h>

/* Persistent CFW-owned state that must survive image-container teardown/rebuild.
 * The image container (its display buffer A @ state+0x8 and recon buffer B @
 * state+0xc) is freed and reallocated on rebuild. The packed shadow lives in A
 * only for the lifetime of the current streaming layout and must be seeded by a
 * mode-6 keyframe after every rebuild. The bookkeeping that does
 * need to survive rebuilds is anchored by a pointer in 1 KiB of SRAM explicitly
 * removed from the top of the stock primary TLSF arena by patch_compress.py. The
 * stock arena is [0x202728a8,0x2029f8a8); the patched size is 0x2cc00, reserving
 * [0x2029f4a8,0x2029f8a8) for CFW. Its first word holds the context pointer and
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

/* Sidecar for a stock IMU ring record; written/read on the sensor-hub task. */
typedef struct {
    uint32_t timestamp;
    uint8_t accuracy, anomalies, source, flags;
} cfw_compass_sample;

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
    volatile uint8_t compass_forward;      /* mode 10: forward sensor-hub heading reports to BLE */
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
    uint32_t mic_settle_timer;              /* 2.2.10.36: one-shot osTimer completing a deferred bring-up */
    uint32_t mic_codec_ready_tick;          /* 2.2.10.36: FW_MS_TICK after which the power-cycled codec
                                             * has finished calibrating and may be tapped (0 = ready) */
    uint8_t  mic_settle_stage;              /* 2.2.10.37: 0 idle, 1 = waiting for chip boot, 2 = I2S
                                             * deinit issued, waiting to re-init on the ready chip */
    uint8_t  mic_notify_buf[32];            /* stable storage for the field-104 sid-0x09 notify */
    /* --- Ambient light sensor (mode 16, als_sensor.c). Passive mode redirects
     * the sensor-hub's ALS timer message to als_hub_handler through the RAM
     * dispatch table and polls the OPT3001 itself, so the stock adjuster never
     * steps the panel brightness. Appended at the tail. --- */
    uint8_t  als_hooked;                    /* hub message-8 entry currently points at als_hub_handler */
    uint8_t  als_opened_by_cfw;             /* the CFW opened the ALS (close it again on stop) */
    uint8_t  als_flags;                     /* ALS_START_FLAG_* from the start command */
    uint8_t  als_read_ok;                   /* last passive read succeeded */
    uint16_t als_interval_ms;               /* passive poll period (100..5000) */
    uint16_t als_min_delta;                 /* report when |value - last reported| >= this */
    uint16_t als_heartbeat_ms;              /* also report after this many ms (0 = never) */
    uint16_t mic_codec_cycled;              /* 2.2.10.35: 1 once the GX8002 voice codec has been
                                             * power-cycled this host boot (formerly als_reserved,
                                             * unused; same size and position). Survives BLE link
                                             * rebuilds, reset only by a host reboot. */
    uint32_t als_orig_handler;              /* stock hub handler for message 8 (Thumb address) */
    uint32_t als_last_reported;             /* value carried by the last report */
    uint32_t als_last_report_tick;          /* FW_MS_TICK of the last report (0 = none yet) */
    cfw_compass_sample compass_samples[20];
    /* Idle-input forwarding (settings_ext.c faceclaw_idle_input_gate): a
     * field-102 notify of its own, since wake_notify_buf may still be queued
     * for a deferred double-tap wake when a tap or release follows it. */
    uint8_t  gesture_notify_buf[16];
    /* 2.2.10.38: the context lives in reserved SRAM and survives warm resets AND firmware
     * re-flashes, and peek validated only a fixed magic — so a context created by one candidate
     * was reused, layout and all, by the next. ctx_size is stamped at creation and checked on
     * every peek: any layout change recreates the context instead of reading past the old one. */
    uint32_t ctx_size;                      /* == sizeof(customCfwContext) when this layout created it */
    /* 2.2.10.38: host-reboot detection. The GX8002 must be power-cycled once per HOST boot
     * (stock boot leaves it in the tap-noisy state), but the context survives warm resets, so
     * mic_codec_cycled alone would never re-run the cycle. The OS ms tick restarts at boot, and
     * the phone talks to the temple constantly (renew every 30 s, status, arm); if the tick is
     * ever LOWER than at the previous control op, the host rebooted in between → clear the flag. */
    uint32_t mic_last_seen_tick;            /* FW_MS_TICK at the last mic control op */
    uint32_t mic_layout_rev;                /* bump to force a fresh context after a re-flash whose
                                             * struct size happens to match the old one. NOTE: the
                                             * OS tick does not reset across the OTA reboot, so the
                                             * tick-backwards host-reboot detector never fires and
                                             * the once-per-boot gate stays stuck after the first
                                             * cycle; bumping the layout each candidate is the
                                             * reliable way to re-run the cycle in testing. A
                                             * production once-per-boot reset needs a real
                                             * cold-boot signal (a startup hook), filed separately. */
    uint32_t mic_layout_rev2;               /* 2.2.10.47: layout bump */
    uint32_t mic_layout_rev3;               /* 2.2.10.48: layout bump (fresh context) */
    /* 2.2.10.49: dirty-rectangle present. present_shadow records the union of the frame's
     * updated rows here; display_copy_hook copies and cache-flushes only [dirty_top,dirty_bot)
     * of the 640x480 4bpp panel instead of all 153,600 bytes every frame. Unioned across
     * coalesced presents (direct_pending still set) so no updated row is missed. */
    uint16_t direct_dirty_top;              /* first updated panel row (inclusive) */
    uint16_t direct_dirty_bot;              /* last updated panel row + 1 (exclusive); 0 = none yet */
    uint32_t mic_layout_rev4;               /* 2.2.10.50: layout bump (fresh context) */
    uint32_t mic_peer_sync_ignored;         /* 2.2.10.51: peer 0x010C frames dropped on the armed RIGHT */
    uint32_t mic_layout_rev5;               /* 2.2.10.51: layout bump (fresh context) */
    uint32_t mic_layout_rev6;               /* 2.2.10.52: layout bump (fresh context) */
    uint32_t mic_last_tap_tick;             /* 2.2.10.53: FW_MS_TICK of the last emitted array frame */
    uint32_t mic_stage_deadline_tick;       /* 2.2.10.53: when the pending settle stage must have fired by */
    uint32_t mic_armed_tick;                /* 2.2.10.53: FW_MS_TICK of the session start */
    uint32_t mic_recoveries;                /* 2.2.10.53: stale/dead-session recoveries (host reboot mid-session) */
    uint32_t mic_layout_rev7;               /* 2.2.10.53: layout bump (fresh context) */
    uint32_t mic_layout_rev8;               /* 2.2.10.54: layout bump (fresh context) */
    uint32_t mic_layout_rev9;               /* 2.2.10.55: layout bump (fresh context) */
    uint32_t mic_layout_rev11;              /* 2.2.10.57: layout bump (fresh context) */
    uint32_t mic_layout_rev13;              /* 2.2.10.59: layout bump (fresh context) */
    uint32_t mic_stock_releases;            /* 2.2.10.60: stock audio-manager slots released while armed */
    uint32_t mic_layout_rev14;              /* 2.2.10.60: layout bump (fresh context) */
    uint32_t mic_layout_rev15;              /* 2.2.10.61: layout bump (fresh context) */
    /* 2.2.10.62: inter-temple LC3 relay (RIGHT -> LEFT over the common-data link, LEFT -> phone
     * as one 4-channel LC3 stream). Buffers live in heap 13; all pointers 0 until first use. */
    void    *relay_enc[2];                  /* SVC_Lc3EncodeMono contexts (0x1c header + encoder) */
    uint8_t *relay_ring;                    /* LEFT: RELAY_RING_N entries of RIGHT chunks */
    uint8_t *relay_out;                     /* packet / notify assembly buffer */
    uint32_t relay_start_tick;              /* LEFT: when START was last sent; RIGHT: tick offset base */
    int32_t  relay_offset_ticks;            /* RIGHT: left_tick = FW_MS_TICK + offset */
    uint16_t relay_seq;                     /* RIGHT: chunk counter since START */
    uint8_t  relay_started;                 /* RIGHT: START received; LEFT: ACK received */
    uint8_t  relay_fbytes;                  /* LC3 bytes per channel frame in use */
    uint16_t relay_tx_pkts, relay_rx_pkts, relay_paired, relay_missing, relay_enc_fail, relay_bad_len;
    int16_t  relay_last_offset_ms;          /* LEFT: RIGHT chunk time minus own chunk time */
    uint32_t relay_ctl_rx;                  /* 2.2.10.63: relay control/audio packets received */
    uint32_t relay_hook_calls;              /* 2.2.10.63: total 0x010C common-data dispatches seen */
    uint32_t relay_stat_ctr;                /* 2.2.10.63: tap counter driving both-side RS emission */
    uint32_t relay_tx_ctl;                  /* 2.2.10.64: control sends attempted */
    uint32_t relay_tx_ctl_ok;               /* 2.2.10.64: control sends returning 0 */
    uint8_t  relay_last_rx[4];              /* 2.2.10.64: first 4 bytes of last hook payload */
    uint16_t relay_last_rx_len;             /* 2.2.10.64: length of last hook payload */
    uint32_t mic_layout_rev18;              /* 2.2.10.64: layout bump (fresh context) */
    uint32_t mic_layout_rev19;              /* 2.2.10.65: layout bump (fresh context) */
    uint32_t mic_layout_rev20;              /* 2.2.10.66: layout bump (fresh context) */
    uint32_t mic_layout_rev21;              /* 2.2.10.67: layout bump (fresh context) */
    uint32_t mic_layout_rev22;              /* 2.2.10.68: layout bump (fresh context) */
    /* 2.2.10.69: display-path efficiency. One zlib inflate stream lives for the whole session
     * (inflateInit2 once, inflateReset per frame) instead of a 34-40 KB window alloc/free per
     * frame through the stock heap; the display gate is taken directly on the stock semaphore
     * so ownership is known from the take's return value, never inferred. */
    uint8_t *zstrm;                         /* zlib 1.1.4 z_stream (0x38 bytes) in heap 13 */
    uint8_t  zstrm_ready;                   /* 1 once inflateInit2 succeeded on zstrm */
    uint8_t  gate_held;                     /* diagnostic: worker currently owns the display gate */
    uint16_t zstrm_fail;                    /* inflateInit2/inflateReset failures (sticky count) */
    uint32_t gate_timeouts;                 /* display-gate takes that timed out (frame dropped) */
    uint32_t mic_dle_requests;              /* HciLeSetDataLen requests issued on the phone link */
    uint32_t mic_layout_rev23;              /* 2.2.10.69: layout bump (fresh context) */
    uint32_t mic_layout_rev24;              /* 2.2.10.70: layout bump (fresh context) */
    uint32_t mic_layout_rev25;              /* 2.2.10.71: layout bump (fresh context) */
    uint8_t *rle_chunk;                     /* 2.2.10.72: RLE_CHUNK-byte inflate output chunk (heap 13) */
    uint16_t relay_notify_skipped;          /* 2.2.10.72: relay notifies withheld for queue back-pressure */
    uint16_t relay_notify_pad;
    uint32_t mic_layout_rev26;              /* 2.2.10.72: layout bump (fresh context) */
} customCfwContext;

#define CFW_CTX_SLOT  0x2029f4a8U    /* first word of the CFW-reserved TLSF tail */
#define CFW_ALLOC_DIAG_SLOT 0x2029f4acU /* second word: magic | sticky failure bit */
#define CFW_ALLOC_DIAG_MAGIC 0xA110CA7EU

// Marker used to validate that the CFW context pointer hasn't been clobbered.
// Does not need updating.
#define CFW_CTX_MAGIC 0xC0FFEE6AU

#define FW_MS_TICK  (*(volatile uint32_t *)0x20076de0U)  /* firmware 1 ms OS tick (SysTick chain) */

static customCfwContext *peekCustomCfwContext(void);
static customCfwContext *getCustomCfwContext(void);
int cfw_fb_lease_active(void);
