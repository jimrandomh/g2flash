#pragma once
#include <stdint.h>
#include "message_transport.h"
#include "panel.h"

/* Persistent CFW-owned state, independent of EvenHub image containers. The
 * full-panel shadow is an owned heap allocation. This context is anchored in
 * the 1 KiB SRAM region explicitly
 * removed from the stock primary TLSF arena: [0x2029f59c,0x2029f99c). The first
 * word holds the pointer; the second holds a sticky allocation diagnostic. */

#define CFW_FID_RING  16     /* recent mode-3 frame ids kept for diagnostics */
#define CFW_TIMER_PAINT_SAMPLES 10
#define CFW_SEQ_MAX   48     /* max steps in a buzzer tone sequence (mode-5 kind 4) */

/* Sidecar for a stock IMU ring record; written/read on the sensor-hub task. */
typedef struct {
    uint32_t timestamp;
    uint8_t accuracy, anomalies, source, flags;
} cfw_compass_sample;

/* One aligned word publishes a consistent size/CRC pair across the BLE and
 * display tasks. The firmware target is little-endian. */
typedef union {
    struct { uint16_t size, checksum; } fields;
    uint32_t snapshot;
} cfw_message_probe;

typedef struct {
    uint32_t magic;      /* CFW_CTX_MAGIC when valid */
    /* --- diagnostics, overlaid as a text line (verify the fix; should stay clear). Mode
     * 7 clears the flags / toggles the overlay visibility (diag_hide). --- */
    uint16_t last_fid;   /* last frame id seen (mode-3 messages) */
    uint16_t high_fid;   /* highest frame id seen */
    uint8_t  diag_seen;  /* recorded at least one frame yet */
    uint8_t  fid_resync; /* keyframe rebaselines the next delta's fid (no false skip) */
    uint8_t  diag_hide;  /* 1 = don't draw the flag overlay (default 0 = visible) */
    uint32_t last_worker_us;  /* image_worker() duration of the PREVIOUS message (overlay) */
    uint32_t last_present_us; /* present_composition() duration of the PREVIOUS present (overlay) */
    uint32_t cyc_per_ms;      /* calibrated DWT cycles per 1 ms OS tick (0 = not yet done) */
    uint8_t  f_reorder;  /* FLAG: ever saw a frame id go backward */
    uint8_t  f_skip;     /* FLAG: ever saw a frame id gap (skipped) */
    uint8_t  f_dup;      /* FLAG: ever saw a duplicate frame id (in the recent ring) */
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
    /* Direct-framebuffer job. The custom worker holds the stock display gate
     * before it mutates the shadow and until the display task consumes this
     * pointer, so no second snapshot or full-size display buffer is required. */
    const uint8_t *direct_shadow;
    volatile uint8_t direct_pending;
    uint8_t direct_failed;
    uint8_t direct_active;                    /* physical framebuffer currently owns the image */
    uint32_t direct_lease_deadline;            /* fail-open repaint-guard deadline */
    /* Resource arena, including its 512-entry pointer table; allocated lazily
     * and released with the framebuffer lease. Wire references are resource IDs. */
    uint8_t *resource_cache;
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
    uint16_t als_reserved;
    uint32_t als_orig_handler;              /* stock hub handler for message 8 (Thumb address) */
    uint32_t als_last_reported;             /* value carried by the last report */
    uint32_t als_last_report_tick;          /* FW_MS_TICK of the last report (0 = none yet) */
    cfw_compass_sample compass_samples[20];
    /* Idle-input forwarding (settings_ext.c faceclaw_idle_input_gate): a
     * field-102 notify of its own, since wake_notify_buf may still be queued
     * for a deferred double-tap wake when a tap or release follows it. */
    uint8_t  gesture_notify_buf[16];
    volatile cfw_message_probe message_probe; /* latest valid SID-f0 payload */
    uint32_t image_mutex; /* serializes commands from BLE and bridge */
    uint8_t *composition_buffer; /* owned 640x480 packed 4bpp; released by mode 11 */
    cfw_message_stream message_streams[2]; /* index = BLE ingress lens bit - 1 */
    uint8_t ancs_connection, ancs_stage;
    uint16_t ancs_sequence;
    uint32_t ancs_token;
    uint8_t  ring_notify_buf[25]; /* atomic timestamped R1 SysEvent */
    uint32_t resource_free_head; /* offset of first free block; zero = none */
    cfw_panel_state panel;
    uint8_t *screen_buffer;
    uint16_t root_display_list; /* id + 1, zero means no root */
    uint32_t animation_timer;
    uint32_t animation_origin_ms;
    uint32_t animation_due_ms;
    uint32_t draw_elapsed_ms; /* one snapshot shared by validation and drawing */
    uint8_t animation_pending;
    uint8_t animation_running;
    uint32_t timer_paint_us[CFW_TIMER_PAINT_SAMPLES];
    uint32_t timer_paint_average_us; /* published to the display-task debug overlay */
    uint8_t timer_paint_count;
    uint8_t timer_paint_next;
} customCfwContext;

#define CFW_CTX_SLOT  0x2029f59cU    /* first word of the CFW-reserved TLSF tail */
#define CFW_ALLOC_DIAG_SLOT 0x2029f5a0U /* second word: magic | sticky failure bit */
#define CFW_ALLOC_DIAG_MAGIC 0xA110CA7EU

// Marker used to validate that the CFW context pointer hasn't been clobbered.
// Does not need updating.
#define CFW_CTX_MAGIC 0xC0FFEE6CU

#define FW_MS_TICK  (*(volatile uint32_t *)0x20077e4cU)  /* firmware 1 ms OS tick (SysTick chain) */

static customCfwContext *peekCustomCfwContext(void);
static customCfwContext *getCustomCfwContext(void);
int cfw_fb_lease_active(void);
