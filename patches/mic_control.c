#include <stdint.h>
#include "cfw_context.h"
#include "malloc.h"

/*
 * Microphone-control + multi-channel routing CFW extension for the G2
 * (SybilSight "glasses -> microphones").
 *
 * TOPOLOGY. Each temple is its own Apollo510 and its own BLE endpoint, and each
 * temple carries a PAIR of microphones (front + rear along the temple). So the
 * system is two independent 2-mic arrays: the phone connects to Left and Right
 * separately and receives at most 2 channels from each. There is no shared audio
 * hardware between temples, so each temple captures, encodes, and streams its own
 * pair on its own clock and its own link. The phone reassembles the four channels
 * and does array processing (beamforming / noise isolation / direction detection,
 * fused with the compass + IMU heading it already receives) itself.
 *
 * STOCK PIPELINE (openCFW recovery of the base g2_2.2.6.10 image; addresses and
 * behavior pinned by the manifests in
 * evenRealities-openCFW/g2/tools/manifests/g2-service-audio-*.tsv,
 * g2-service-algo-*.tsv, and g2-production-mic-*.tsv, with the narrative in
 * g2/docs/research/g2-service-audio-recovery.md, g2-service-algo-recovery.md,
 * and g2-production-mic-recovery.md):
 *   service_audio.c          [0x0057A900,0x0057B444)  two PCM app slots + LC3
 *   service_algo.c           [0x005915DC,0x00591D14)  per-frame SSR + TDOA angle
 *   production_mic_func.c    [0x0058F4E4,0x0058F8CC)  codec/PDM, single/stereo init
 *   drv_pdm_production.c      0x0057B444...           Ambiq PDM capture driver
 * The stock two-channel capture already exists (codec front end, stereo callback,
 * source slot 0), and service_algo_process (0x00591BFC) already returns a signed
 * TDOA angle + SSR per frame -- the bearing a beamformer wants. What stock lacks,
 * and this file adds, is (a) a control plane so the phone can choose front end /
 * channels / rate / codec / bitrate, and (b) a routing path that forwards BOTH
 * channels (instead of the stock mono average) plus the angle and a timestamp, so
 * the phone can beamform.
 *
 * CONTRACT. Rides the already-wired sid-0x09 settings hooks (settings_send_wrapper
 * / settings_decode_wrapper), so NO new binary patch offsets are introduced -- the
 * injected blob just grows and gen_patches.py recomputes sizes/checksums.
 *
 *   field 103 (RX)  ['M','C', ver=1, op, <op payload>]
 *     op 1 CONFIGURE  [src, chanMask, codec, fmt, rateLo, rateHi, brLo, brHi, flags]
 *          src      0 = codec DMIC/I2S front end, 1 = Ambiq PDM mics
 *          chanMask bit0/bit1 = enable this temple's front/rear mic
 *          codec    0 = LC3 encoded, 1 = raw PCM passthrough
 *          fmt      0 = 16-bit, 1 = 24-bit, 2 = 32-bit PCM sample width
 *          rate     LE16, sample rate in units of 100 Hz  (160 = 16 kHz; clamp 80..480)
 *          br       LE16, LC3 target bitrate in units of 100 bps (0 = default; <=5000)
 *          flags    bit0 MIC_FLAG_BEAMFORM  append the SSR/TDOA angle to each frame
 *                   bit1 MIC_FLAG_ARM_HW    bring up capture + streaming (GATED)
 *          Arming also starts a fail-open 90 s streaming lease (below).
 *     op 2 QUERY      push the live config now as a field-104 notify (and it is
 *                     also appended to every sid-0x09 settings READ response)
 *     op 3 STOP       tear down the CFW capture session, restore stock
 *     op 4 RENEW      renew the streaming lease without touching the config
 *
 *   field 104 (TX)  ['M','C', ver=1, active, src, chanMask, codec, fmt,
 *                    rateLo, rateHi, brLo, brHi, flags, hwArmed, sideId,
 *                    framesLo..framesHi(32), effRateLo, effRateHi]
 *     `rate`/`br` echo the REQUESTED values; `effRate` is what capture actually
 *     runs at (see EFFECTIVE vs REQUESTED below). `sideId`: 1 = right temple,
 *     2 = left temple.
 *
 * The phone sends an IDENTICAL CONFIGURE to both temples for a consistent array;
 * each temple answers field 104 on its own link so SybilSight can confirm they
 * match before enabling its radar-style beam view.
 *
 * MULTI-CHANNEL STREAM FRAME (glasses -> phone, via the stock streaming-notify
 * BLE facade), fixed 21-byte header:
 *   [0]  'S'   [1] 'M'   [2] ver=1
 *   [3]  flags       config MIC_FLAG_* bits, plus bit7 = payload truncated
 *   [4]  seqLo  [5] seqHi        (low 16 bits of the frame counter)
 *   [6..9]  tick     u32 LE, this temple's 1 ms OS tick at packetization
 *   [10] nCh         channels in the payload (1 or 2)
 *   [11..12] rateDiv u16 LE, EFFECTIVE sample rate in units of 100 Hz
 *   [13] fmt         PCM width code as configured (0=16/1=24/2=32-bit)
 *   [14] codec       what the payload ACTUALLY is (0 = LC3, 1 = raw PCM)
 *   [15..16] angle   s16 LE, on-device TDOA angle (degrees; 0 if not computed)
 *   [17..18] ssr     s16 LE, on-device SSR ratio (0 if not computed)
 *   [19..20] payLen  u16 LE, payload bytes that follow
 * `tick` gives coarse host-side L/R alignment; `angle`/`ssr` are this temple's
 * own-pair estimate (only computed when MIC_FLAG_BEAMFORM is set AND the frame
 * is 2-channel 16-bit -- the recovered algo object expects interleaved stereo
 * 16-bit input). CHANNEL LAYOUT CAVEAT: the recovered stereo production callback
 * dispatches the two channels as back-to-back 400-byte blocks (concatenated, not
 * interleaved) when channel extraction is enabled, and forwards the raw source
 * buffer when it is not. The payload is the dispatched buffer verbatim; which
 * layout a live session produces is a validation-gate item -- confirm on
 * hardware and pin it in the SybilSight demuxer.
 *
 * EFFECTIVE vs REQUESTED. The recovered init entries take no rate/width/bitrate
 * arguments -- stock capture runs the LC3 voice pipeline's fixed 16 kHz. The
 * requested rate/bitrate are therefore stored and echoed (so the UI round-trips
 * user intent) but capture runs at MIC_RATE_DEFAULT until the codec/PDM
 * reconfiguration seams are recovered; stream frames carry the EFFECTIVE rate,
 * which is the one the host DSP must trust. Likewise a requested codec=LC3 keeps
 * streaming raw PCM (frame byte 14 says so) until the on-device per-channel
 * LC3 encode path (SVC_Lc3EncodeMono @ 0x0057A940) has a validated ABI: raw
 * frames always carry the true multi-channel samples the beamformer needs, so
 * "best quality" is the default rather than a failure mode.
 *
 * STREAMING LEASE (fail-open, same paradigm as the wake + framebuffer leases).
 * An armed session is only kept alive while the phone renews it: CONFIGURE and
 * RENEW both push the deadline MIC_LEASE_MS out and (re)arm a one-shot osTimer
 * watchdog. If the phone disappears, mic_pcm_tap stops emitting immediately at
 * the deadline (cheap signed tick compare) and the watchdog tears the capture
 * hardware down from the RTOS timer thread. Mode-11 session cleanup
 * (cfw_cleanup_session) also tears the session down, so a departing custom app
 * cannot leave the mics running.
 *
 * SAFETY / STATUS. A CONFIGURE without MIC_FLAG_ARM_HW only stores and advertises
 * the configuration -- it touches no audio hardware and cannot fault. The capture
 * + streaming path (mic_session_start / mic_pcm_tap / mic_session_stop) is
 * compiled in but runs ONLY when the phone sets MIC_FLAG_ARM_HW, and every
 * firmware entry it uses is an address-pinned but ABI-INFERRED seam (the tap
 * callback signature, the codec-init channel selector, and the streaming-notify
 * sender args are recovered behaviourally, not to register level). Those seams
 * MUST be confirmed on sacrificial hardware before a phone build ships with
 * ARM_HW enabled. Self-contained: no external symbols, no writable globals;
 * state lives in the customCfwContext singleton, guarded by magic + bounds.
 */

/* --- Recovered stock audio entry points (Thumb bit set for blx via const ptr).
 * ABI-INFERRED where noted; every use is gated behind MIC_FLAG_ARM_HW. --- */
typedef void (*mic_sel_fn)(uint32_t selector);
typedef void (*mic_void_fn)(void);
/* SVC_PcmAppRegister(slot, app_id, callback) — recovery: "registers one callback
 * and application ID in either of two PCM source slots"; SVC_PcmAppProcessData
 * dispatches to the registered callback INSTEAD of the stock mono-average
 * fallback. Argument order is inferred. */
typedef int  (*pcm_register_fn)(uint32_t slot, uint32_t app_id, void *cb);
typedef int  (*pcm_unregister_fn)(uint32_t slot, uint32_t app_id);
/* service_algo_process(interleaved2ch16, &ssr, &angle) — recovery: "preprocesses
 * one frame and returns SSR and angle results through two shorts". */
typedef void (*algo_process_fn)(const void *pcm, int16_t *ssr, int16_t *angle);
/* Thread_MsgStreamingNotifyByBle @ 0x00475D78 — the facade the stock fallback
 * path forwards its completed LC3 packet through ("transport-one subtype-one
 * wrapper"). ABI inferred as (buf, len). */
typedef int  (*audio_notify_fn)(const void *buf, uint32_t len);
typedef uint32_t (*lens_side_fn2)(void);

#define FW_CODEC_MIC_INIT    ((mic_sel_fn)0x0058F69BU)      /* production_codec_mic_func_init  */
#define FW_CODEC_MIC_DEINIT  ((mic_void_fn)0x0058F74BU)     /* production_codec_mic_func_deinit */
#define FW_PDM_MIC_INIT      ((mic_sel_fn)0x0058F7B1U)      /* production_pdm_mic_func_init    */
#define FW_PDM_MIC_DEINIT    ((mic_void_fn)0x0058F807U)     /* production_pdm_mic_func_deinit  */
#define FW_PCM_REGISTER      ((pcm_register_fn)0x0057AB79U) /* SVC_PcmAppRegister   (ABI inferred) */
#define FW_PCM_UNREGISTER    ((pcm_unregister_fn)0x0057ACD1U)/* SVC_PcmAppUnregister (ABI inferred) */
#define FW_ALGO_PROCESS      ((algo_process_fn)0x00591BFDU) /* service_algo_process (ABI inferred) */
#define FW_AUDIO_NOTIFY      ((audio_notify_fn)0x00475D79U) /* streaming notify     (ABI inferred) */
#define FW_MIC_SIDE          ((lens_side_fn2)0x0045a569U)   /* 1 = right temple, 2 = left temple */
/* Future validation-gate seam, unused until its ABI is confirmed: on-device LC3
 * (SVC_Lc3EncodeMono @ 0x0057A940, "encodes one or more mono or interleaved PCM
 * frames through liblc3") would let codec=LC3 honor mic_bitrate_100. */

/* wire contract */
#define MIC_CONTROL_FIELD    103u
#define MIC_STATUS_FIELD     104u
#define MIC_PROTO_VERSION    1u
#define MIC_OP_CONFIGURE     1u
#define MIC_OP_QUERY         2u
#define MIC_OP_STOP          3u
#define MIC_OP_RENEW         4u

#define MIC_SRC_CODEC        0u
#define MIC_SRC_PDM          1u
#define MIC_CODEC_LC3        0u
#define MIC_CODEC_RAW        1u
#define MIC_FLAG_BEAMFORM    0x01u
#define MIC_FLAG_ARM_HW      0x02u
#define MIC_STREAM_TRUNC     0x80u  /* stream-frame flags bit: payload was truncated */

#define MIC_RATE_MIN_100HZ   80u    /* 8 kHz  */
#define MIC_RATE_MAX_100HZ   480u   /* 48 kHz */
#define MIC_RATE_DEFAULT     160u   /* 16 kHz — the stock LC3 capture rate */
#define MIC_BR_MAX_100BPS    5000u  /* <= 500 kbps */
#define MIC_LEASE_MS         90000u /* same fail-open cadence as the wake/fb leases */

/* Stream-frame constants. */
#define MIC_STREAM_MAGIC0    'S'
#define MIC_STREAM_MAGIC1    'M'
#define MIC_STREAM_HDR_BYTES 21u
/* The recovered production callbacks work in 400-byte per-channel chunks (the
 * stereo callback dispatches 2x400 B); the algo work buffer is 1600 B. Anything
 * larger than that is not a plausible capture chunk — truncate and flag it. */
#define MIC_STREAM_MAX_PAY   1600u
/* PCM app slot the CFW tap registers on (slot 0 = the slot the stock stereo
 * callback dispatches into and whose empty-slot fallback is the mono-average
 * LC3 path; registering here suppresses that fallback). */
#define MIC_PCM_SLOT         0u
#define MIC_APP_ID           0x4643u  /* "FC"-ish CFW owner id */

static uint8_t mic_popcount2(uint8_t mask) {
    return (uint8_t)((mask & 1u) + ((mask >> 1) & 1u));
}

/* What capture actually runs at: the recovered init entries take no rate
 * argument, so an armed session is the stock fixed 16 kHz pipeline regardless
 * of the requested rate (see EFFECTIVE vs REQUESTED above). */
static uint16_t mic_effective_rate(const customCfwContext *ctx) {
    (void)ctx;
    return (uint16_t)MIC_RATE_DEFAULT;
}

/* Channels a live session actually delivers. The recovered PDM init registers
 * only the SINGLE-channel callback (capture mode 1), so PDM is mono in stock;
 * only the codec front end has the stereo callback. A rear-only mask (0x2)
 * still needs stereo capture — the phone drops the front channel. */
static uint8_t mic_effective_channels(const customCfwContext *ctx) {
    if (ctx->mic_source == MIC_SRC_PDM) return 1u;
    return ctx->mic_chan_mask == 0x1u ? 1u : 2u;
}

static int mic_lease_live(const customCfwContext *ctx) {
    return ctx->mic_lease_deadline != 0 &&
           (int32_t)(ctx->mic_lease_deadline - FW_MS_TICK) > 0;
}

/* ---- capture + streaming (GATED behind MIC_FLAG_ARM_HW) -------------------- */

/* The registered PCM tap. Receives this temple's capture dispatch, packs a
 * stream frame with the timestamp and on-device angle/SSR, and hands it to the
 * streaming-notify facade. Runs on the audio service's thread. ABI
 * (source, pcm, bytes) is INFERRED — do not enable ARM_HW until it is confirmed
 * on hardware. Nonstatic + noinline: registered by address via `&`. */
__attribute__((used, noinline)) void mic_pcm_tap(uint32_t source, const void *pcm, uint32_t bytes) {
    (void)source;
    customCfwContext *ctx = peekCustomCfwContext();
    if (!ctx || !ctx->mic_active || !ctx->mic_hw_armed || pcm == 0 || bytes == 0) return;
    /* Fail-open: the phone stopped renewing — stop emitting immediately. The
     * watchdog timer does the actual hardware teardown from the timer thread
     * (deinit from inside the capture callback would be re-entrant). */
    if (!mic_lease_live(ctx)) return;

    uint8_t nch = mic_effective_channels(ctx);
    int16_t ssr = 0, angle = 0;
    /* The recovered algo object splits 800 interleaved stereo 16-bit frames;
     * feeding it anything else would return garbage bearings. */
    if ((ctx->mic_flags & MIC_FLAG_BEAMFORM) && nch == 2u && ctx->mic_format == 0u)
        FW_ALGO_PROCESS(pcm, &ssr, &angle);

    uint8_t flags = ctx->mic_flags;
    uint32_t pay = bytes;
    if (pay > MIC_STREAM_MAX_PAY) { pay = MIC_STREAM_MAX_PAY; flags |= MIC_STREAM_TRUNC; }

    uint8_t *f = (uint8_t *)cfw_malloc(MIC_STREAM_HDR_BYTES + pay);
    if (!f) return;
    uint32_t tick = FW_MS_TICK;
    uint16_t seq = (uint16_t)ctx->mic_frames;
    uint16_t rate = mic_effective_rate(ctx);
    f[0] = MIC_STREAM_MAGIC0; f[1] = MIC_STREAM_MAGIC1; f[2] = MIC_PROTO_VERSION;
    f[3] = flags;
    f[4] = (uint8_t)seq; f[5] = (uint8_t)(seq >> 8);
    f[6] = (uint8_t)tick; f[7] = (uint8_t)(tick >> 8);
    f[8] = (uint8_t)(tick >> 16); f[9] = (uint8_t)(tick >> 24);
    f[10] = nch;
    f[11] = (uint8_t)rate; f[12] = (uint8_t)(rate >> 8);
    f[13] = ctx->mic_format;
    f[14] = MIC_CODEC_RAW;   /* payload is raw PCM until the LC3 seam is validated */
    f[15] = (uint8_t)angle; f[16] = (uint8_t)((uint16_t)angle >> 8);
    f[17] = (uint8_t)ssr;   f[18] = (uint8_t)((uint16_t)ssr >> 8);
    f[19] = (uint8_t)pay;   f[20] = (uint8_t)(pay >> 8);
    for (uint32_t i = 0; i < pay; i++) f[MIC_STREAM_HDR_BYTES + i] = ((const uint8_t *)pcm)[i];

    FW_AUDIO_NOTIFY(f, MIC_STREAM_HDR_BYTES + pay);
    ctx->mic_frames++;
    FW_FREE(f);
}

/* Bring up this temple's selected front end + channel pair and register the tap.
 * Releases the other front end first so both are never held at once. The codec
 * init's one-byte argument "selects the single or stereo callback"; the
 * boolean encoding (0 = single, 1 = stereo) is inferred — validation-gate item. */
static void mic_session_start(customCfwContext *ctx) {
    if (!(ctx->mic_flags & MIC_FLAG_ARM_HW)) return;
    if (ctx->mic_source == MIC_SRC_PDM) {
        FW_CODEC_MIC_DEINIT();
        FW_PDM_MIC_INIT(0);                 /* PDM init has only the single callback */
    } else {
        FW_PDM_MIC_DEINIT();
        FW_CODEC_MIC_INIT(mic_effective_channels(ctx) == 2u ? 1u : 0u);
    }
    FW_PCM_REGISTER(MIC_PCM_SLOT, MIC_APP_ID, (void *)&mic_pcm_tap);
    ctx->mic_hw_armed = 1;
}

static void mic_session_stop(customCfwContext *ctx) {
    if (!ctx->mic_hw_armed) return;
    ctx->mic_hw_armed = 0;                  /* silence the tap before teardown */
    FW_PCM_UNREGISTER(MIC_PCM_SLOT, MIC_APP_ID);
    FW_CODEC_MIC_DEINIT();
    FW_PDM_MIC_DEINIT();
}

/* Watchdog callback, on the RTOS timer thread (the same context stock deinit
 * paths run in). Tears down an armed session whose lease lapsed; if the lease
 * was renewed since arming, re-arms itself for the remaining time. */
void mic_watchdog_tick(void *arg) {
    customCfwContext *ctx = (customCfwContext *)arg;
    if (!ctx || ctx->magic != CFW_CTX_MAGIC || !ctx->mic_hw_armed) return;
    if (mic_lease_live(ctx)) {
        uint32_t left = ctx->mic_lease_deadline - FW_MS_TICK;
        if (ctx->mic_watchdog_timer) FW_TIMER_START(ctx->mic_watchdog_timer, left ? left : 1u);
        return;
    }
    ctx->mic_lease_deadline = 0;
    mic_session_stop(ctx);
}

static void mic_lease_renew(customCfwContext *ctx) {
    ctx->mic_lease_deadline = FW_MS_TICK + MIC_LEASE_MS;
    if (!ctx->mic_hw_armed) return;
    if (ctx->mic_watchdog_timer == 0)
        ctx->mic_watchdog_timer = FW_TIMER_NEW((void *)&mic_watchdog_tick, 0, ctx, 0);
    if (ctx->mic_watchdog_timer) {
        FW_TIMER_STOP(ctx->mic_watchdog_timer);
        FW_TIMER_START(ctx->mic_watchdog_timer, MIC_LEASE_MS);
    }
}

/* Full teardown for STOP / mode-11 session cleanup (cfw_cleanup_session).
 * Idempotent, same retry contract as the other cleanup-owned timers: a timer
 * whose delete fails stays in the context so a later cleanup can retry. */
static void mic_cleanup_session(void) {
    customCfwContext *ctx = peekCustomCfwContext();
    if (!ctx) return;
    ctx->mic_lease_deadline = 0;
    mic_session_stop(ctx);
    ctx->mic_active = 0;
    if (ctx->mic_watchdog_timer) {
        FW_TIMER_STOP(ctx->mic_watchdog_timer);
        if (FW_TIMER_DELETE(ctx->mic_watchdog_timer) == 0) ctx->mic_watchdog_timer = 0;
    }
}

/* ---- control plane (always active; touches no hardware unless armed) ------- */

static unsigned mic_status_body(customCfwContext *ctx, unsigned char *body);

/* Push the live config to the phone right now as a standalone sid-0x09 notify
 * (G2SettingPackage{commandId=3, magic=0, field104}), the same wire shape as
 * the Faceclaw wake event. Unlike that event this is NOT right-arm gated: the
 * phone configures each temple over its own link and each answers for itself.
 * The buffer lives in the singleton because the stock sender's copy/queue
 * lifetime is intentionally treated as opaque. */
static void mic_send_status_notify(customCfwContext *ctx) {
    unsigned char *p = ctx->mic_notify_buf;
    p[0] = 0x08; p[1] = 0x03;                       /* field 1: commandId=3 */
    p[2] = 0x10; p[3] = 0x00;                       /* field 2: magic=0 */
    p[4] = 0xC2; p[5] = 0x06;                       /* field 104, wire type 2: tag 834 */
    unsigned n = mic_status_body(ctx, p + 7);
    p[6] = (unsigned char)n;
    ((send_fn)FW_SEND)(1, 9, p, 7 + n);
}

/* Parse a field-103 record. Called from faceclaw_scan_settings_control for each
 * sid-0x09 settings WRITE, before the stock decoder runs. */
void mic_apply_control(const uint8_t *data, uint32_t len) {
    if (len < 4u || data[0] != 'M' || data[1] != 'C' ||
        data[2] != MIC_PROTO_VERSION) return;
    customCfwContext *ctx = getCustomCfwContext();
    if (!ctx) return;
    uint8_t op = data[3];

    if (op == MIC_OP_CONFIGURE) {
        if (len < 4u + 9u) return;
        const uint8_t *c = data + 4;

        /* Reconfiguring a live session: quiesce the old one first. */
        mic_session_stop(ctx);

        ctx->mic_source    = c[0] > MIC_SRC_PDM ? MIC_SRC_CODEC : c[0];
        ctx->mic_chan_mask = c[1] & 0x03u;
        if (ctx->mic_chan_mask == 0) ctx->mic_chan_mask = 0x01u;
        ctx->mic_codec  = c[2] > MIC_CODEC_RAW ? MIC_CODEC_LC3 : c[2];
        ctx->mic_format = c[3] > 2u ? 0u : c[3];

        uint32_t rate = (uint32_t)c[4] | ((uint32_t)c[5] << 8);
        if (rate < MIC_RATE_MIN_100HZ) rate = MIC_RATE_MIN_100HZ;
        if (rate > MIC_RATE_MAX_100HZ) rate = MIC_RATE_MAX_100HZ;
        ctx->mic_rate_hz_div = (uint16_t)rate;

        uint32_t br = (uint32_t)c[6] | ((uint32_t)c[7] << 8);
        if (br > MIC_BR_MAX_100BPS) br = MIC_BR_MAX_100BPS;
        ctx->mic_bitrate_100 = (uint16_t)br;

        ctx->mic_flags    = c[8] & (MIC_FLAG_BEAMFORM | MIC_FLAG_ARM_HW);
        ctx->mic_channels = mic_popcount2(ctx->mic_chan_mask);
        ctx->mic_active = 1;
        ctx->mic_frames = 0;
        mic_session_start(ctx);              /* no-op unless MIC_FLAG_ARM_HW set */
        mic_lease_renew(ctx);
        mic_send_status_notify(ctx);         /* confirm the applied config */
    } else if (op == MIC_OP_QUERY) {
        mic_send_status_notify(ctx);
    } else if (op == MIC_OP_STOP) {
        mic_cleanup_session();
        mic_send_status_notify(ctx);
    } else if (op == MIC_OP_RENEW) {
        if (ctx->mic_active) mic_lease_renew(ctx);
    }
}

/* Serialize the field-104 status body (21 bytes; see the contract above). */
static unsigned mic_status_body(customCfwContext *ctx, unsigned char *body) {
    unsigned n = 0;
    body[n++] = 'M'; body[n++] = 'C'; body[n++] = (unsigned char)MIC_PROTO_VERSION;
    body[n++] = ctx->mic_active;
    body[n++] = ctx->mic_source;
    body[n++] = ctx->mic_chan_mask;
    body[n++] = ctx->mic_codec;
    body[n++] = ctx->mic_format;
    body[n++] = (unsigned char)ctx->mic_rate_hz_div;
    body[n++] = (unsigned char)(ctx->mic_rate_hz_div >> 8);
    body[n++] = (unsigned char)ctx->mic_bitrate_100;
    body[n++] = (unsigned char)(ctx->mic_bitrate_100 >> 8);
    body[n++] = ctx->mic_flags;
    body[n++] = ctx->mic_hw_armed;
    body[n++] = (unsigned char)FW_MIC_SIDE();     /* 1 = right, 2 = left */
    body[n++] = (unsigned char)ctx->mic_frames;
    body[n++] = (unsigned char)(ctx->mic_frames >> 8);
    body[n++] = (unsigned char)(ctx->mic_frames >> 16);
    body[n++] = (unsigned char)(ctx->mic_frames >> 24);
    uint16_t eff = mic_effective_rate(ctx);
    body[n++] = (unsigned char)eff;
    body[n++] = (unsigned char)(eff >> 8);
    return n;
}

/* Append the live mic configuration as settings field 104 (wire type 2) to the
 * sid-0x09 settings READ response. tag = (104<<3)|2 = 834 -> varint 0xC2 0x06.
 * Returns bytes written. A missing context reports an all-zero (inactive) body
 * of the same shape so the phone parser never needs a special case. */
unsigned mic_append_status(unsigned char *p) {
    customCfwContext *ctx = peekCustomCfwContext();
    unsigned char body[24];
    unsigned n;
    if (ctx) {
        n = mic_status_body(ctx, body);
    } else {
        n = 0;
        body[n++] = 'M'; body[n++] = 'C'; body[n++] = (unsigned char)MIC_PROTO_VERSION;
        while (n < 21u) body[n++] = 0;
    }
    p[0] = 0xC2; p[1] = 0x06; p[2] = (unsigned char)n;
    for (unsigned i = 0; i < n; i++) p[3 + i] = body[i];
    return 3u + n;
}
