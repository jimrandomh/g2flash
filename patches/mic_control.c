#include <stdint.h>
#include "cfw_context.h"
#include "malloc.h"
#include "protobuf.h"

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
 * STOCK PIPELINE (openCFW recovery of the original g2_2.2.6.10 image; every
 * callable entry below was re-matched against g2_2.2.9.22 by its complete
 * normalized function body). The behavior is pinned by the manifests in
 * evenRealities-openCFW/g2/tools/manifests/g2-service-audio-*.tsv,
 * g2-service-algo-*.tsv, and g2-production-mic-*.tsv, with the narrative in
 * g2/docs/research/g2-service-audio-recovery.md, g2-service-algo-recovery.md,
 * and g2-production-mic-recovery.md):
 *   service_audio.c          0x005930F8...             two PCM app slots + LC3
 *   service_algo_process     0x005adf18                per-frame SSR + TDOA angle
 *   production mic init      0x005ab8ca...0x005A8E6E codec/PDM, mono/stereo
 *   drv_pdm_production.c      follows service_audio    Ambiq PDM capture driver
 * The stock two-channel capture already exists (codec front end, stereo callback,
 * source slot 0), and service_algo_process already returns a signed
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
 * LC3 encode path (SVC_Lc3EncodeMono) has a validated ABI: raw
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
/* 2.2.10 (SybilSight overlay, disassembled 2026-09-13 at 0x00595F74/0x005960CC):
 * SVC_PcmAppRegister(app_id, slot, cb): r1 is the slot (must be < 2, stored at
 * entry+4 and compared by the dispatcher), r0 the owner id stored at entry+0
 * (the stock audio front ends use 0x10B) and r2 the callback stored at entry+8.
 * Re-registering an occupied slot clears and overwrites it. Unregister compares
 * entry+0 with its first argument, so the owner id must match what was
 * registered. The upstream (slot, app_id, cb) order put 0x4643 into the slot
 * check, which failed, so the tap was never installed and no frame was sent. */
typedef int  (*pcm_register_fn)(uint32_t app_id, uint32_t slot, void *cb);
typedef int  (*pcm_unregister_fn)(uint32_t app_id, uint32_t slot);
/* service_algo_process(pcm, length, &ssr, &angle) — disassembled at 0x005ADF18:
 * four arguments; the stock dispatcher's own fallback calls it with its (pcm,
 * length) pair and two stack halfword slots. The upstream three-argument call
 * left r3 uninitialised and the angle store wrote through a stray pointer. */
typedef void (*algo_process_fn)(const void *pcm, uint32_t length, int16_t *ssr, int16_t *angle);
/* Thread_MsgStreamingNotifyByBle @ 0x0047ed08 — the facade the stock fallback
 * path forwards its completed LC3 packet through ("transport-one subtype-one
 * wrapper"). ABI inferred as (buf, len). */
typedef int  (*audio_notify_fn)(const void *buf, uint32_t len);
typedef uint32_t (*lens_side_fn2)(void);

#define FW_CODEC_MIC_INIT    ((mic_sel_fn)0x005ab8cbU)      /* production_codec_mic_func_init  */
#define FW_CODEC_MIC_DEINIT  ((mic_void_fn)0x005ab97bU)     /* production_codec_mic_func_deinit */
#define FW_PDM_MIC_INIT      ((mic_sel_fn)0x005ab9e1U)      /* production_pdm_mic_func_init    */
#define FW_PDM_MIC_DEINIT    ((mic_void_fn)0x005aba37U)     /* production_pdm_mic_func_deinit  */
#define FW_PCM_REGISTER      ((pcm_register_fn)0x00595f75U) /* SVC_PcmAppRegister   (ABI inferred) */
#define FW_PCM_UNREGISTER    ((pcm_unregister_fn)0x005960cdU)/* SVC_PcmAppUnregister (ABI inferred) */
#define FW_ALGO_PROCESS      ((algo_process_fn)0x005adf19U) /* service_algo_process (ABI inferred) */
#define FW_AUDIO_NOTIFY      ((audio_notify_fn)0x0047ed09U) /* streaming notify     (ABI inferred) */
/* _private_getCurrentRoleStatus getter (rebased to 0x0045D35D on 2.2.10.10 by the address
 * profile): returns RAM byte set from GPIO156 at boot, 1 = RIGHT, 2 = LEFT, 3 = unknown. */
#define FW_MIC_SIDE          ((lens_side_fn2)0x0045d35dU)   /* 1 = right temple, 2 = left temple */

/* 2.2.10.18: the code seam below is a 2.2.9.22 literal rebased by the address profile; the RAM cells are 2.2.10.10 addresses (RAM is not rebased).
 * thread.audio's own codec-control primitive: posts {id 0, param} to the audio thread.
 * param 1 = DMIC open + I2S output on + I2S init + arm the 2 s "codec audio check" timer;
 * param 0 = timer stop + I2S deinit + I2S output off; param 2 = full GX8002 reboot (1.75 s
 * blocking) — never used here. This is what the stock audio manager's acquire/release use.
 * ABI (uint32 param) INFERRED from stock callers 0x5ab8ca (production init: `movs r0,#1;
 * bl 0x5565e0`) and 0x56ac64 (acquire). Gated behind MIC_FLAG_MGR_CTRL from the phone. */
typedef void (*codec_ctrl_fn)(uint32_t param);
#define FW_CODEC_CTRL        ((codec_ctrl_fn)0x005565e1U)   /* 2.2.9.22 literal; rebased to 0x005565E1 by the profile */
/* 2.2.10.28: the codec-prep message the stock service_audio_manager acquire (0x56ac64) posts
 * to the audio thread IMMEDIATELY BEFORE ctrl(1). Disassembly (2026-09-14): acquire does
 * `movs r0,#1; bl 0x556602` then `movs r0,#1; bl 0x5565e0` — 0x556602 posts codec message
 * subtype 1 via the audio-thread poster 0x5561b0 (queue 0x200046f8), which is the GX8002
 * configuration step. Every earlier CFW candidate posted ONLY ctrl(1) (I2S peripheral on)
 * and never this prep, so the GX8002 chip was never told to produce valid audio and the I2S
 * DMA read its free-running output as full-scale noise (the array-capture root cause). Doing
 * prep(1) then ctrl(1) reproduces the exact clean bring-up the stock LEFT mic uses; it has no
 * side gate so it works on the RIGHT temple too. 2.2.9.22 literal 0x00556602 rebased to
 * 0x00556603 by the profile (0x556602 on 2.2.10.10, Thumb +1). ABI (uint32 subtype) inferred
 * from the two stock callers 0x556602 (subtype 1) and 0x556624 (subtype 4). */
typedef void (*codec_prep_fn)(uint32_t subtype);
#define FW_CODEC_PREP        ((codec_prep_fn)0x00556603U)   /* 2.2.9.22 literal 0x00556602+1; rebased to 0x00556603 by the profile */
/* 2.2.10.46: the LEFT temple is the service_audio_manager hardware owner (role 2). Its own codec
 * power-cycle resets the shared audio-manager state and disables the codec/DMA, so after the cycle
 * the LEFT re-acquires through the manager to bring the codec + PDM + DMA back before it taps
 * (openCFW: first acquire resets shared state and enables PDM + codec; release when all slots 0
 * disables them). The RIGHT has no audio manager and never calls these. Rebased by the profile. */
typedef void (*audm_fn)(uint32_t app_id);
#define FW_AUDM_ACQUIRE      ((audm_fn)0x0056ac65U)   /* rebased to 0x0056AC65 */
/* 2.2.10.52: the two halves of the stock ctrl(2) reboot, so the rail cycle can be staged over
 * RTOS timers instead of blocking the CONFIGURE thread for the codec's whole boot. */
typedef void (*codec_pwr_step_fn)(void);
#define FW_CODEC_PWR_OFF     ((codec_pwr_step_fn)0x005959b9U)    /* gx8002 power off; rebased to 0x005959B9 */
#define FW_CODEC_PWR_ON      ((codec_pwr_step_fn)0x00595905U)    /* gx8002 power on;  rebased to 0x00595905 */
#define FW_AUDM_RELEASE      ((audm_fn)0x0056adf3U)   /* rebased to 0x0056ADF3 */
#define MIC_AUDM_APP_ID      7u
/* RAM cells read by the codec watchdog (0x5564d2) and the codec-control handler (0x556404):
 *   CODEC_ON_FLAG   u8   1 while the on-path completed; the one-shot check only runs if set
 *   DMA_FRAME_COUNT u32  fresh 50 ms DMA frames consumed since codec-on (check needs >= 20)
 *   I2S_INIT / CODEC_POWER u8  driver state; SYNC_FLAG u8 master/slave boot-sync state
 *   APP_SLOTS       u8[8] service.audio.manager app table (LEFT only): release shuts the
 *                   hardware down when every slot is 0. Slot 7 is unused by stock apps (1..6). */
#define CODEC_ON_FLAG        ((volatile uint8_t *)0x20077470U)
#define DMA_FRAME_COUNT      ((volatile uint32_t *)0x20076e48U)
#define CODEC_I2S_INIT       ((volatile uint8_t *)0x200773d7U)
#define CODEC_POWER_ON       ((volatile uint8_t *)0x200773d6U)
#define AUDIO_SYNC_FLAG      ((volatile uint8_t *)0x20077451U)
#define AUDIO_APP_SLOTS      ((volatile uint8_t *)0x200763b8U)
#define AUDIO_APP_SLOT_CFW   7u
/* 2.2.10.19: phone-link connection parameters as granted by the central. The slave
 * connection context pointer lives at 0x200765b8; the update-event handler (0x47cc08)
 * stores the granted interval (+0x18, 1.25 ms units), latency (+0x1a), supervision
 * timeout (+0x1c, 10 ms units) and profile mode (+0x1e, 0xa3 fast / 0xa4 slow) there.
 * 0x200773a5 is the fast-mode-in-effect flag (set by the update-event handler when the
 * central grants the fast profile, cleared when it leaves it; 2.2.10.19-.23 mislabelled
 * it "central connected"). Read-only diagnostics. */
#define BLE_CONN_CTX_PTR     ((volatile uint32_t *)0x200765b8U)
#define BLE_FAST_MODE_FLAG   ((volatile uint8_t *)0x200773a5U)
#define BLE_MODE_FAST        0xA3u
/* 2.2.10.26: request-path state cells, read-only, appended to the 'ML' record so the phone
 * can see whether a fast request was made, deferred, capped or accepted:
 *   0x20004d7f last mode handed to _connectParamReq_impl (0xa3/0xa4)
 *   0x20004d80 mode currently accepted (written by the update-event handler 0x47cc08)
 *   0x20004d82 mode the 45 s rate limiter is holding for its retry timer
 *   0x20004d83 mode recorded by the request setter 0x47b922
 *   0x200773a4 fast-budget link bitmask (bit per link, cap 2; 0x47b61e/0x47b736)
 *   0x200773a7 rate-limiter pending flag (0x200765c0 its timestamp)
 *   0x200765c4 request counter (0x47b922 increments it on every request) */
#define BLE_REQ_MODE_LAST    ((volatile uint8_t *)0x20004d7fU)
#define BLE_MODE_ACCEPTED    ((volatile uint8_t *)0x20004d80U)
#define BLE_MODE_DEFERRED    ((volatile uint8_t *)0x20004d82U)
#define BLE_MODE_SETTER      ((volatile uint8_t *)0x20004d83U)
#define BLE_FAST_BUDGET_MASK ((volatile uint8_t *)0x200773a4U)
#define BLE_REQ_PENDING      ((volatile uint8_t *)0x200773a7U)
#define BLE_REQ_COUNTER      ((volatile uint32_t *)0x200765c4U)
/* 2.2.10.24: request the fast connection profile ourselves. Disassembly (2026-09-14) of the
 * 2.2.10.10 connection-parameter path showed stock only asks the central for the fast
 * profile when (a) its streaming dispatcher (_dispatchMsgTxByBle 0x47e5de) is backpressured
 * (ESS queue deep) or (b) the phone sends the protobuf device-config ConnParams.setSpeed =
 * FAST, or (c) OTA / image transfer paths. Our array stream never backpressures, so no
 * request was ever made and every candidate measured the Mac's 30 ms default. This seam is
 * the setter the protobuf handler (0x4d3eba) calls: it records the requested mode (0xa3),
 * (re)starts the 0 ms request timer whose callback (0x47c710) posts event 0xb9 to the BLE
 * task, and the task-side _connectParamReq_impl (0x47c0e8) issues DmConnUpdate with the
 * fast table (15/15 ms on this candidate). Argument 0 = keep fast (no 60 s auto-slow),
 * exactly what the protobuf handler passes. It only touches flags and an app timer, so it
 * is safe from the settings-write context that runs mic_apply_control. ABI (uint32
 * auto_slow) inferred from the protobuf handler (`movs r0,#0; bl`) and the 60 s timer arm
 * behind a nonzero argument. 2.2.9.22 literal, rebased to 0x0047D429 by the profile. */
typedef void (*conn_speed_fn)(uint32_t auto_slow);
#define FW_CONN_FAST         ((conn_speed_fn)0x0047d429U)
/* 2.2.10.50: Cordio DmSetPhy(connId, allPhys, txPhys, rxPhys, phyOptions) and the phone
 * connection object (connId byte at +4; the stock conn-param request reads the same slot). */
typedef void (*dm_set_phy_fn)(uint8_t conn_id, uint8_t all_phys, uint8_t tx_phys, uint8_t rx_phys, uint16_t phy_options);
#define FW_DM_SET_PHY        ((dm_set_phy_fn)0x004ddb05U)
#define PHONE_CONN_OBJ_PTR   ((volatile uint32_t *)0x20076594U)
/* 2.2.10.72: the stock ble_msgtx control block (queue handle at +0xc). The stock enqueue drops a
 * streaming message silently once the 150-entry queue is half full and still returns success,
 * so the relay could not tell it was losing frames under load. Read the FreeRTOS queue depth
 * (Queue_t.uxMessagesWaiting at +0x38) and skip a frame while the queue is near that watermark. */
#define BLE_MSGTX_CTRL       ((volatile uint32_t *)0x20004780U)
#define BLE_QUEUE_DEPTH_OFF  0x38u
#define RELAY_NOTIFY_MAX_DEPTH 48u
static uint32_t ble_notify_queue_depth(void) {
    uint32_t h = BLE_MSGTX_CTRL[3];
    if ((h >> 24) != 0x20u) return 0u;
    return *(volatile uint32_t *)(h + BLE_QUEUE_DEPTH_OFF);
}
/* 2.2.10.69: LE Data Length Extension on the phone link. The stock host never raises its own
 * TX octets (HciLeSetDataLen's only caller is the ring link), so every notify the temple sends
 * (mic frames, image ACKs, status) leaves as 27-byte LL PDUs: a 237-byte relay notify is nine
 * air packets. HciLeSetDataLen(handle, txOctets, txTime) is the Cordio HCI wrapper marked by
 * the unique `movw #0x2022` (LE Set Data Length opcode); the connection handle comes from the
 * DM connection control block the DmSetPhy wrapper reads (dmConnCcb lookup by connId, handle
 * u16 at +0xc). The controller clamps to what the central negotiated; a refusal is harmless. */
typedef void (*hci_set_data_len_fn)(uint16_t handle, uint16_t tx_octets, uint16_t tx_time);
typedef void *(*dm_conn_ccb_fn)(uint8_t conn_id);
#define FW_HCI_LE_SET_DATA_LEN ((hci_set_data_len_fn)0x00543997U)
#define FW_DM_CONN_CCB         ((dm_conn_ccb_fn)0x004cca83U)
#define DLE_TX_OCTETS          251u
#define DLE_TX_TIME_US         2120u
/* 2.2.10.51: stock common-data handler for inter-temple frame 0x010C (audio-manager peer
 * sync: DMIC open/close + init handshake). Reached only through the rodata dispatch table
 * entry {0x010C, handler, 0} (2.2.10.10 0x6c12a0 / 2.2.9.22 0x6be5b0), which the patcher
 * repoints at mic_peer_sync_hook below. */
/* 2.2.10.65: the common-data dispatch calls a registered handler as
 * handler(uint32_t recordId, const uint8_t *data, uint32_t length, void *entry) -- confirmed by
 * disassembly of the stock 0x010C handler (2.2.10.10 0x56b1f6 / 2.2.9.22 0x0056b1f6: r1=data,
 * r2=length). The .51-.64 hook took (data, len) and so read the record id as its data pointer,
 * which is why relay control packets were never recognised (ctlRx stayed 0). */
typedef uint32_t (*common_data_fn)(uint32_t record_id, const uint8_t *data, uint32_t len, void *entry);
#define FW_AUDM_PEER_MSG_ORIG ((common_data_fn)0x0056b1f7U)
/* 2.2.10.62: inter-temple common-data send, as used by the audio manager's peer-sync sender
 * (2.2.10.10 0x46b0ac, 2.2.9.22 0x0046b0ac, identical bodies): send(record_id, buf, len, 0). */
typedef int (*peer_send_fn)(uint32_t record_id, const void *buf, uint32_t len, uint32_t opt);
#define FW_PEER_SEND         ((peer_send_fn)0x0046b0adU)
/* 2.2.10.62: stock LC3 wrapper SVC_Lc3EncodeMono (2.2.10.10 0x595d3c, 2.2.9.22 0x00595d3c):
 * encode(pcm, bytes, out, &written, ctx); ctx = {u8 fmt; u32 dt_us; u32 sr_hz; u32 nch; u32 ch;
 * u32 bitrate; void *enc; u8 mem[]} — the encoder is created lazily inside mem when enc == 0.
 * Interleaved input: bytes must be a whole number of frames of (samples * nch * 2). */
typedef int (*lc3_encode_fn)(const void *pcm, uint32_t bytes, void *out, int32_t *written, void *ctx);
#define FW_LC3_ENCODE_MONO   ((lc3_encode_fn)0x00595d3dU)
static int relay_on(const customCfwContext *ctx);
static void relay_tap(customCfwContext *ctx, const void *pcm, uint32_t bytes);
/* Future validation-gate seam, unused until its ABI is confirmed: on-device LC3
 * (SVC_Lc3EncodeMono, "encodes one or more mono or interleaved PCM
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
/* bit2: arm through the audio manager's codec-control primitive instead of the production
 * test wrappers (see FW_CODEC_CTRL), defuse the one-shot codec watchdog once frames flow,
 * and hold audio-manager app slot 7 on the LEFT so a stock release cannot power the codec
 * down under the tap. Phone opt-in; the production path stays the default. */
#define MIC_FLAG_MGR_CTRL    0x04u
/* 2.2.10.50: bit3 asks the temple to request the LE 2M PHY on its phone link when the session
 * arms. Stock never requests a PHY change on the phone link (RING_SetPhyProcess only touches
 * the ring link, DmSetPhy 0x4ddb04 has that single caller), so a 2M-capable central that does
 * not initiate the update itself stays on 1M for the life of the connection. 2M halves on-air
 * time per byte for the array stream (radio energy + notify-queue headroom). */
#define MIC_FLAG_PHY_2M      0x08u
/* 2.2.10.52: bit4 makes the RIGHT ignore inter-temple audio-sync frames while armed (the .51
 * behaviour); clear = stock handling. Phone-switchable so both can be measured on one build. */
#define MIC_FLAG_RIGHT_DEAF  0x10u
#define MIC_FLAG_DLE         0x80u   /* 2.2.10.70: request LE data length 251 on the phone link (bit 5 was
                                       * taken by the internal MIC_FLAG_UNREG_PENDING, so .69 lost it) */
/* Internal state kept in the high bits of ctx->mic_flags (masked out of every echo):
 * WE_POWERED — this session posted codec ctrl(1) itself, so stop may post ctrl(0);
 * UNREG_PENDING — stop has silenced the tap and is waiting for the audio thread to
 * process ctrl(0) before PCM slot 0 is handed back to the stock fallback. */
#define MIC_FLAG_WE_POWERED    0x40u
#define MIC_FLAG_UNREG_PENDING 0x20u
#define MIC_FLAG_PUBLIC_MASK   (MIC_FLAG_BEAMFORM | MIC_FLAG_ARM_HW | MIC_FLAG_MGR_CTRL | MIC_FLAG_PHY_2M | MIC_FLAG_RIGHT_DEAF | MIC_FLAG_DLE)
#define MIC_STREAM_TRUNC     0x80u  /* stream-frame flags bit: payload was truncated */
/* 2.2.10 overlay: raw PCM16 stereo at 16 kHz is 64 kB/s and a single tap buffer
 * (21 + 800 bytes) already exceeds one BLE notification, so nothing raw ever
 * reached the phone (49 frames counted, none received, 2026-09-13). The tap
 * therefore decimates 2:1 to 8 kHz and packs packet-independent IMA ADPCM, the
 * same wire format the phone already decodes for contract 29 (`micadpcm`):
 * payload = [pred0 s16 LE, idx0 u8, pred1 s16 LE, idx1 u8] + nibble pairs
 * (low nibble channel 0, high nibble channel 1), one pair per output sample. */
#define MIC_CODEC_ADPCM      2u
#define MIC_ADPCM_RATE_100HZ 80u      /* 8 kHz after 2:1 decimation (requested rate < 160) */
/* 2.2.10.15: the phone may request the full 16 kHz capture rate (rate >= 160). Then no
 * decimation is applied and every 16 kHz sample is coded, so the 2x400-byte stereo
 * dispatch (200 samples per channel) becomes a 206-byte payload + 21-byte header =
 * 227 bytes, still one notification at the negotiated MTU 247 (244 usable). Requested
 * rates below 160 keep the 8 kHz decimated form (106-byte payload). Capture itself
 * always runs at 16 kHz; the frame header carries the EFFECTIVE payload rate. */
#define MIC_ADPCM_FULL_RATE_100HZ 160u
#define MIC_ADPCM_MAX_SAMPLES 200u    /* per channel per frame → 206-byte payload */
/* 2.2.10.16: measured on hardware (2026-09-13, 2.2.10.15) the capture dispatch hands the tap
 * one chunk every ~51 ms, i.e. 800 samples per channel, while one frame carries at most 200
 * coded samples per channel. A single frame per chunk therefore truncated 75 % of the audio
 * at 16 kHz (and 50 % at 8 kHz on 2.2.10.14). The tap now SLICES each chunk into as many
 * frames as it needs (two at 8 kHz stereo, four at 16 kHz stereo). Frame flags bits 4..6
 * carry the slice index within the chunk and bit 3 says another slice follows, so the phone
 * pairs left/right slices by index; all slices of one chunk share the same tick. */
#define MIC_STREAM_SLICE_SHIFT 4u
#define MIC_STREAM_SLICE_MASK  0x30u
#define MIC_STREAM_MORE        0x08u
#define MIC_MAX_SLICES         4u
/* Bit 6: this frame was produced by the LEFT temple (FW_MIC_SIDE() == 2). The stock
 * protobuf TX path (Thread_MsgPbTxByBle) drops every settings reply on the left temple
 * ("left can't send pb"), so field 104 and the device-info read-back are always the right
 * temple's; the per-link stream frame is the only place the left can state its side. */
#define MIC_STREAM_LEFT        0x40u

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
#define MIC_APP_ID           0x4643u  /* "FC"-ish CFW owner id (status/lease bookkeeping only) */
/* Owner id the stock codec/PDM front ends register their own callback under
 * (production_codec_mic_func_init at 0x005AB8CA: SVC_PcmAppRegister(0x10B, 0, cb)).
 * The tap re-registers slot 0 under the same id so the stock deinit's
 * SVC_PcmAppUnregister(0x10B, 0) still matches and tears the slot down. */
#define MIC_STOCK_OWNER_ID   0x10Bu

static uint8_t mic_popcount2(uint8_t mask) {
    return (uint8_t)((mask & 1u) + ((mask >> 1) & 1u));
}

/* What capture actually runs at: the recovered init entries take no rate
 * argument, so an armed session is the stock fixed 16 kHz pipeline regardless
 * of the requested rate (see EFFECTIVE vs REQUESTED above). */
static uint16_t mic_effective_rate(const customCfwContext *ctx) {
    /* Capture is fixed at 16 kHz; the advertised effective rate is the PAYLOAD rate the
     * phone will decode (8 kHz decimated unless 16 kHz was requested). */
    return ctx->mic_rate_hz_div >= MIC_ADPCM_FULL_RATE_100HZ
        ? (uint16_t)MIC_ADPCM_FULL_RATE_100HZ : (uint16_t)MIC_ADPCM_RATE_100HZ;
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
/* IMA ADPCM step table and index deltas (standard). */
static const int16_t mic_ima_steps[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230,
    253, 279, 307, 337, 371, 408, 449, 494, 544, 598, 658, 724, 796, 876, 963,
    1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272, 2499, 2749, 3024, 3327,
    3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493, 10442,
    11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794,
    32767 };
static const int8_t mic_ima_index_delta[16] = { -1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8 };

typedef struct { int32_t predictor; int32_t index; } mic_ima_state;

static uint8_t mic_ima_encode(mic_ima_state *st, int32_t sample) {
    int32_t step = mic_ima_steps[st->index];
    int32_t diff = sample - st->predictor;
    uint8_t code = 0;
    if (diff < 0) { code = 8; diff = -diff; }
    int32_t delta = step >> 3;
    if (diff >= step) { code |= 4; diff -= step; delta += step; }
    step >>= 1;
    if (diff >= step) { code |= 2; diff -= step; delta += step; }
    step >>= 1;
    if (diff >= step) { code |= 1; delta += step; }
    st->predictor += (code & 8) ? -delta : delta;
    if (st->predictor > 32767) st->predictor = 32767;
    if (st->predictor < -32768) st->predictor = -32768;
    st->index += mic_ima_index_delta[code & 15];
    if (st->index < 0) st->index = 0;
    if (st->index > 88) st->index = 88;
    return code & 15;
}

/* Pack one frame: `pcm` holds `bytes` of planar stereo PCM16 (channel 0 block
 * then channel 1 block, per the recovered stereo dispatch). Output samples are
 * the mean of each input pair (2:1 decimation), so a 16 kHz block of N samples
 * per channel yields N/2 nibble pairs. Returns payload bytes written. */
/* Pack the slice starting at input sample `first` (per channel) into one frame. */
static uint32_t mic_pack_adpcm(const int16_t *pcm, uint32_t bytes, uint8_t nch, uint8_t decim, uint32_t first, uint8_t *out, uint32_t cap) {
    uint32_t total = bytes / 2u;                 /* int16 samples across channels */
    uint32_t per_ch = nch == 2u ? total / 2u : total;
    if (first >= per_ch) return 0u;
    uint32_t avail = per_ch - first;
    uint32_t out_samples = decim == 2u ? avail / 2u : avail;
    if (out_samples > MIC_ADPCM_MAX_SAMPLES) out_samples = MIC_ADPCM_MAX_SAMPLES;
    if (cap < 6u + out_samples) out_samples = cap > 6u ? cap - 6u : 0u;
    const int16_t *c0 = pcm + first;
    const int16_t *c1 = nch == 2u ? pcm + per_ch + first : pcm + first;
    mic_ima_state s0 = { c0[0], 0 }, s1 = { c1[0], 0 };
    out[0] = (uint8_t)s0.predictor; out[1] = (uint8_t)(s0.predictor >> 8); out[2] = 0;
    out[3] = (uint8_t)s1.predictor; out[4] = (uint8_t)(s1.predictor >> 8); out[5] = 0;
    for (uint32_t i = 0; i < out_samples; i++) {
        int32_t a, b;
        if (decim == 2u) {
            a = ((int32_t)c0[2u * i] + (int32_t)c0[2u * i + 1u]) >> 1;
            b = ((int32_t)c1[2u * i] + (int32_t)c1[2u * i + 1u]) >> 1;
        } else {
            a = c0[i];
            b = c1[i];
        }
        out[6u + i] = (uint8_t)(mic_ima_encode(&s0, a) | (mic_ima_encode(&s1, b) << 4));
    }
    return 6u + out_samples;
}

static void mic_finish_unregister(customCfwContext *ctx);
void mic_watchdog_tick(void *arg);
void mic_settle_tick(void *arg);
static void mic_complete_bringup(customCfwContext *ctx);
#define MIC_PWR_OFF_MS 100u          /* 2.2.10.52: power-off dwell before power-on (as the stock reboot handler) */
#define MIC_SYNC_SETTLE_MS 12000u    /* 2.2.10.57: both temples tap this long after the LEFT's manager acquire re-opened the codecs */
#define MIC_CODEC_SETTLE_MS 8000u    /* 2.2.10.37: chip boot time after the one-shot power-cycle */
#define MIC_I2S_REINIT_GAP_MS 1000u  /* 2.2.10.37: gap between I2S deinit and re-init on the ready chip */
#define MIC_LEFT_EXTRA_MS 6000u      /* 2.2.10.44: LEFT completes its bring-up this long AFTER the right, past the right->left sync */

__attribute__((used, noinline)) void mic_pcm_tap(uint32_t source, const void *pcm, uint32_t bytes) {
    (void)source;
    customCfwContext *ctx = peekCustomCfwContext();
    if (!ctx) return;
    if (!ctx->mic_hw_armed) { mic_finish_unregister(ctx); return; }
    if (!ctx->mic_active || pcm == 0 || bytes == 0) return;
    /* Fail-open: the phone stopped renewing — stop emitting immediately. The
     * watchdog timer does the actual hardware teardown from the timer thread
     * (deinit from inside the capture callback would be re-entrant). */
    /* Fail-open teardown is the RTOS watchdog's job (mic_watchdog_tick); the
     * per-frame tick comparison is not repeated here because the rebased tick
     * cell's unit on 2.2.10 is not yet measured (frames stopped after 49 on
     * 2.2.10.13 while the session still reported armed). Frame headers still
     * carry the tick so the phone can measure it. */

    if (relay_on(ctx)) { if (*CODEC_ON_FLAG) *CODEC_ON_FLAG = 0u; relay_tap(ctx, pcm, bytes); return; }   /* 2.2.10.62 */
    uint8_t nch = mic_effective_channels(ctx);
    int16_t ssr = 0, angle = 0;
    /* Manager mode: the codec driver's one-shot "audio check" (2 s after codec-on) reboots
     * the codec when fewer than 20 fresh DMA frames were consumed; the tap's own work plus a
     * codec UART transaction can make frames stale and trip it into a reboot loop. Frames
     * are demonstrably flowing here, so clear the flag the check keys on. */
    if (*CODEC_ON_FLAG) *CODEC_ON_FLAG = 0u;   /* both modes (2.2.10.23) */
    /* The recovered algo object splits 800 interleaved stereo 16-bit frames;
     * feeding it anything else would return garbage bearings. */
    if ((ctx->mic_flags & MIC_FLAG_BEAMFORM) && nch == 2u && ctx->mic_format == 0u)
        FW_ALGO_PROCESS(pcm, bytes, &ssr, &angle);

    /* Requested rate >= 160 (16 kHz) selects the undecimated form; anything lower keeps 2:1. */
    uint8_t decim = ctx->mic_rate_hz_div >= MIC_ADPCM_FULL_RATE_100HZ ? 1u : 2u;
    uint32_t per_ch = bytes / 2u / (nch == 2u ? 2u : 1u);
    uint32_t slice_in = MIC_ADPCM_MAX_SAMPLES * decim;      /* input samples per slice */
    uint32_t slices = (per_ch + slice_in - 1u) / slice_in;
    if (slices > MIC_MAX_SLICES) slices = MIC_MAX_SLICES;     /* beyond that: truncated + flagged */
    uint8_t *f = (uint8_t *)cfw_malloc(MIC_STREAM_HDR_BYTES + 6u + MIC_ADPCM_MAX_SAMPLES);
    if (!f) return;
    uint32_t tick = FW_MS_TICK;
    uint16_t rate = decim == 2u ? MIC_ADPCM_RATE_100HZ : MIC_ADPCM_FULL_RATE_100HZ;    /* what the payload actually is */
    uint8_t side_left = FW_MIC_SIDE() == 2u ? MIC_STREAM_LEFT : 0u;
    for (uint32_t si = 0; si < slices; si++) {
        uint8_t flags = (uint8_t)(ctx->mic_flags & (MIC_FLAG_BEAMFORM | MIC_FLAG_ARM_HW));   /* internal bits never leave */
        flags |= (uint8_t)((si << MIC_STREAM_SLICE_SHIFT) & MIC_STREAM_SLICE_MASK) | side_left;
        if (si + 1u < slices) flags |= MIC_STREAM_MORE;
        if (si + 1u == MIC_MAX_SLICES && per_ch > slices * slice_in) flags |= MIC_STREAM_TRUNC;
        uint32_t pay = mic_pack_adpcm((const int16_t *)pcm, bytes, nch, decim, si * slice_in, f + MIC_STREAM_HDR_BYTES, 6u + MIC_ADPCM_MAX_SAMPLES);
        if (pay == 0u) break;
        uint16_t seq = (uint16_t)ctx->mic_frames;
        f[0] = MIC_STREAM_MAGIC0; f[1] = MIC_STREAM_MAGIC1; f[2] = MIC_PROTO_VERSION;
        f[3] = flags;
        f[4] = (uint8_t)seq; f[5] = (uint8_t)(seq >> 8);
        f[6] = (uint8_t)tick; f[7] = (uint8_t)(tick >> 8);
        f[8] = (uint8_t)(tick >> 16); f[9] = (uint8_t)(tick >> 24);
        f[10] = nch;
        f[11] = (uint8_t)rate; f[12] = (uint8_t)(rate >> 8);
        f[13] = ctx->mic_format;
        f[14] = MIC_CODEC_ADPCM;
        f[15] = (uint8_t)angle; f[16] = (uint8_t)((uint16_t)angle >> 8);
        f[17] = (uint8_t)ssr;   f[18] = (uint8_t)((uint16_t)ssr >> 8);
        f[19] = (uint8_t)pay;   f[20] = (uint8_t)(pay >> 8);
        FW_AUDIO_NOTIFY(f, MIC_STREAM_HDR_BYTES + pay);
        ctx->mic_frames++;
        ctx->mic_last_tap_tick = FW_MS_TICK;
    }
    FW_FREE(f);
}

/* Bring up this temple's selected front end + channel pair and register the tap.
 * Releases the other front end first so both are never held at once. The codec
 * init's one-byte argument "selects the single or stereo callback"; the
 * boolean encoding (0 = single, 1 = stereo) is inferred — validation-gate item. */
/* Granted phone-link profile byte (0xa3 fast / 0xa4 slow / 0 unknown), read-only. */
static unsigned char mic_link_mode(void) {
    uint32_t cx = *BLE_CONN_CTX_PTR;
    if ((cx >> 24) != 0x20u) return 0;
    return *(volatile uint8_t *)(cx + 0x1eu);
}

/* 2.2.10.24: ask the central for the fast profile when a session arms, and again on RENEW
 * while the granted profile is still not the fast one (the request path has its own
 * 45 s rate limiter and a two-link fast budget, so re-asking is cheap and bounded). */
static void mic_request_fast_link(void) {
    FW_CONN_FAST(0u);
}

/* 2.2.10.51: the RIGHT temple ignores inter-temple audio sync frames while its array session
 * is armed and its codec has been power-cycled. Measured 2026-09-15: every LEFT bring-up
 * that goes through the stock audio manager (acquire) emits a 0x010C sync that makes the
 * RIGHT re-initialise its codec without a rail cycle — which is exactly the noisy state. With
 * the RIGHT deaf to the peer sync, the LEFT is free to use the manager (it is the hardware
 * owner and needs it to stream) without perturbing a clean RIGHT. Every other frame and
 * every unarmed state passes straight through to stock. External linkage so -O2 keeps it
 * (no C caller; the dispatch table points here). */
/* ---------------------------------------------------------------------------------------- */
/* 2.2.10.62: microphone relay. CONFIGURE codec 0 (LC3) selects it. The RIGHT temple encodes
 * its two channels and sends each 50 ms DMA chunk to the LEFT over the common-data link
 * (record 0x010C with a CFW magic byte the stock handler never uses); the LEFT encodes its own
 * pair, pairs the nearest RIGHT chunk by time, and emits one 4-channel notify to the phone. */
#define RELAY_RECORD_ID      0x010Cu
#define RELAY_MAGIC_AUDIO    0xA5u   /* RIGHT -> LEFT: one encoded chunk */
#define RELAY_MAGIC_START    0xA6u   /* LEFT -> RIGHT: {fbytes, left_tick u32} */
#define RELAY_MAGIC_ACK      0xA7u   /* RIGHT -> LEFT: {right_tick u32} */
#define RELAY_MAGIC_STOP     0xA8u   /* LEFT -> RIGHT */
#define RELAY_DT_US          10000u
#define RELAY_SR_HZ          16000u
#define RELAY_FRAME_SAMPLES  160u    /* 10 ms at 16 kHz */
#define RELAY_MAX_FRAMES     5u      /* one 800-sample DMA chunk = 5 LC3 frames */
#define RELAY_MAX_FBYTES     80u     /* 64 kbps */
#define RELAY_ENC_CTX_BYTES  2720u   /* 0x1c header + 2600 encoder + slack */
#define RELAY_RING_N         4u
#define RELAY_ENTRY_HDR      12u     /* valid, fbytes, seq16, left_tick32, rx_tick32 */
#define RELAY_ENTRY_BYTES    (RELAY_ENTRY_HDR + 2u * RELAY_MAX_FRAMES * RELAY_MAX_FBYTES)
#define RELAY_PKT_HDR        10u     /* magic, seq16, left_tick32, nframes, fbytes, flags */
#define RELAY_OUT_BYTES      (MIC_STREAM_HDR_BYTES + 4u + 4u * RELAY_MAX_FRAMES * RELAY_MAX_FBYTES + 16u)
#define RELAY_PAIR_MAX_AGE_MS 250     /* 2.2.10.68: drop a RIGHT chunk not paired within this arrival age */
#define RELAY_START_RETRY_MS 2000u
#define RELAY_START_HEARTBEAT_MS 1000u   /* 2.2.10.66: LEFT re-sends START this often so a reset RIGHT re-syncs */
#define MIC_STREAM_RELAY     0x04u   /* stream-frame flags bit: 4-channel relay payload */

static inline uint32_t relay_rd32(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline void relay_wr32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24); }
static void relay_zero(void *p, uint32_t n) { uint8_t *b = (uint8_t *)p; while (n--) *b++ = 0; }

static int relay_on(const customCfwContext *ctx) { return ctx->mic_codec == MIC_CODEC_LC3; }

#define RELAY_NOTIFY_MAX_FBYTES 53u    /* 2.2.10.67: 4*53 + 25 = 237 B, under the ~244 B ATT notify limit */
static uint8_t relay_fbytes_for(const customCfwContext *ctx) {
    uint32_t br = (uint32_t)ctx->mic_bitrate_100 * 100u;
    if (br < 16000u || br > 64000u) br = 32000u;
    uint32_t fb = br / 800u;                       /* bytes per 10 ms frame */
    if (fb < 20u) fb = 20u;
    if (fb > RELAY_NOTIFY_MAX_FBYTES) fb = RELAY_NOTIFY_MAX_FBYTES;   /* one 4-ch frame per notify */
    return (uint8_t)fb;
}

static void relay_enc_reset(customCfwContext *ctx, uint8_t fb) {
    for (unsigned c = 0; c < 2u; c++) {
        uint8_t *e = (uint8_t *)ctx->relay_enc[c];
        if (!e) continue;
        relay_zero(e, RELAY_ENC_CTX_BYTES);
        e[0] = 0u;                                  /* PCM S16 */
        relay_wr32(e + 0x04, RELAY_DT_US);
        relay_wr32(e + 0x08, RELAY_SR_HZ);
        relay_wr32(e + 0x0c, 2u);                         /* interleaved stereo input */
        relay_wr32(e + 0x10, c);                          /* this context's channel */
        relay_wr32(e + 0x14, (uint32_t)fb * 800u);        /* bitrate */
        relay_wr32(e + 0x18, 0u);                         /* encoder created lazily */
    }
    ctx->relay_fbytes = fb;
}

static int relay_alloc(customCfwContext *ctx) {
    for (unsigned c = 0; c < 2u; c++)
        if (!ctx->relay_enc[c]) ctx->relay_enc[c] = cfw_heap13_malloc(RELAY_ENC_CTX_BYTES);
    if (!ctx->relay_ring) {
        ctx->relay_ring = (uint8_t *)cfw_heap13_malloc(RELAY_RING_N * RELAY_ENTRY_BYTES);
        if (ctx->relay_ring) relay_zero(ctx->relay_ring, RELAY_RING_N * RELAY_ENTRY_BYTES);
    }
    if (!ctx->relay_out) ctx->relay_out = (uint8_t *)cfw_heap13_malloc(RELAY_OUT_BYTES);
    return ctx->relay_enc[0] && ctx->relay_enc[1] && ctx->relay_ring && ctx->relay_out;
}

/* Encode one channel of an interleaved-stereo chunk; returns frames encoded (0 on failure). */
static uint32_t relay_encode_channel(customCfwContext *ctx, unsigned c, const void *pcm, uint32_t bytes, uint8_t *out) {
    uint8_t *e = (uint8_t *)ctx->relay_enc[c];
    if (!e) return 0u;
    relay_wr32(e + 0x10, c);
    int32_t written = 0;
    int r = FW_LC3_ENCODE_MONO(pcm, bytes, out, &written, e);
    uint32_t fb = ctx->relay_fbytes;
    if (r != 0 || written <= 0 || ((uint32_t)written % fb) != 0u) { ctx->relay_enc_fail++; return 0u; }
    return (uint32_t)written / fb;
}

static void relay_send_ctl(uint8_t magic, uint32_t tick, uint8_t fb) {
    uint8_t b[6];
    b[0] = magic; b[1] = fb; relay_wr32(b + 2, tick);
    customCfwContext *ctx = peekCustomCfwContext();
    int r = FW_PEER_SEND(RELAY_RECORD_ID, b, 6u, 0u);
    if (ctx) { ctx->relay_tx_ctl++; if (r == 0) ctx->relay_tx_ctl_ok++; }
}

/* LEFT: store a RIGHT chunk in the ring (called on the common-data worker). */
static void relay_ring_put(customCfwContext *ctx, const uint8_t *pkt, uint32_t len) {
    if (!ctx->relay_ring || len < RELAY_PKT_HDR) return;
    uint8_t nf = pkt[7], fb = pkt[8];
    if (nf == 0u || nf > RELAY_MAX_FRAMES || fb < 20u || fb > RELAY_MAX_FBYTES) { ctx->relay_bad_len++; return; }
    if (len < RELAY_PKT_HDR + 2u * nf * fb) { ctx->relay_bad_len++; return; }
    uint32_t now = FW_MS_TICK;
    uint8_t *best = 0; uint32_t best_age = 0;
    for (unsigned i = 0; i < RELAY_RING_N; i++) {
        uint8_t *en = ctx->relay_ring + i * RELAY_ENTRY_BYTES;
        if (!en[0]) { best = en; break; }
        uint32_t age = now - relay_rd32(en + 8);
        if (!best || age > best_age) { best = en; best_age = age; }
    }
    best[0] = 0u;                                   /* invalid while writing */
    best[1] = fb; best[2] = pkt[1]; best[3] = pkt[2];
    relay_wr32(best + 4, relay_rd32(pkt + 3)); relay_wr32(best + 8, now);
    { const uint8_t *s = pkt + RELAY_PKT_HDR; uint8_t *d = best + RELAY_ENTRY_HDR; uint32_t n = 2u * nf * fb; while (n--) *d++ = *s++; }
    best[0] = nf;                                   /* valid: frame count */
    ctx->relay_rx_pkts++;
}

/* 2.2.10.68: FIFO pairing. The two temples free-run at the same nominal 16 kHz / 50 ms-chunk
 * rate, so instead of translating the RIGHT's tick into the LEFT's domain (fragile across
 * clock drift and START-handshake latency, which left pairing broken), pair each LEFT chunk
 * with the OLDEST un-consumed RIGHT chunk still in the ring, dropping any that has waited past
 * RELAY_PAIR_MAX_AGE_MS by arrival. The residual sub-chunk offset (the RIGHT's stamped capture
 * tick minus the LEFT's now) is reported to the phone, which cross-correlates for the exact
 * beamforming delay. */
static uint8_t *relay_ring_take(customCfwContext *ctx, uint32_t now, int *offset_ms) {
    uint8_t *best = 0; uint32_t best_rx = 0;
    for (unsigned i = 0; i < RELAY_RING_N; i++) {
        uint8_t *en = ctx->relay_ring + i * RELAY_ENTRY_BYTES;
        if (!en[0]) continue;
        uint32_t rx = relay_rd32(en + 8);              /* arrival tick */
        if ((int)(now - rx) > RELAY_PAIR_MAX_AGE_MS) { en[0] = 0u; continue; }   /* stale */
        if (!best || (int)(rx - best_rx) < 0) { best = en; best_rx = rx; }        /* oldest first */
    }
    if (best) *offset_ms = (int)(relay_rd32(best + 4) - now);
    return best;
}

static void relay_tap(customCfwContext *ctx, const void *pcm, uint32_t bytes) {
    if (!relay_alloc(ctx)) return;
    uint8_t fb = ctx->relay_fbytes ? ctx->relay_fbytes : relay_fbytes_for(ctx);
    if (!ctx->relay_fbytes) relay_enc_reset(ctx, fb);
    uint32_t frame_in = RELAY_FRAME_SAMPLES * 2u * 2u;      /* 640 bytes of interleaved stereo */
    uint32_t nf = bytes / frame_in;
    if (nf == 0u || (bytes % frame_in) != 0u) { ctx->relay_bad_len++; return; }
    if (nf > RELAY_MAX_FRAMES) { nf = RELAY_MAX_FRAMES; bytes = nf * frame_in; }
    uint32_t now = FW_MS_TICK;
    uint8_t *o = ctx->relay_out;
    if ((++ctx->relay_stat_ctr % 40u) == 0u) {                    /* 2.2.10.63: BOTH sides emit RS */
        uint8_t st[36];
        st[0] = 'R'; st[1] = 'S'; st[2] = (uint8_t)MIC_PROTO_VERSION; st[3] = (uint8_t)FW_MIC_SIDE();
        st[4] = (uint8_t)ctx->relay_rx_pkts; st[5] = (uint8_t)(ctx->relay_rx_pkts >> 8);
        st[6] = (uint8_t)ctx->relay_tx_pkts; st[7] = (uint8_t)(ctx->relay_tx_pkts >> 8);
        st[8] = (uint8_t)ctx->relay_ctl_rx; st[9] = (uint8_t)(ctx->relay_ctl_rx >> 8);
        st[10] = (uint8_t)ctx->relay_hook_calls; st[11] = (uint8_t)(ctx->relay_hook_calls >> 8);
        st[12] = (uint8_t)ctx->relay_paired; st[13] = (uint8_t)(ctx->relay_paired >> 8);
        st[14] = (uint8_t)ctx->relay_missing; st[15] = (uint8_t)(ctx->relay_missing >> 8);
        st[16] = (uint8_t)ctx->relay_enc_fail; st[17] = (uint8_t)(ctx->relay_enc_fail >> 8);
        st[18] = ctx->relay_started; st[19] = fb;
        st[20] = (uint8_t)ctx->relay_tx_ctl; st[21] = (uint8_t)(ctx->relay_tx_ctl >> 8);
        st[22] = (uint8_t)ctx->relay_tx_ctl_ok; st[23] = (uint8_t)(ctx->relay_tx_ctl_ok >> 8);
        st[24] = ctx->relay_last_rx[0]; st[25] = ctx->relay_last_rx[1];
        st[26] = ctx->relay_last_rx[2]; st[27] = ctx->relay_last_rx[3];
        st[28] = (uint8_t)ctx->relay_last_rx_len; st[29] = (uint8_t)(ctx->relay_last_rx_len >> 8);
        st[30] = (uint8_t)ctx->relay_bad_len; st[31] = (uint8_t)(ctx->relay_bad_len >> 8);
        /* 2.2.10.72 (v3, 36 bytes): notifies skipped for queue back-pressure, current depth. */
        st[32] = (uint8_t)ctx->relay_notify_skipped; st[33] = (uint8_t)(ctx->relay_notify_skipped >> 8);
        { uint32_t d = ble_notify_queue_depth(); st[34] = (uint8_t)(d > 255u ? 255u : d); }
        st[35] = (uint8_t)ctx->gate_timeouts;
        FW_AUDIO_NOTIFY(st, 36u);
    }
    if (FW_MIC_SIDE() == 1u) {                                  /* RIGHT: encode + send to LEFT */
        if (!ctx->relay_started) return;                         /* wait for START */
        uint8_t *d0 = o + RELAY_PKT_HDR, *d1 = d0 + nf * fb;
        if (relay_encode_channel(ctx, 0u, pcm, bytes, d0) != nf) return;
        if (relay_encode_channel(ctx, 1u, pcm, bytes, d1) != nf) return;
        o[0] = RELAY_MAGIC_AUDIO;
        o[1] = (uint8_t)ctx->relay_seq; o[2] = (uint8_t)(ctx->relay_seq >> 8);
        relay_wr32(o + 3, now + (uint32_t)ctx->relay_offset_ticks);   /* in the LEFT's tick domain */
        o[7] = (uint8_t)nf; o[8] = fb; o[9] = 0u;
        if (FW_PEER_SEND(RELAY_RECORD_ID, o, RELAY_PKT_HDR + 2u * nf * fb, 0u) == 0) ctx->relay_tx_pkts++;
        ctx->relay_seq++;
        ctx->mic_frames++;
        ctx->mic_last_tap_tick = now;
        return;
    }
    /* 2.2.10.66: LEFT re-issues START as a periodic heartbeat regardless of `relay_started`.
     * A re-CONFIGURE (silent-side retry, lease renew) resets the RIGHT's relay state; the
     * heartbeat re-establishes it within one interval. The RIGHT's START handler is idempotent
     * (it re-ACKs and refreshes the tick offset without re-initialising the encoder), so this
     * costs one 6-byte control packet per second and does not glitch a running stream. */
    if ((now - ctx->relay_start_tick) > RELAY_START_HEARTBEAT_MS) {
        ctx->relay_start_tick = now; relay_send_ctl(RELAY_MAGIC_START, now, fb);
    }
    uint8_t *pay = o + MIC_STREAM_HDR_BYTES;
    uint8_t *L0 = pay + 4u, *L1 = L0 + nf * fb, *R0 = L1 + nf * fb, *R1 = R0 + nf * fb;
    if (relay_encode_channel(ctx, 0u, pcm, bytes, L0) != nf) return;
    if (relay_encode_channel(ctx, 1u, pcm, bytes, L1) != nf) return;
    int off = 0; uint8_t present = 0u;
    uint8_t *en = relay_ring_take(ctx, now, &off);
    if (en && en[1] == fb && en[0] == nf) {
        const uint8_t *s = en + RELAY_ENTRY_HDR; uint32_t n = 2u * nf * fb; uint8_t *d = R0;
        while (n--) *d++ = *s++;
        en[0] = 0u; present = 1u; ctx->relay_paired++; ctx->relay_last_offset_ms = (int16_t)off;
    } else {
        relay_zero(R0, 2u * nf * fb); ctx->relay_missing++; off = 0;
    }
    if (off > 127) off = 127;
    if (off < -127) off = -127;
    /* 2.2.10.67: the 4-channel LC3 chunk (up to 4*nf*fb bytes) far exceeds one BLE notify's
     * ~244-byte ATT limit, so emit one notify per 10 ms LC3 frame: 21-byte 'SM' header +
     * {nframes=1, fbytes, offset, present} + the four channels' frame f (4*fb bytes). At the
     * default fb=40 that is 185 bytes; capped at fb=53 it is 237. The phone reassembles the
     * stream from consecutive frames. */
    for (uint32_t f = 0; f < nf; f++) {
        uint8_t nb[MIC_STREAM_HDR_BYTES + 4u + 4u * RELAY_NOTIFY_MAX_FBYTES];
        uint8_t *np = nb + MIC_STREAM_HDR_BYTES;
        np[0] = 1u; np[1] = fb; np[2] = (uint8_t)(int8_t)off; np[3] = present;
        const uint8_t *chs[4] = { L0 + f * fb, L1 + f * fb, R0 + f * fb, R1 + f * fb };
        for (unsigned c = 0; c < 4u; c++) {
            const uint8_t *sc = chs[c]; uint8_t *dc = np + 4u + (uint32_t)c * fb;
            for (uint32_t k = 0; k < fb; k++) dc[k] = sc[k];
        }
        uint32_t paylen = 4u + 4u * fb;
        uint16_t seq = (uint16_t)ctx->mic_frames;
        nb[0] = MIC_STREAM_MAGIC0; nb[1] = MIC_STREAM_MAGIC1; nb[2] = MIC_PROTO_VERSION;
        nb[3] = (uint8_t)(MIC_FLAG_ARM_HW | MIC_STREAM_LEFT | MIC_STREAM_RELAY);
        nb[4] = (uint8_t)seq; nb[5] = (uint8_t)(seq >> 8);
        relay_wr32(nb + 6, now + f * 10u);                       /* per-frame tick (10 ms apart) */
        nb[10] = 4u;
        nb[11] = (uint8_t)MIC_RATE_DEFAULT; nb[12] = (uint8_t)(MIC_RATE_DEFAULT >> 8);
        nb[13] = 0u;
        nb[14] = MIC_CODEC_LC3;
        nb[15] = 0xFFu; nb[16] = 0x7Fu;
        nb[17] = (uint8_t)off; nb[18] = (uint8_t)((uint16_t)(int16_t)off >> 8);
        nb[19] = (uint8_t)paylen; nb[20] = (uint8_t)(paylen >> 8);
        /* 2.2.10.72: skip (and count) rather than feed a queue the stock enqueue would drop
         * from silently; the phone tracks the sequence gap. */
        if (ble_notify_queue_depth() >= RELAY_NOTIFY_MAX_DEPTH) ctx->relay_notify_skipped++;
        else FW_AUDIO_NOTIFY(nb, MIC_STREAM_HDR_BYTES + paylen);
        ctx->mic_frames++;
    }
    ctx->mic_last_tap_tick = now;
}

/* Relay control/audio packets arriving on record 0x010C. Returns 1 when consumed. */
static int relay_handle_peer(customCfwContext *ctx, const uint8_t *data, uint32_t len) {
    if (!ctx || !data || len < 1u) return 0;
    uint8_t magic = data[0];
    if (magic >= RELAY_MAGIC_AUDIO && magic <= RELAY_MAGIC_STOP) ctx->relay_ctl_rx++;   /* 2.2.10.63 */
    if (magic == RELAY_MAGIC_AUDIO) {
        if (FW_MIC_SIDE() == 2u && relay_on(ctx) && ctx->mic_hw_armed) relay_ring_put(ctx, data, len);
        return 1;
    }
    if (magic == RELAY_MAGIC_START && len >= 6u) {
        if (FW_MIC_SIDE() == 1u) {
            uint8_t fb = data[1]; uint32_t left_tick = relay_rd32(data + 2); uint32_t now = FW_MS_TICK;
            if (!ctx->relay_started) {                              /* first START of this session */
                if (relay_alloc(ctx)) relay_enc_reset(ctx, fb);
                ctx->relay_seq = 0u; ctx->relay_started = 1u;
            }
            ctx->relay_offset_ticks = (int32_t)(left_tick - now);   /* refresh the tick offset */
            relay_send_ctl(RELAY_MAGIC_ACK, now, fb);               /* re-ACK every heartbeat */
        }
        return 1;
    }
    if (magic == RELAY_MAGIC_ACK) {
        if (FW_MIC_SIDE() == 2u) ctx->relay_started = 1u;
        return 1;
    }
    if (magic == RELAY_MAGIC_STOP) {
        if (FW_MIC_SIDE() == 1u) ctx->relay_started = 0u;
        return 1;
    }
    return 0;
}

uint32_t mic_peer_sync_hook(uint32_t record_id, const uint8_t *data, uint32_t len, void *entry) {
    { customCfwContext *hc = peekCustomCfwContext();
      if (hc) { hc->relay_hook_calls++;
        hc->relay_last_rx_len = (uint16_t)len;
        for (unsigned i = 0; i < 4u; i++) hc->relay_last_rx[i] = (i < len && data) ? data[i] : 0u; } }   /* 2.2.10.64 */
    if (relay_handle_peer(peekCustomCfwContext(), data, len)) return 0u;
    customCfwContext *ctx = peekCustomCfwContext();
    /* 2.2.10.61: with bit4 set, BOTH temples ignore inter-temple audio syncs while armed — the
     * LEFT from the moment it arms (fixed bench: the RIGHT's prep/I2S-on sends a sync that
     * re-initialises the LEFT's codec under the live tap and pulls the stock voice app back in;
     * with the RIGHT silent the LEFT stayed 80/0 clean), the RIGHT only once it has started
     * tapping (it still needs the LEFT's re-open to get its capture running). */
    if (ctx && (ctx->mic_flags & MIC_FLAG_ARM_HW) && (ctx->mic_flags & MIC_FLAG_RIGHT_DEAF) && ctx->mic_hw_armed) {
        int deaf = (FW_MIC_SIDE() == 2u) || (ctx->mic_last_tap_tick != 0u);
        if (deaf) { ctx->mic_peer_sync_ignored++; return 0u; }
    }
    return FW_AUDM_PEER_MSG_ORIG(record_id, data, len, entry);
}

/* 2.2.10.50: request LE 2M on the phone link (tx and rx), no coded-PHY options. Only when the
 * phone asked for it (MIC_FLAG_PHY_2M) so the phone can A/B it. The central may refuse. */
static void mic_request_phy_2m(customCfwContext *ctx) {
    if (!(ctx->mic_flags & MIC_FLAG_PHY_2M)) return;
    uint32_t obj = *PHONE_CONN_OBJ_PTR;
    if ((obj >> 24) != 0x20u) return;
    uint8_t cid = *(volatile uint8_t *)(obj + 4u);
    if (cid == 0u || cid > 4u) return;
    FW_DM_SET_PHY(cid, 0u, 0x02u, 0x02u, 0u);
}

/* 2.2.10.69: ask the controller for 251-byte TX PDUs on the phone link. Issued on every
 * CONFIGURE (the phone sends one at connect, before any mic session) and again at session
 * start; the LL length procedure is idempotent. */
static void mic_request_dle(customCfwContext *ctx) {
    if (!(ctx->mic_flags & MIC_FLAG_DLE)) return;
    uint32_t obj = *PHONE_CONN_OBJ_PTR;
    if ((obj >> 24) != 0x20u) return;
    uint8_t cid = *(volatile uint8_t *)(obj + 4u);
    if (cid == 0u || cid > 4u) return;
    uint8_t *ccb = (uint8_t *)FW_DM_CONN_CCB(cid);
    if (ccb == 0) return;
    uint16_t handle = (uint16_t)(ccb[0xc] | ((uint16_t)ccb[0xd] << 8));
    if (handle == 0u || handle > 0x0effu) return;
    FW_HCI_LE_SET_DATA_LEN(handle, DLE_TX_OCTETS, DLE_TX_TIME_US);
    ctx->mic_dle_requests++;
}

static void mic_session_start(customCfwContext *ctx) {
    if (!(ctx->mic_flags & MIC_FLAG_ARM_HW)) return;
    ctx->mic_armed_tick = FW_MS_TICK;
    ctx->mic_last_tap_tick = 0u;
    mic_request_fast_link();
    mic_request_phy_2m(ctx);
    mic_request_dle(ctx);
    if (relay_on(ctx)) {                                          /* 2.2.10.62 */
        ctx->relay_started = 0u; ctx->relay_seq = 0u;
        ctx->relay_tx_pkts = ctx->relay_rx_pkts = ctx->relay_paired = ctx->relay_missing = 0u;
        ctx->relay_enc_fail = ctx->relay_bad_len = 0u; ctx->relay_last_offset_ms = 0;
        ctx->relay_ctl_rx = 0u; ctx->relay_hook_calls = 0u; ctx->relay_stat_ctr = 0u;
        ctx->relay_tx_ctl = 0u; ctx->relay_tx_ctl_ok = 0u; ctx->relay_last_rx_len = 0u;
        if (relay_alloc(ctx)) relay_enc_reset(ctx, relay_fbytes_for(ctx));
        if (FW_MIC_SIDE() == 2u) { ctx->relay_start_tick = FW_MS_TICK; relay_send_ctl(RELAY_MAGIC_START, ctx->relay_start_tick, ctx->relay_fbytes); }
    }
    /* 2.2.10.23: the codec source always arms the stock way, whatever bit2 says. Disassembly
     * (2026-09-14) showed the production-test wrappers do nothing the stock acquire path
     * does not, except open a recording file and post codec ctrl(1) unconditionally; and
     * stock NEVER posts ctrl(1) to a codec that is already on. On the left temple the codec
     * is normally already on (the phone keeps the stock mic enabled), and re-sending
     * DMIC-open + I2S-on to the running GX8002 with I2S init skipped is the only
     * stock-impossible sequence we emitted — the prime suspect for the full-scale noise the
     * stock path produced afterwards. */
    if (ctx->mic_source == MIC_SRC_CODEC) {
        ctx->mic_flags &= (uint8_t)~(MIC_FLAG_WE_POWERED | MIC_FLAG_UNREG_PENDING);
        if (ctx->mic_watchdog_timer) FW_TIMER_STOP(ctx->mic_watchdog_timer);
        /* Manager-style arming: take slot 0 first (so the stock production callback never
         * sees a frame), mark the LEFT audio manager busy, then ask the audio thread to
         * open the DMIC and start the I2S DMA exactly as a stock acquire does. */
        ctx->mic_hw_armed = 1;
        /* 2.2.10.30: hold an audio-manager app-slot on the LEFT while we tap. The manager powers
         * the codec down only when EVERY app-slot reads 0 (release path). The app keeps the stock
         * codec up (setMicEnabled true → stock voice path holds a slot, I2S initialised, clean),
         * then our tap steals PCM slot 0 so the stock LC3 encoder stops getting frames. The stock
         * voice path, starved of LC3, then RELEASES its slot; with no slot of our own held that
         * dropped every slot to 0 and the manager tore the codec down (I2S 1→0, measured
         * 2.2.10.29: pre-arm i2s=1, armed i2s=0). Holding slot 7 keeps a client registered so the
         * manager keeps the codec — and its live, clean I2S — up for the life of the tap. Slots
         * 1..6 are stock; 7 is unused. The RIGHT temple has no audio-manager, so it brings its own
         * codec up (prep+ctrl) and holds it (nothing tears it down; the tap suppresses the 2 s
         * codec-audio-check by clearing CODEC_ON_FLAG). */
        /* 2.2.10.33: reboot the GX8002 only on the temple that needs it. 2.2.10.31 showed a
         * codec-chip reboot (op 2) un-wedges a codec stuck streaming full-scale noise (the RIGHT
         * temple went clean, RMS ~0.005), but a reboot also fires the boot-time inter-temple sync
         * that CLOSES the other temple's DMIC, so rebooting both broke whichever temple armed
         * before the last reboot. The RIGHT was the wedged one; the LEFT produces audio from a
         * plain bring-up. So: RIGHT reboots its GX8002 then brings it up; the LEFT skips the reboot
         * and just brings its codec up. The phone arms RIGHT first, then LEFT, so the LEFT — armed
         * last — reopens its own DMIC after the RIGHT's reboot-close has run, and nothing reboots
         * after it. Both then run clean and independently, exactly the per-temple capture the phone
         * merges and beamforms. CODEC_ON_FLAG is cleared so the 2 s codec-audio-check does not
         * reboot-loop while the codec settles. */
        /* 2.2.10.35: ONE-SHOT GX8002 power-cycle per host boot, then plain bring-ups.
         * The .31-.34 sweep established the facts: the GX8002 voice codec produces clean audio
         * only after its power-cycle (codec-control op 2: power off, 100 ms, power on, re-init);
         * a plain configure + I2S enable from cold always yields the chip's free-running
         * full-scale noise (.34), but a plain bring-up RIGHT AFTER a power-cycle is clean (.33).
         * One temple's power-cycle also cleans the other via the inter-temple codec sync. The
         * only cost is that the ~1.75 s blocking reboot overruns the phone-granted 720 ms link
         * supervision, so the rebooting temple's link is rebuilt — and the phone then re-arms
         * (it treats an armed link rebuild as "pair rebooted; re-arming").
         * So: cycle the codec exactly once per host boot, on the first arm. That arm may cost a
         * link rebuild; the re-arm finds mic_codec_cycled set and does a plain bring-up on a
         * freshly-cycled, clean codec — both temples, no further reboots, no further link
         * disruption for the life of the boot. mic_codec_cycled lives in the reserved-RAM
         * context, so it survives link rebuilds and resets only with the host. On a CFW build
         * the four-mic array is the sole microphone, so nothing else ever re-wedges the codec. */
        if (!ctx->mic_codec_cycled) {
            ctx->mic_codec_cycled = 1u;
            /* 2.2.10.45: emit an audio-path marker so the phone can confirm the once-per-boot
             * power-cycle actually ran (a stale reused context would skip it silently). */
            { unsigned char mk[4] = { 'M','K', (unsigned char)MIC_PROTO_VERSION, (unsigned char)FW_MIC_SIDE() };
              FW_AUDIO_NOTIFY(mk, 4u); }
            /* 2.2.10.52: NON-BLOCKING rail cycle. ctrl(2) blocks this (BLE message) thread through
             * the off-dwell and the codec's boot handshake; every RIGHT link drop landed ~0.8-1 s
             * after CONFIGURE — the central's supervision window — so the blocking reboot is the
             * prime suspect for the drops. Power off here; the settle timer powers on after the
             * off-dwell (stage 3), waits out the chip boot (stage 1 -> I2S deinit), then re-inits
             * I2S on the calibrated chip and taps (stage 2 -> mic_complete_bringup). */
            /* 2.2.10.54: back to the stock blocking reboot, RIGHT only. The staged power-off /
             * power-on of .52/.53 never came up clean on either temple: ctrl(2) re-initialises
             * I2S immediately after power-on and the GX8002 evidently needs that clock to boot
             * and run its MicDelay calibration; with I2S off until the settle expired, the chip
             * booted and calibrated only when we tapped it (noisy). The link drops that made
             * .44+ cycle both temples were a bench artifact (the arm raced the pair reboot), so
             * the LEFT returns to the plain deferred bring-up that measured 110/0 clean in
             * .40/.41: the RIGHT's reboot fires the right->left audio sync, whose stock
             * DMIC close/reopen re-initialises the LEFT's codec through the manager it owns. */
            /* 2.2.10.57: NO rail reboot on either temple. Fixed-bench measurements: LEFT plain
             * (stock-driven init, late tap) = 99/0 clean; every temple that ran the CFW reboot
             * (RIGHT in .56, both in .56) streamed noise despite the settle. The RIGHT's capture
             * showed dma=0 whenever the LEFT never touched the manager: the phone's stock-mic
             * disable is the LEFT's last release, which closes the codec/PDM paths and tells the
             * RIGHT to close its DMIC. So the LEFT re-opens everything the stock way — a manager
             * acquire (first acquire enables PDM + codec and sends the peer re-open) — and both
             * temples tap only after the stock init has had its calibration window. */
            if (FW_MIC_SIDE() == 2u) FW_AUDM_ACQUIRE(MIC_AUDM_APP_ID);
            /* 2.2.10.36: after its power-cycle the GX8002 runs its post-power-on MicDelay
             * calibration (the driver retries it up to three times). Tapping the DMA while it
             * calibrates breaks the calibration and the codec streams noise for the whole
             * session (measured: a sustained arm right after the power-cycle stayed noisy for
             * 100 s, while arms that came a settle period later were clean). So leave the codec
             * running UNDISTURBED — PCM slot 0 empty, the stock fallback consuming, no tap — for a
             * settle window, and only then complete the bring-up and register the tap. Any arm
             * (this one or a re-arm after the link rebuild the power-cycle costs) that lands
             * before the codec is ready defers to the same ready tick. */
            /* 2.2.10.44: BOTH temples rail-cycle (ctrl 2) — required for clean audio, and doing
             * it on both is what keeps the RIGHT's phone link alive (a right-only cycle dropped it
             * every time; the inter-temple sync coordinates them). But the RIGHT's cycle fires the
             * right->left audio sync (close / low-power / reopen), which killed the LEFT whenever
             * the LEFT finished its own bring-up at the same time (.38/.39: RIGHT clean, LEFT 0).
             * So the LEFT completes its bring-up MIC_LEFT_EXTRA_MS later — after the right's cycle
             * and that sync have fully run — while the RIGHT completes at the normal settle. The
             * phone arms right-first, so the right's cycle leads. */
            ctx->mic_codec_ready_tick = FW_MS_TICK + MIC_SYNC_SETTLE_MS;
        }
        {
            uint32_t now = FW_MS_TICK;
            if (ctx->mic_codec_ready_tick && (int32_t)(ctx->mic_codec_ready_tick - now) > 0) {
                uint32_t wait = ctx->mic_codec_ready_tick - now;
                if (ctx->mic_settle_timer == 0)
                    ctx->mic_settle_timer = FW_TIMER_NEW((void *)&mic_settle_tick, 0, ctx, 0);
                if (ctx->mic_settle_timer) {
                    ctx->mic_settle_stage = 1u;
                    FW_TIMER_STOP(ctx->mic_settle_timer);
                    FW_TIMER_START(ctx->mic_settle_timer, wait ? wait : 1u);
                    ctx->mic_stage_deadline_tick = FW_MS_TICK + wait + 3000u;
                    return;                          /* bring-up completes in mic_settle_tick */
                }
            }
        }
        mic_complete_bringup(ctx);
        return;
    }
    if (ctx->mic_source == MIC_SRC_PDM) {
        FW_CODEC_MIC_DEINIT();
        FW_PDM_MIC_INIT(0);                 /* PDM init has only the single callback */
    } else {
        FW_PDM_MIC_DEINIT();
        FW_CODEC_MIC_INIT(mic_effective_channels(ctx) == 2u ? 1u : 0u);
    }
    /* The front-end init above registered the stock callback in slot 0 and
     * started capture; take the slot over (same owner id) so the dispatcher
     * hands every buffer to the tap instead. */
    FW_PCM_REGISTER(MIC_STOCK_OWNER_ID, MIC_PCM_SLOT, (void *)&mic_pcm_tap);
    ctx->mic_hw_armed = 1;
}

/* 2.2.10.36: the codec bring-up proper — configure, enable I2S, suppress the codec-audio-check,
 * and only then take PCM slot 0 with the tap. Called directly when the codec is ready, or from
 * mic_settle_tick after the one-shot power-cycle's calibration window. */
/* 2.2.10.60: while the array is armed, no STOCK application may hold the audio manager.
 * Fixed-bench status traces: every noisy LEFT window showed a stock slot (app 5, the stock
 * voice/LC3 capture) alongside ours (slots=0xa0), every clean window showed ours alone or none
 * (0x80 / 0x00). The stock app re-initialises the codec/I2S path under the live tap, which is
 * the misaligned-word "noise". On a CFW build the array is the only microphone, so the stock
 * capture is simply released whenever it shows up (bring-up, and every renew/status). */
static void mic_release_stock_apps(customCfwContext *ctx) {
    for (unsigned i = 1; i < 7u; i++) {
        if (AUDIO_APP_SLOTS[i]) { FW_AUDM_RELEASE((uint32_t)i); ctx->mic_stock_releases++; }
    }
}

static void mic_complete_bringup(customCfwContext *ctx) {
    mic_release_stock_apps(ctx);
    FW_CODEC_PREP(1u);                              /* configure the GX8002 */
    ctx->mic_flags |= MIC_FLAG_WE_POWERED;
    FW_CODEC_CTRL(1u);                              /* I2S output on + init (no-op if already on) */
    *CODEC_ON_FLAG = 0u;                            /* suppress the 2 s codec-audio-check reboot loop */
    ctx->mic_codec_ready_tick = 0u;                 /* calibrated: later arms tap immediately */
    FW_PCM_REGISTER(MIC_STOCK_OWNER_ID, MIC_PCM_SLOT, (void *)&mic_pcm_tap);
}

/* Settle-timer callback (RTOS timer thread): the power-cycled codec has had its quiet window.
 * Complete the bring-up only if the session is still wanted. */
void mic_settle_tick(void *arg) {
    customCfwContext *ctx = (customCfwContext *)arg;
    if (!ctx || ctx->magic != CFW_CTX_MAGIC) return;
    if (!ctx->mic_hw_armed || !ctx->mic_active) { ctx->mic_settle_stage = 0u; return; }   /* stopped while settling */
    /* 2.2.10.37: every arm that ever went clean had an I2S DEINIT then RE-INIT after the codec's
     * power-cycle (the disarm/re-arm gap in the cycled benches); the one arm that kept I2S
     * continuously on after the power-cycle stayed noisy for its whole window. The power-cycle's
     * own I2S init runs while the GX8002 is still booting, so I2S locks to garbage; re-initialising
     * I2S once the chip is fully up locks it correctly. So: stage 1 (chip booted) → I2S deinit;
     * stage 2 (a short gap later) → configure + I2S re-init on the ready chip, then the tap. */
    if (ctx->mic_settle_stage == 3u) {              /* 2.2.10.52: off-dwell elapsed -> power on */
        FW_CODEC_PWR_ON();
        ctx->mic_settle_stage = 1u;                 /* now waiting for the chip to boot + calibrate */
        {
            uint32_t now = FW_MS_TICK;
            uint32_t wait = (ctx->mic_codec_ready_tick && (int32_t)(ctx->mic_codec_ready_tick - now) > 0)
                ? ctx->mic_codec_ready_tick - now : MIC_CODEC_SETTLE_MS;
            if (ctx->mic_settle_timer) { FW_TIMER_START(ctx->mic_settle_timer, wait ? wait : 1u); ctx->mic_stage_deadline_tick = FW_MS_TICK + wait + 3000u; }
        }
        return;
    }
    /* 2.2.10.59: the RIGHT's bring-up is selectable from the phone through the (otherwise unused
     * for raw PCM) bitrate field, low nibble, so the variants can be measured on one build:
     *   0  prep + I2S on + tap at the settle (the LEFT's clean recipe; .57 RIGHT = noisy)
     *   1  I2S off, prep, wait MIC_CODEC_SETTLE_MS with I2S off, then I2S on + tap
     *   2  I2S off, 1 s, I2S on + tap (no prep)
     *   3  tap only (leave the codec exactly as the manager re-open left it)
     * The LEFT always uses variant 0. */
    if (FW_MIC_SIDE() == 1u && ctx->mic_settle_stage == 1u) {
        uint32_t v = ctx->mic_bitrate_100 & 0xfu;
        if (v == 1u) {
            FW_CODEC_CTRL(0u);
            FW_CODEC_PREP(1u);
            ctx->mic_settle_stage = 2u;
            if (ctx->mic_settle_timer) { FW_TIMER_START(ctx->mic_settle_timer, MIC_CODEC_SETTLE_MS); ctx->mic_stage_deadline_tick = FW_MS_TICK + MIC_CODEC_SETTLE_MS + 3000u; }
            return;
        }
        if (v == 2u) {
            FW_CODEC_CTRL(0u);
            ctx->mic_settle_stage = 2u;
            if (ctx->mic_settle_timer) { FW_TIMER_START(ctx->mic_settle_timer, MIC_I2S_REINIT_GAP_MS); ctx->mic_stage_deadline_tick = FW_MS_TICK + MIC_I2S_REINIT_GAP_MS + 3000u; }
            return;
        }
        if (v == 3u) {
            mic_release_stock_apps(ctx);
            ctx->mic_settle_stage = 0u;
            ctx->mic_codec_ready_tick = 0u;
            ctx->mic_flags |= MIC_FLAG_WE_POWERED;
            *CODEC_ON_FLAG = 0u;
            FW_PCM_REGISTER(MIC_STOCK_OWNER_ID, MIC_PCM_SLOT, (void *)&mic_pcm_tap);
            return;
        }
    }
    if (FW_MIC_SIDE() == 1u && ctx->mic_settle_stage == 2u) {   /* variants 1/2: I2S on + tap, no prep */
        mic_release_stock_apps(ctx);
        ctx->mic_settle_stage = 0u;
        ctx->mic_codec_ready_tick = 0u;
        ctx->mic_flags |= MIC_FLAG_WE_POWERED;
        FW_CODEC_CTRL(1u);
        *CODEC_ON_FLAG = 0u;
        FW_PCM_REGISTER(MIC_STOCK_OWNER_ID, MIC_PCM_SLOT, (void *)&mic_pcm_tap);
        return;
    }
    /* LEFT (stage 1) falls straight to mic_complete_bringup: its codec was re-initialised by the
     * right's inter-temple sync, so it needs only a plain, correctly-timed bring-up. */
    ctx->mic_settle_stage = 0u;
    mic_complete_bringup(ctx);                      /* prep + I2S re-init + tap */
}

/* Hand PCM slot 0 back to the stock fallback. Called from the tap once the audio thread has
 * seen the session go idle, or from the watchdog timer as a fallback when no further frames
 * arrive (codec already off) — the slot must never stay registered, or the stock microphone
 * would be silenced for good. */
static void mic_finish_unregister(customCfwContext *ctx) {
    if (!(ctx->mic_flags & MIC_FLAG_UNREG_PENDING)) return;
    ctx->mic_flags &= (uint8_t)~MIC_FLAG_UNREG_PENDING;
    FW_PCM_UNREGISTER(MIC_STOCK_OWNER_ID, MIC_PCM_SLOT);
}

static void mic_session_stop(customCfwContext *ctx) {
    if (!ctx->mic_hw_armed) return;
    ctx->mic_hw_armed = 0;                  /* silence the tap before teardown */
    if (relay_on(ctx) && FW_MIC_SIDE() == 2u) relay_send_ctl(RELAY_MAGIC_STOP, FW_MS_TICK, ctx->relay_fbytes);   /* 2.2.10.62 */
    ctx->relay_started = 0u;
    if (FW_MIC_SIDE() == 2u) FW_AUDM_RELEASE(MIC_AUDM_APP_ID);   /* 2.2.10.57: LEFT releases its manager slot */
    if (ctx->mic_settle_timer) FW_TIMER_STOP(ctx->mic_settle_timer);   /* 2.2.10.36: cancel a deferred bring-up */
    ctx->mic_settle_stage = 0u;
    if (ctx->mic_source == MIC_SRC_CODEC) {
        /* Power the codec down only if this session powered it up and no stock app holds it
         * (LEFT; the RIGHT has no stock users of its DMIC). */
        uint8_t busy = 0u;
        if (FW_MIC_SIDE() == 2u) for (unsigned i = 1; i < 7u; i++) busy |= AUDIO_APP_SLOTS[i];
        if ((ctx->mic_flags & MIC_FLAG_WE_POWERED) && !busy) FW_CODEC_CTRL(0u);
        ctx->mic_flags &= (uint8_t)~MIC_FLAG_WE_POWERED;
        /* Keep the (now silent) tap in slot 0 until ctrl(0) has been processed, so no
         * stock-format LC3 packet leaks from either temple during the hand-back; the tap
         * itself unregisters on its next call (being called proves it still owns the slot).
         * 2.2.10.25: NO timer fallback. 2.2.10.23's 200 ms watchdog unregister ran after the
         * phone had already re-enabled the stock microphone, whose production init had
         * overwritten slot 0 with the stock callback under the same owner id (0x10B): the
         * blind SVC_PcmAppUnregister(0x10B, 0) then removed the STOCK callback and the stock
         * microphone went silent (measured 2026-09-14 12:24: no audio reached the recognizer
         * after three manager-mode cycles). A lingering tap registration is harmless: with the
         * codec off nothing dispatches to it, and both a stock enable and our next CONFIGURE
         * re-register slot 0 (SVC_PcmAppRegister overwrites an occupied slot). */
        ctx->mic_flags |= MIC_FLAG_UNREG_PENDING;
        return;
    }
    FW_PCM_UNREGISTER(MIC_STOCK_OWNER_ID, MIC_PCM_SLOT);
    FW_CODEC_MIC_DEINIT();
    FW_PDM_MIC_DEINIT();
}

/* Watchdog callback, on the RTOS timer thread (the same context stock deinit
 * paths run in). Tears down an armed session whose lease lapsed; if the lease
 * was renewed since arming, re-arms itself for the remaining time. */
void mic_watchdog_tick(void *arg) {
    customCfwContext *ctx = (customCfwContext *)arg;
    if (!ctx || ctx->magic != CFW_CTX_MAGIC) return;
    if (!ctx->mic_hw_armed) return;          /* 2.2.10.25: never unregister from the timer */
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
    /* 2.2.10.52: per-SESSION gate. A re-arm after a stop or a lapsed lease must rail-cycle again
     * (measured 2026-09-15: every re-arm on a reused context skipped the cycle and stayed noisy
     * for the rest of the run). Renews/retries inside a live session still never re-cycle. */
    ctx->mic_codec_cycled = 0u;
    ctx->mic_codec_ready_tick = 0u;
    if (ctx->mic_watchdog_timer) {
        FW_TIMER_STOP(ctx->mic_watchdog_timer);
        if (FW_TIMER_DELETE(ctx->mic_watchdog_timer) == 0) ctx->mic_watchdog_timer = 0;
    }
    if (ctx->mic_settle_timer) {
        FW_TIMER_STOP(ctx->mic_settle_timer);
        if (FW_TIMER_DELETE(ctx->mic_settle_timer) == 0) ctx->mic_settle_timer = 0;
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
    /* 2.2.10.17: the stock pb TX path drops every settings notification on the LEFT temple
     * ("left can't send pb", Thread_MsgPbTxByBle), so the phone never saw a left-produced
     * field-104 record. Mirror the bare 21-byte status body on the streaming-notify path
     * (no side gate); the phone already routes an 'M','C',1 record arriving on the audio
     * characteristic to its microphone-array status handler, attributed to the link. */
    /* 2.2.10.18: both temples mirror the record on the audio path, with the codec driver's
     * state cells appended so the phone can tell "codec rebooting", "stock app released the
     * codec" and "BLE drops" apart: [21..24] DMA frame count u32, [25] codec-on flag,
     * [26] I2S init, [27] codec power, [28] audio sync flag, [29] app-slot bitmask,
     * [30] this temple's side (1 = right, 2 = left). */
    {
        unsigned char *m = (unsigned char *)cfw_malloc(n + 10u);
        if (m) {
            for (unsigned i = 0; i < n; i++) m[i] = p[7 + i];
            uint32_t frames = *DMA_FRAME_COUNT;
            m[n + 0] = (unsigned char)frames; m[n + 1] = (unsigned char)(frames >> 8);
            m[n + 2] = (unsigned char)(frames >> 16); m[n + 3] = (unsigned char)(frames >> 24);
            m[n + 4] = *CODEC_ON_FLAG;
            m[n + 5] = *CODEC_I2S_INIT;
            m[n + 6] = *CODEC_POWER_ON;
            m[n + 7] = *AUDIO_SYNC_FLAG;
            unsigned char slots = 0u;
            for (unsigned i = 0; i < 8u; i++) if (AUDIO_APP_SLOTS[i]) slots |= (unsigned char)(1u << i);
            m[n + 8] = slots;
            m[n + 9] = (unsigned char)FW_MIC_SIDE();
            FW_AUDIO_NOTIFY(m, n + 10u);
            FW_FREE(m);
        }
    }
    /* 2.2.10.19: link parameters, as a separate 'M','L' record on the audio path:
     * [3] fast-mode flag (2.2.10.24: was mislabelled "connected"), [4..5] interval (1.25 ms
     * units), [6..7] latency, [8..9] timeout (10 ms), [10] mode (0xa3 fast / 0xa4 slow),
     * [11] this temple's side. */
    {
        unsigned char *l = (unsigned char *)cfw_malloc(20u);
        if (l) {
            uint32_t cx = *BLE_CONN_CTX_PTR;
            uint16_t iv = 0, lat = 0, to = 0; unsigned char mode = 0;
            if ((cx >> 24) == 0x20u) {            /* SRAM window sanity check */
                iv = *(volatile uint16_t *)(cx + 0x18u); lat = *(volatile uint16_t *)(cx + 0x1au);
                to = *(volatile uint16_t *)(cx + 0x1cu); mode = *(volatile uint8_t *)(cx + 0x1eu);
            }
            l[0] = 'M'; l[1] = 'L'; l[2] = MIC_PROTO_VERSION;
            l[3] = *BLE_FAST_MODE_FLAG;
            l[4] = (unsigned char)iv; l[5] = (unsigned char)(iv >> 8);
            l[6] = (unsigned char)lat; l[7] = (unsigned char)(lat >> 8);
            l[8] = (unsigned char)to; l[9] = (unsigned char)(to >> 8);
            l[10] = mode; l[11] = (unsigned char)FW_MIC_SIDE();
            /* 2.2.10.26: [12] last requested, [13] accepted, [14] deferred, [15] setter mode,
             * [16] fast-budget mask, [17] rate-limiter pending, [18..19] request counter. */
            l[12] = *BLE_REQ_MODE_LAST; l[13] = *BLE_MODE_ACCEPTED;
            l[14] = *BLE_MODE_DEFERRED; l[15] = *BLE_MODE_SETTER;
            l[16] = *BLE_FAST_BUDGET_MASK; l[17] = *BLE_REQ_PENDING;
            { uint32_t c = *BLE_REQ_COUNTER; l[18] = (unsigned char)c; l[19] = (unsigned char)(c >> 8); }
            FW_AUDIO_NOTIFY(l, 20u);
            FW_FREE(l);
        }
    }
}

/* Parse a field-103 record. Called from faceclaw_scan_settings_control for each
 * sid-0x09 settings WRITE, before the stock decoder runs. */
void mic_apply_control(const uint8_t *data, uint32_t len) {
    if (len < 4u || data[0] != 'M' || data[1] != 'C' ||
        data[2] != MIC_PROTO_VERSION) return;
    customCfwContext *ctx = getCustomCfwContext();
    if (!ctx) return;
    uint8_t op = data[3];
    /* 2.2.10.38: host-reboot detection (see cfw_context.h). A backwards OS tick means the host
     * rebooted since the last control op, so the GX8002 is back in its stock-boot (tap-noisy)
     * state and must be power-cycled again on the next arm. */
    {
        uint32_t now = FW_MS_TICK;
        int stale = 0;
        if (now < ctx->mic_last_seen_tick) stale = 1;                 /* OS tick went backwards */
        /* 2.2.10.53: a host reboot does NOT clear this context (it lives in the reserved heap
         * tail) and the OS tick is not guaranteed to restart from zero, so a session that was
         * mid-bring-up or streaming when the host rebooted comes back looking live: armed,
         * stage != 0 with an RTOS timer handle that no longer exists, or "active" with no tap
         * calls ever arriving. Measured 2026-09-15 (every bench arm that raced the pair reboot):
         * the next CONFIGURE was treated as a renew, nothing re-armed the codec, and the temple
         * streamed the stock-boot (noisy) codec for the rest of the run. Detect both shapes by
         * time and start over with a fresh rail cycle. The old timer handles are dropped, never
         * stopped or deleted: after a reboot they point at freed RTOS memory. */
        if (ctx->mic_hw_armed && !stale) {
            if (ctx->mic_settle_stage != 0u && ctx->mic_stage_deadline_tick
                && (int32_t)(now - ctx->mic_stage_deadline_tick) > 0) stale = 1;
            if (ctx->mic_settle_stage == 0u && (int32_t)(now - ctx->mic_armed_tick) > 20000
                && (ctx->mic_last_tap_tick == 0u || (int32_t)(now - ctx->mic_last_tap_tick) > 3000)) stale = 1;
        }
        if (stale) {
            ctx->mic_recoveries++;
            ctx->mic_settle_timer = 0;                                /* stale after a reboot: never touch */
            ctx->mic_watchdog_timer = 0;
            ctx->mic_settle_stage = 0u;
            ctx->mic_hw_armed = 0;
            ctx->mic_active = 0;
            ctx->mic_lease_deadline = 0;
            ctx->mic_codec_cycled = 0u;
            ctx->mic_codec_ready_tick = 0u;
            ctx->mic_stage_deadline_tick = 0u;
            ctx->mic_flags &= (uint8_t)~(MIC_FLAG_WE_POWERED | MIC_FLAG_UNREG_PENDING);
        }
        ctx->mic_last_seen_tick = now;
    }

    if (ctx->mic_hw_armed && ctx->mic_settle_stage == 0u) mic_release_stock_apps(ctx);   /* 2.2.10.60 */
    if (op == MIC_OP_CONFIGURE) {
        if (len < 4u + 9u) return;
        const uint8_t *c = data + 4;

        /* 2.2.10.39: a CONFIGURE that lands while the one-shot settle is in progress must NOT
         * restart the session. The phone re-CONFIGUREs a silent temple every 3 s (its
         * silent-side retry), and each stop/start cancelled the deferred bring-up and restarted
         * the settle, so the LEFT never completed it and streamed 0 frames (measured .38: three
         * "settle #N re-CONFIGURE left" lines, no left frames) while the RIGHT — armed once and
         * left alone — came up clean. So while settling, treat CONFIGURE as a lease renewal:
         * keep the settle running, refresh the lease, answer with status. */
        if (ctx->mic_hw_armed && ctx->mic_settle_stage != 0u) {
            mic_lease_renew(ctx);
            mic_send_status_notify(ctx);
            return;
        }
        /* 2.2.10.55: a CONFIGURE that repeats the live session's exact configuration is a
         * renewal, not a restart. The phone re-CONFIGUREs on its silent-side retries and after
         * every display-session cleanup; restarting a clean, streaming codec for that (stop ->
         * prep -> immediate I2S init) is exactly the sequence that comes up noisy. */
        if (ctx->mic_hw_armed && ctx->mic_active) {
            uint32_t rate_c = (uint32_t)c[4] | ((uint32_t)c[5] << 8);
            uint32_t br_c = (uint32_t)c[6] | ((uint32_t)c[7] << 8);
            uint8_t src_c = c[0] > MIC_SRC_PDM ? MIC_SRC_CODEC : c[0];
            uint8_t mask_c = (c[1] & 0x03u) ? (c[1] & 0x03u) : 0x01u;
            uint8_t codec_c = c[2] > MIC_CODEC_RAW ? MIC_CODEC_LC3 : c[2];
            uint8_t fmt_c = c[3] > 2u ? 0u : c[3];
            if (rate_c < MIC_RATE_MIN_100HZ) rate_c = MIC_RATE_MIN_100HZ;
            if (rate_c > MIC_RATE_MAX_100HZ) rate_c = MIC_RATE_MAX_100HZ;
            if (br_c > MIC_BR_MAX_100BPS) br_c = MIC_BR_MAX_100BPS;
            if (src_c == ctx->mic_source && mask_c == ctx->mic_chan_mask && codec_c == ctx->mic_codec
                && fmt_c == ctx->mic_format && (uint16_t)rate_c == ctx->mic_rate_hz_div
                && (uint16_t)br_c == ctx->mic_bitrate_100
                && (c[8] & MIC_FLAG_PUBLIC_MASK) == (ctx->mic_flags & MIC_FLAG_PUBLIC_MASK)) {
                mic_lease_renew(ctx);
                mic_send_status_notify(ctx);
                return;
            }
        }

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

        ctx->mic_flags    = c[8] & MIC_FLAG_PUBLIC_MASK;   /* 2.2.10.53: was the 3-bit mask; bits 3/4 never landed */
        ctx->mic_channels = mic_popcount2(ctx->mic_chan_mask);
        ctx->mic_active = 1;
        ctx->mic_frames = 0;
        mic_request_dle(ctx);                /* 2.2.10.69: link-level, independent of the mic session */
        mic_session_start(ctx);              /* no-op unless MIC_FLAG_ARM_HW set */
        mic_lease_renew(ctx);
        mic_send_status_notify(ctx);         /* confirm the applied config */
    } else if (op == MIC_OP_QUERY) {
        mic_send_status_notify(ctx);
    } else if (op == MIC_OP_STOP) {
        mic_cleanup_session();
        mic_send_status_notify(ctx);
    } else if (op == MIC_OP_RENEW) {
        /* 2.2.10.20: a stock app release on the LEFT can power the codec down while the tap is
         * armed (manager mode). If the I2S is no longer initialised, bring it back up. */
        if (ctx->mic_hw_armed && ctx->mic_source == MIC_SRC_CODEC && !*CODEC_I2S_INIT) {
            /* 2.2.10.28: re-bring-up the codec the stock way (prep + ctrl) if a stock release
             * powered it down under the armed tap. */
            FW_CODEC_PREP(1u);
            ctx->mic_flags |= MIC_FLAG_WE_POWERED;
            FW_CODEC_CTRL(1u);
        }
        if (ctx->mic_active) mic_lease_renew(ctx);
        if (ctx->mic_hw_armed && mic_link_mode() != BLE_MODE_FAST) mic_request_fast_link();
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
    body[n++] = (unsigned char)(ctx->mic_flags & MIC_FLAG_PUBLIC_MASK);
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
 * sid-0x09 settings READ response. A missing context reports an all-zero
 * (inactive) body of the same shape so the phone parser never needs a special
 * case. Returns the new message length, or the original length if it will not
 * fit in the settings response buffer. */
unsigned mic_append_status(unsigned char *buf, unsigned len, unsigned capacity) {
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
    return pb_append_bytes_field(buf, len, capacity, 104u, body, n);
}
