#include <stdint.h>
#include "cfw_context.h"

/*
 * als_sensor.c — ambient light sensor access for the G2 CFW (image-handler mode 16).
 *
 * WHAT THE STOCK FIRMWARE DOES (2.2.9.22, recovered from the [sensor_als] driver)
 *
 * The G2 carries a TI OPT3001 ambient light sensor (manufacturer id 0x5449,
 * device id 0x3001, I2C address 0x45). It is owned by the sensor-hub task and is
 * only ever opened on the master lens, and only while the user's auto-brightness
 * setting is on:
 *
 *   auto on  -> FUN_0046fe70 -> FUN_004b80ee(4)  hub "FuncOpen"  type 4 -> FUN_004bf520 (ALS open)
 *   auto off -> FUN_0046ff40 -> FUN_004b8160(4)  hub "FuncClose" type 4 -> FUN_004bf634 (ALS close)
 *
 * ALS open resets the driver state, then arms a hub osTimer (handle @0x20076c48,
 * FUN_004b7b28(ms) start / FUN_004b7b38 stop). The timer callback (FUN_004b7b0c)
 * posts hub message id 8, and the hub task dispatches that through an 8-entry
 * {uint16 id, fn} table at 0x20003d08 (FUN_004b7a82 "HUB_MessageProcesser") to the
 * ALS state machine FUN_004bfb60:
 *
 *   status 1 (start read)  FUN_004bf6e4: read, seed peak/target, APPLY target
 *                          brightness immediately unless a manual level was set,
 *                          then status 3 / 1000 ms.
 *   status 3 (polling)     FUN_004bf980: read every 1000 ms, keep a 5-sample ring,
 *                          peak = max of ring, target = curve(peak) * scale_q10.
 *                          While a manual level is "locked" (settings+4 != 0) it
 *                          does nothing unless the ring spread exceeds 300 or the
 *                          lock is ~12 h old; otherwise target != current flips to
 *                          status 2 / 200 ms.
 *   status 2 (adjust)      FUN_004bf844: step the level by 2 (5 when far off)
 *                          toward target every 200 ms through FUN_004bf2ee ->
 *                          FUN_004beda2 -> message 0x10e to the UI task, which
 *                          reprograms the panel. THIS is the visible stepping /
 *                          flicker of stock auto-brightness.
 *
 * The reading (FUN_004bf482) converts the OPT3001 result register (mantissa <<
 * exponent = lux * 100) into the stock "als value": raw * lux_base / 1e6 when a
 * production lux_base calibration exists in NV (@0x20004014+0x28), else raw / 10.
 * Either way it is roughly tenths of a lux. Readings taken while the IMU pitch is
 * below -30 degrees are discarded (the previous value is reused). The stock
 * brightness curve is a 6-row table @0x755358 of {als threshold, level}:
 * <=10 -> 35, <=200 -> 50, <=400 -> 70, <=1000 -> 70, <=1300 -> 100, else 100,
 * scaled by scale_q10 (learned from the user's manual adjustments, 0x266..0x59a).
 *
 * Driver globals (all on the master lens, all written only by the hub task):
 *   0x200763d4 opened      0x200763d8 status     0x200763f0 als value
 *   0x200763f4 peak        0x200763f8 gear       0x20076400 target level
 *   0x20000068 scale_q10   0x20074c80 settings: +1 brightness level, +2 auto on
 *
 * WHAT THIS EXTENSION ADDS
 *
 * Mode 16 lets the phone read the sensor and, optionally, run the sensor in a
 * "passive" mode where the CFW polls it and the stock adjuster never touches the
 * panel — so the phone can implement its own brightness policy (e.g. change the
 * level only while the display is blank or during a UI transition):
 *
 *   [16][0]                 QUERY: send one report (from the driver's current globals,
 *                           whatever opened/closed state they are in).
 *   [16][1][flags][interval16][min-delta16][heartbeat16]
 *                           PASSIVE START: redirect hub message 8 to als_hub_handler,
 *                           open the ALS if it is closed (auto-brightness off), poll
 *                           every `interval` ms (clamped 100..5000), and send a
 *                           report whenever |value - last reported| >= min-delta or
 *                           `heartbeat` ms elapsed since the last report (0 = never on
 *                           time alone). Idempotent: repeating it updates the
 *                           parameters and re-opens the ALS if stock closed it.
 *                           flags bit0: bind to the Faceclaw framebuffer lease — stop
 *                           automatically when that lease is released or lapses.
 *   [16][2]                 PASSIVE STOP: restore the stock handler; close the ALS if
 *                           the CFW opened it and auto-brightness is still off. If
 *                           auto-brightness is on the stock machine simply resumes.
 *
 * Reports are G2SettingPackage{commandId=3, magic=0, field 105} on sid 0x09 (the
 * same shape as the field-102 wake event and field-104 mic status), from the
 * master lens only. Field 105 body, little-endian:
 *
 *   [0]'A' [1]'L' [2]version=1 [3]reason (0 query, 1 started, 2 poll, 3 stopped)
 *   [4]flags  bit0 ALS opened   bit1 passive hook installed
 *             bit2 auto-brightness setting on   bit3 last passive read succeeded
 *   [5]stock status byte (1 start-read, 2 adjusting, 3 polling)
 *   [6..9]   als value (uint32)     [10..13] peak (uint32, max of last 5)
 *   [14..15] stock target level     [16] current brightness setting (settings+1)
 *   [17] gear (0..5 row of the stock curve)   [18..19] scale_q10
 *   [20..23] firmware ms tick when the report was built
 *
 * Only the passive tick runs on the hub task (the same context as the stock
 * machine, so the I2C read, ring update and timer re-arm are exactly as safe as
 * stock). Control and the hook install/uninstall run on the EvenHub task; the
 * table entry is a single aligned 32-bit store so the hub task always sees either
 * the stock or the CFW handler. FW_SEND copies its payload into a queued packet,
 * so the report is built on the stack.
 *
 * The state machine pointer only exists in RAM (the dispatch table is initialised
 * from IAR-compressed .data, so 0x4bfb61 appears nowhere in flash); that is why
 * this is a runtime hook instead of a flash patch site. The entry is located by
 * id (8) at install time and restored only if it still holds our handler.
 */

typedef int      (*als_read_fn)(uint32_t *out);        /* FUN_004bf482: 0 = ok */
typedef void     (*als_ring_push_fn)(uint32_t v);      /* FUN_004bedec: 5-sample ring */
typedef uint32_t (*als_ring_peak_fn)(void);            /* FUN_004bef4a: max of ring */
typedef void     (*als_target_fn)(uint32_t peak);      /* FUN_004bf176: gear/target globals */
typedef void     (*als_timer_start_fn)(uint32_t ms);   /* FUN_004b7b28: hub ALS osTimer */
typedef int      (*als_hub_func_fn)(uint32_t func_id); /* FUN_004b80ee open / FUN_004b8160 close */
typedef int      (*als_send_fn)(int type, int sid, unsigned char *buf, unsigned len);
typedef void     (*als_hub_handler_fn)(void *msg);

#define ALS_FW_READ        ((als_read_fn)0x004bf483U)
#define ALS_FW_RING_PUSH   ((als_ring_push_fn)0x004bededU)
#define ALS_FW_RING_PEAK   ((als_ring_peak_fn)0x004bef4bU)
#define ALS_FW_TARGET      ((als_target_fn)0x004bf177U)
#define ALS_FW_TIMER_START ((als_timer_start_fn)0x004b7b29U)
#define ALS_FW_FUNC_OPEN   ((als_hub_func_fn)0x004b80efU)
#define ALS_FW_FUNC_CLOSE  ((als_hub_func_fn)0x004b8161U)
#define ALS_FW_SEND        ((als_send_fn)0x0047d809U)   /* FUN_0047d808 aa21 send */
#define ALS_FW_SIDE        ((lens_side_fn)0x0045cfddU)  /* 1 = master lens */

#define ALS_HUB_FUNC_ID    4u          /* sensor-hub FuncOpen/FuncClose type for the ALS */
#define ALS_HUB_MSG_ID     8u          /* hub message posted by the ALS timer */
#define ALS_HUB_TABLE      0x20003d08U /* 8 x {uint16 id, pad, fn} — hub struct + 0x24 */
#define ALS_HUB_TABLE_N    8u

#define ALS_OPENED     (*(volatile uint32_t *)0x200763d4U)
#define ALS_STATUS     (*(volatile uint32_t *)0x200763d8U)
#define ALS_VALUE      (*(volatile uint32_t *)0x200763f0U)
#define ALS_PEAK       (*(volatile uint32_t *)0x200763f4U)
#define ALS_GEAR       (*(volatile uint32_t *)0x200763f8U)
#define ALS_TARGET     (*(volatile uint32_t *)0x20076400U)
#define ALS_SCALE_Q10  (*(volatile uint32_t *)0x20000068U)
#define ALS_SETTINGS   ((volatile uint8_t *)0x20074c80U) /* +1 level, +2 auto-brightness */

#define ALS_STATUS_POLLING 3u
#define ALS_INTERVAL_MIN   100u
#define ALS_INTERVAL_MAX   5000u
#define ALS_INTERVAL_DEFAULT 500u

#define ALS_PROTO_VERSION  1u
#define ALS_REPORT_FIELD   105u
#define ALS_REPORT_LEN     24u
#define ALS_REASON_QUERY   0u
#define ALS_REASON_START   1u
#define ALS_REASON_POLL    2u
#define ALS_REASON_STOP    3u
#define ALS_START_FLAG_LEASE 0x01u   /* bind passive mode to the framebuffer lease */

void als_hub_handler(void *msg);

static void als_wr32(unsigned char *p, uint32_t v) {
    p[0] = (unsigned char)v; p[1] = (unsigned char)(v >> 8);
    p[2] = (unsigned char)(v >> 16); p[3] = (unsigned char)(v >> 24);
}

/* Locate the hub dispatch entry for the ALS timer message. Returns the address of
 * its fn word, or 0 if the table does not look the way 2.2.9.22 lays it out. */
static volatile uint32_t *als_hub_entry(void) {
    for (uint32_t i = 0; i < ALS_HUB_TABLE_N; i++) {
        volatile uint8_t *e = (volatile uint8_t *)(ALS_HUB_TABLE + i * 8u);
        if (*(volatile uint16_t *)e != ALS_HUB_MSG_ID) continue;
        volatile uint32_t *fn = (volatile uint32_t *)(e + 4);
        uint32_t v = *fn;
        if ((v & 1u) == 0 || v < 0x00438000U || v >= 0x00800000U) return 0;
        return fn;
    }
    return 0;
}

static void als_send_report(customCfwContext *ctx, uint8_t reason) {
    if (ALS_FW_SIDE() != 1) return;
    unsigned char p[7 + ALS_REPORT_LEN];
    p[0] = 0x08; p[1] = 0x03;                    /* field 1: commandId=3 (EVENT) */
    p[2] = 0x10; p[3] = 0x00;                    /* field 2: magic=0 */
    p[4] = 0xCA; p[5] = 0x06;                    /* field 105, wire type 2: tag 842 */
    p[6] = ALS_REPORT_LEN;
    unsigned char *b = p + 7;
    uint8_t flags = 0;
    if (ALS_OPENED) flags |= 0x01u;
    if (ctx->als_hooked) flags |= 0x02u;
    if (ALS_SETTINGS[2]) flags |= 0x04u;
    if (ctx->als_read_ok) flags |= 0x08u;
    b[0] = 'A'; b[1] = 'L'; b[2] = ALS_PROTO_VERSION; b[3] = reason;
    b[4] = flags;
    b[5] = (unsigned char)ALS_STATUS;
    als_wr32(b + 6, ALS_VALUE);
    als_wr32(b + 10, ALS_PEAK);
    uint32_t target = ALS_TARGET;
    b[14] = (unsigned char)target; b[15] = (unsigned char)(target >> 8);
    b[16] = ALS_SETTINGS[1];
    b[17] = (unsigned char)ALS_GEAR;
    uint32_t scale = ALS_SCALE_Q10;
    b[18] = (unsigned char)scale; b[19] = (unsigned char)(scale >> 8);
    als_wr32(b + 20, FW_MS_TICK);
    ctx->als_last_reported = ALS_VALUE;
    ctx->als_last_report_tick = FW_MS_TICK;
    ALS_FW_SEND(1, 9, p, (unsigned)sizeof(p));
}

/* Restore the stock handler and give the sensor back. Safe from either task:
 * the table write is one aligned word and FuncClose only posts a hub message. */
static void als_passive_stop(customCfwContext *ctx, uint8_t reason) {
    int was_hooked = ctx->als_hooked != 0;
    if (was_hooked) {
        volatile uint32_t *fn = als_hub_entry();
        if (fn && *fn == (uint32_t)&als_hub_handler && ctx->als_orig_handler)
            *fn = ctx->als_orig_handler;
        ctx->als_hooked = 0;
    }
    if (ctx->als_opened_by_cfw) {
        ctx->als_opened_by_cfw = 0;
        /* Stock never polls with auto-brightness off, so leaving the sensor open
         * would just burn power. With auto-brightness on the stock machine resumes
         * from status 3 on the next tick, exactly as if it had been polling. */
        if (ALS_SETTINGS[2] == 0 && ALS_OPENED) ALS_FW_FUNC_CLOSE(ALS_HUB_FUNC_ID);
    }
    if (was_hooked) als_send_report(ctx, reason);
}

/* True once a lease-bound passive session has lost its lease (released by the
 * phone, or lapsed because the phone went away). Read-only on purpose: the
 * lease-expiry side effects belong to cfw_fb_lease_active on the EvenHub task. */
static int als_lease_lapsed(customCfwContext *ctx) {
    if ((ctx->als_flags & ALS_START_FLAG_LEASE) == 0) return 0;
    if (ctx->direct_lease_deadline == 0) return 1;
    return (int32_t)(ctx->direct_lease_deadline - FW_MS_TICK) <= 0;
}

/* Hub-task handler for message 8 while passive mode is active. Replaces the
 * stock state machine tick: read the sensor, keep the stock ring/peak/target
 * globals current (so QUERY and a later stock resume see sane values), re-arm
 * the stock timer at our interval, and never touch the panel brightness.
 * Non-static so -O2 keeps it despite its address only ever being stored. */
void als_hub_handler(void *msg) {
    customCfwContext *ctx = peekCustomCfwContext();
    if (ctx == 0 || !ctx->als_hooked) {
        /* Table still points here after our state went away: behave like stock. */
        if (ctx && ctx->als_orig_handler) ((als_hub_handler_fn)ctx->als_orig_handler)(msg);
        return;
    }
    if (als_lease_lapsed(ctx)) {
        als_passive_stop(ctx, ALS_REASON_STOP);
        return;
    }
    if (ALS_OPENED == 0) return;            /* closed underneath us; timer is stopped */

    uint32_t value = 0;
    int ok = ALS_FW_READ(&value) == 0;
    if (ok) {
        ALS_VALUE = value;
        ALS_FW_RING_PUSH(value);
        uint32_t peak = ALS_FW_RING_PEAK();
        ALS_PEAK = peak;
        ALS_FW_TARGET(peak);
    }
    ctx->als_read_ok = (uint8_t)ok;
    ALS_STATUS = ALS_STATUS_POLLING;
    ALS_FW_TIMER_START(ctx->als_interval_ms);

    if (!ok) return;
    uint32_t last = ctx->als_last_reported;
    uint32_t diff = value > last ? value - last : last - value;
    int due = ctx->als_last_report_tick == 0 || diff >= ctx->als_min_delta;
    if (!due && ctx->als_heartbeat_ms &&
        (uint32_t)(FW_MS_TICK - ctx->als_last_report_tick) >= ctx->als_heartbeat_ms) due = 1;
    if (due) als_send_report(ctx, ALS_REASON_POLL);
}

static int als_passive_start(customCfwContext *ctx, uint8_t flags, uint32_t interval,
                             uint32_t min_delta, uint32_t heartbeat) {
    if (interval < ALS_INTERVAL_MIN) interval = ALS_INTERVAL_MIN;
    if (interval > ALS_INTERVAL_MAX) interval = ALS_INTERVAL_MAX;
    ctx->als_flags = flags;
    ctx->als_interval_ms = (uint16_t)interval;
    ctx->als_min_delta = (uint16_t)min_delta;
    ctx->als_heartbeat_ms = (uint16_t)heartbeat;

    if (!ctx->als_hooked) {
        volatile uint32_t *fn = als_hub_entry();
        if (fn == 0) return -2;             /* unexpected table: leave stock alone */
        uint32_t orig = *fn;
        if (orig != (uint32_t)&als_hub_handler) ctx->als_orig_handler = orig;
        if (ctx->als_orig_handler == 0) return -2;
        *fn = (uint32_t)&als_hub_handler;   /* install BEFORE the sensor can tick */
        ctx->als_hooked = 1;
    }
    if (ALS_OPENED == 0) {
        /* Same path the stock auto-brightness toggle uses; the hub task opens the
         * sensor and its first timer tick (110 ms later) already lands in our
         * handler, so the stock "apply target on start" never runs. */
        int r = ALS_FW_FUNC_OPEN(ALS_HUB_FUNC_ID);
        if (r != 0) {
            als_passive_stop(ctx, ALS_REASON_STOP);
            return r;
        }
        ctx->als_opened_by_cfw = 1;
    }
    als_send_report(ctx, ALS_REASON_START);
    ctx->als_last_report_tick = 0;          /* the first real poll always reports */
    return 0;
}

/* Mode-16 entry, called from image_dispatch on both lenses with src[0] == 16. */
int als_control(const uint8_t *src, uint32_t srclen) {
    if (srclen < 2) return -1;
    customCfwContext *ctx = getCustomCfwContext();
    if (ctx == 0) return -1;
    if (ALS_FW_SIDE() != 1) return 0;       /* slave lens: stock never opens its ALS */
    uint8_t op = src[1];
    if (op == 0) {
        als_send_report(ctx, ALS_REASON_QUERY);
        return 0;
    }
    if (op == 1) {
        uint8_t flags = srclen >= 3 ? src[2] : 0;
        uint32_t interval = srclen >= 5 ? ((uint32_t)src[3] | ((uint32_t)src[4] << 8)) : ALS_INTERVAL_DEFAULT;
        uint32_t min_delta = srclen >= 7 ? ((uint32_t)src[5] | ((uint32_t)src[6] << 8)) : 1u;
        uint32_t heartbeat = srclen >= 9 ? ((uint32_t)src[7] | ((uint32_t)src[8] << 8)) : 5000u;
        return als_passive_start(ctx, flags, interval, min_delta, heartbeat);
    }
    if (op == 2) {
        als_passive_stop(ctx, ALS_REASON_STOP);
        return 0;
    }
    return -1;
}

/* Mode-11 session cleanup: hand the sensor back to stock. */
static void als_cleanup_session(void) {
    customCfwContext *ctx = peekCustomCfwContext();
    if (ctx == 0) return;
    if (ctx->als_hooked || ctx->als_opened_by_cfw) als_passive_stop(ctx, ALS_REASON_STOP);
}
