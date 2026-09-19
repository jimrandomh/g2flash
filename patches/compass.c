#include <stdint.h>
#include "cfw_context.h"
#include "protobuf.h"

/* 2.3.0.24: capture the GAF output before DRV_IMUDataParserCallback clears it.
 * The stock parser can select GRV, GMRV, or RV into the same quaternion slot.
 * Keep a timestamp-keyed sidecar for each of its 20 records; sampling and report
 * selection both run on the sensor-hub task. Never infer the source later from
 * a global that may already describe a newer sample.
 *
 * Replace the heading-report call at 0x4b90ce, preserving its stock UI event,
 * and send one sid-8 command-15 notification directly. This replaces the old
 * CFW global-display forwarding hook (it must not also emit a second heading).
 * Stock Navigation remains untouched. The sender copies its stack payload.
 * 2.3.0 filters centidegrees before this call, which still receives whole
 * degrees. Diagnostics describe the latest captured input to that filter;
 * the heading itself can incorporate older samples through stock filtering.
 *
 * Root protobuf field 100 is an optional 12-byte diagnostic extension:
 *   'C','M',1, accuracy, anomalies, source, flags,0, sample_ms_LE32.
 * accuracy/anomalies: 0..3 / 0..2; 255 = unavailable.
 * source: 0 none/unknown, 1 GRV, 2 GMRV, 3 RV.
 * flags: bit7 matching capture, bit0 GRV valid, bit1 GMRV + heading valid,
 * bit2 RV + heading valid, bit3 fresh magnetic bias/flags in this GAF frame.
 * Magnetic flags otherwise carry the driver's last known values.
 */
#ifndef COMPASS_HOST_TEST
#define COMPASS_DECODE ((int (*)(void *, const void *, const void *, uint8_t *))0x00521531U)
#define COMPASS_STOCK_EVENT ((int (*)(uint32_t, int32_t))0x004b8865U)
/* Thread_MsgPbNotifyByBle, as called by stock navigation at 0x59f58c.
 * MsgPbTxByBle (0x47ef04, used by ALS replies) gives the wrong frame flag. */
#define COMPASS_SEND ((int (*)(int, int, const uint8_t *, unsigned))0x0047f025U)
#define COMPASS_SIDE ((int (*)(void))0x00465d4dU)
#define COMPASS_RING ((volatile uint8_t *)0x20066650U)
#define COMPASS_MAG_SEEN (*(volatile uint8_t *)0x20078473U)
#define COMPASS_ACCURACY (*(volatile uint8_t *)0x20078475U)
#define COMPASS_ANOMALIES (*(volatile uint8_t *)0x20078474U)
#endif

/* DRV_IMUSetSensorParameters, 2.3.0.24, at 0x4b75be:
 *   movs r1,#0; strb r1,[r0]
 * r0 points at the driver's magnetic accuracy byte (0x20078475). The next
 * instructions reload it into r2 and call the vendor bias-restoration API
 * with the driver's cached bias. That API restores covariance as well as bias.
 *
 * Replace only this reset with a BL. Return the pointer in r0; r1/r2/r3 are
 * overwritten before use, flags are dead, and LR is saved by the stock prologue.
 * Preserve the current accuracy, including zero/downgrades, rather than a
 * historical maximum. The stock FIFO parser continues updating bias/accuracy.
 *
 * Read the framebuffer (session) lease without cfw_fb_lease_active(): its
 * expiration path frees display resources, which this sensor-task hook must
 * not do. No allocation, sensor I/O, or state restoration occurs here. A missing
 * context, released/expired lease, or invalid accuracy uses the stock reset.
 * Do not gate on compass_forward: stopping reports may itself reconfigure IMU.
 */
volatile uint8_t *compass_preserve_accuracy(volatile uint8_t *accuracy) {
    customCfwContext *ctx = peekCustomCfwContext();
    uint32_t deadline = ctx ? __atomic_load_n(&ctx->direct_lease_deadline, __ATOMIC_RELAXED) : 0;
    if (!deadline || (int32_t)(deadline - FW_MS_TICK) <= 0 || *accuracy > 3)
        *accuracy = 0;
    return accuracy;
}

int compass_decode_capture(void *device, const void *sensor0, const void *sensor1, uint8_t *gaf) {
    int result = COMPASS_DECODE(device, sensor0, sensor1, gaf);
    customCfwContext *ctx = peekCustomCfwContext();
    if (result == -1 || !gaf[0x4d] || !ctx || !ctx->compass_forward) return result;
    uint32_t index = *(volatile uint32_t *)(COMPASS_RING + 8);
    if (index >= 20) return result;
    cfw_compass_sample *sample = &ctx->compass_samples[index];
    sample->timestamp = *(volatile uint32_t *)(COMPASS_RING + 12 + index * 0x70);
    sample->source = 0;
    sample->flags = 0x80;
    if (gaf[8]) { sample->source = 1; sample->flags |= 1; }
    if (gaf[18] && gaf[22]) { sample->source = 2; sample->flags |= 2; }
    if (gaf[32] && gaf[36]) { sample->source = 3; sample->flags |= 4; }
    if (gaf[0x46]) {
        sample->accuracy = gaf[0x44];
        sample->anomalies = gaf[0x45];
        sample->flags |= 8;
    } else {
        sample->accuracy = COMPASS_MAG_SEEN ? COMPASS_ACCURACY : 255;
        sample->anomalies = COMPASS_MAG_SEEN ? COMPASS_ANOMALIES : 255;
    }
    return result;
}

int compass_report_event(uint32_t event, int32_t heading) {
    int result = COMPASS_STOCK_EVENT(event, heading);
    customCfwContext *ctx = peekCustomCfwContext();
    if (event != 9 || heading < 0 || heading >= 360 || !ctx ||
        !ctx->compass_forward || COMPASS_SIDE() != 1) return result;

    uint8_t diagnostic[12] = {'C', 'M', 1, 255, 255, 0, 0, 0, 0, 0, 0, 0};
    uint32_t count = *(volatile uint32_t *)(COMPASS_RING + 8);
    /* Match the stock emitter's backward scan exactly, including count == 0.
     * A missing or stale capture remains explicitly unknown. */
    if (count <= 20) {
        while (count) {
            uint32_t index = --count;
            volatile uint8_t *record = COMPASS_RING + 12 + index * 0x70;
            if (!(record[4] & 0x20)) continue;
            const cfw_compass_sample *sample = &ctx->compass_samples[index];
            uint32_t timestamp = *(volatile uint32_t *)record;
            if ((sample->flags & 0x80) && sample->timestamp == timestamp) {
                diagnostic[3] = sample->accuracy;
                diagnostic[4] = sample->anomalies;
                diagnostic[5] = sample->source;
                diagnostic[6] = sample->flags;
                for (unsigned i = 0; i < 4; i++) diagnostic[8 + i] = (uint8_t)(timestamp >> (8 * i));
            }
            break;
        }
    }
    uint8_t packet[24];
    packet[0] = 0x08; packet[1] = 15; /* command: compass changed */
    packet[2] = 0x10; packet[3] = 0;  /* magic=0, as in the stock notifier */
    packet[4] = 0x52;               /* field 10: Compass { field 1: heading } */
    packet[6] = 0x08;
    unsigned n = pb_write_varint(packet + 7, (unsigned)heading);
    packet[5] = (uint8_t)(n + 1);
    n = pb_append_bytes_field(packet, 7 + n, sizeof(packet), 100, diagnostic, sizeof(diagnostic));
    COMPASS_SEND(1, 8, packet, n);
    return result;
}
