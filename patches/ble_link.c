#include <stdint.h>

/*
 * ble_link.c -- "already fast" test for the forced fast connection profile.
 *
 * patch_compress.py rewrites the stock fast profile to min = max = 7.5 ms and
 * forces every connection-parameter request in _connectParamReq_impl (2.3.0.24
 * 0x47bdb0) to fast (0xa3). Before it sends anything, the fast branch asks the
 * stock link classifier (0x47b890, `bl` at 0x47c1e4) whether the LIVE link is
 * already fast: stock answers 0xa3 when the connection record's interval
 * (+0x18, 1.25 ms units) is below 25 (31.25 ms) and its latency (+0x1a) equals
 * the fast profile's (0), and a 0xa3 verdict ends the request without sending
 * one. That test was written for the stock 15..30 ms fast profile; with the
 * 7.5 ms profile it means a phone link already at 15 or 30 ms with latency 0
 * (iOS's and macOS's default) never gets asked to go faster. Tetramo's fork
 * observed exactly this on hardware (request queued, link unchanged).
 *
 * This replaces the classifier call. A link counts as already fast only at
 * 11.25 ms or less with latency 0. 11.25 ms rather than exactly 7.5 ms because
 * AOSP clamps peripheral requests to its 11.25 ms default floor: requiring 7.5
 * would re-request on every stock trigger (traffic backpressure, image/OTA
 * transfers) and the phone would keep answering 11.25. Those triggers are
 * already rate-limited by the stock 30-45 s hold-off, so a central that never
 * goes below 11.25 ms (or rejects the request) costs at most one request per
 * hold-off, not a loop.
 */

#define BLE_MODE_FAST              0xa3u
#define BLE_MODE_SLOW              0xa4u
#define BLE_CONN_INTERVAL          0x18u   /* connection record: interval, 1.25 ms units */
#define BLE_CONN_LATENCY           0x1au   /* connection record: peripheral latency */
#define BLE_FAST_ENOUGH_INTERVAL   9u      /* 9 x 1.25 ms = 11.25 ms */

/* Replaces `bl 0x47b890` at 0x47c1e4; r0 is the connection record the stock
 * branch loads from 0x200775f8. Returns 0xa3 ("already fast": no request) or
 * 0xa4 (send the fast profile). Stock dereferences the record unconditionally;
 * a missing record is treated as "nothing to request". */
__attribute__((used, noinline)) uint32_t ble_classify_link(const uint8_t *conn) {
    if (conn == 0) return BLE_MODE_FAST;
    uint32_t interval = (uint32_t)conn[BLE_CONN_INTERVAL] | ((uint32_t)conn[BLE_CONN_INTERVAL + 1u] << 8);
    uint32_t latency = (uint32_t)conn[BLE_CONN_LATENCY] | ((uint32_t)conn[BLE_CONN_LATENCY + 1u] << 8);
    return interval <= BLE_FAST_ENOUGH_INTERVAL && latency == 0u ? BLE_MODE_FAST : BLE_MODE_SLOW;
}
