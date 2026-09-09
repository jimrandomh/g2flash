#include <stdint.h>
#include "protobuf.h"

/* 2.2.9.22: SVC_RingBattery_Update (0x512cfc) receives the R1 report;
 * ux battery-sync calls the cache setter at 0x512d84. The dashboard's getter
 * 0x4a9be2 calls 0x512da2, reading 0x200772a6 (level) and +1 (charging).
 * 0x47efa8 is the dashboard's role-aware ring-connected predicate.
 * See docs/ring-battery.md for the disassembly evidence and wire contract. */
#ifndef RING_BATTERY_CACHE
#define RING_BATTERY_CACHE (*(volatile uint16_t *)0x200772a6u)
#define RING_BATTERY_CONNECTED() (((unsigned (*)(void))0x0047efa9u)())
#define RING_BATTERY_SIDE() (((unsigned (*)(void))0x0045cfddu)())
#define RING_BATTERY_SEND(buf, len) \
    (((int (*)(int, int, unsigned char *, unsigned))0x0047d90fu)(1, 9, buf, len))
#endif

/* Field 106: ['R','B',1,flags,level]; flags: connected=1, valid=2,
 * charging=4. Unknown/disconnected level is 255. Read only the stock cache;
 * do not open a second ring connection or disturb its battery-report cadence. */
unsigned ring_battery_append_status(unsigned char *buf, unsigned len, unsigned capacity) {
    unsigned char body[5] = {'R', 'B', 1, 0, 255};
    if (RING_BATTERY_CONNECTED()) {
        uint16_t cache = RING_BATTERY_CACHE;
        unsigned level = cache & 255u;
        body[3] = 1;
        if (level <= 100u) {
            body[3] |= 2u;
            if (cache >> 8) body[3] |= 4u;
            body[4] = (unsigned char)level;
        }
    }
    return pb_append_bytes_field(buf, len, capacity, 106u, body, sizeof(body));
}

/* Image-handler mode 17, op 0: one settings-channel notification. */
int ring_battery_control(const uint8_t *src, uint32_t len) {
    if (!src || len != 2 || src[0] != 17 || src[1] != 0) return -1;
    if (RING_BATTERY_SIDE() != 1) return 0;
    unsigned char buf[12] = {0x08, 0x03, 0x10, 0x00};
    unsigned size = ring_battery_append_status(buf, 4, sizeof(buf));
    return RING_BATTERY_SEND(buf, size);
}
