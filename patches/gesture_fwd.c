/* Source-qualified gestures while Faceclaw owns the EvenHub framebuffer.
 * 2.3.0.24 dispatcher 0x44480a keeps source at r6[0], subtype at r6[2..5].
 * Its new menu-off checks discard press/release BEFORE the UI poster. Hook
 * each first state getter instead: owned events exit through 0x445182;
 * unowned events replay that getter and retain every stock guard and action.
 * All three sites have dead r1/r2 and a saved, eight-byte-aligned stock frame.
 */
typedef int *(*gesture_modelookup_t)(void *g);
typedef int (*gesture_sysevt_t)(int, int, int, int, int, int);
#define GESTURE_FW_MODE ((gesture_modelookup_t)0x0045decbu)
#define GESTURE_FW_SYSEVT ((gesture_sysevt_t)0x004ef959u)
#define GESTURE_UI_CTX (*(void *volatile *)0x200777fcu)
#define GESTURE_APP_EVENHUB 0xe0
#define GESTURE_ET_HEAD_UP 12
#define GESTURE_ET_RING_PRESS 14 /* 13 was used by an experimental temple edge */

/* Wrap the dispatcher's mode lookup at 0x44484a. r6 holds the full input
 * record, including its u16 source and unaligned u32 subtype. The return
 * value is still the original mode pointer, and stock dispatch continues.
 * Event 14 is ring-specific: the stock SysEvent sender leaves its source 0
 * (unspecified), so the phone derives ring provenance from this event ID.
 */
int *gesture_ring_press_mode_impl(void *ctx, const unsigned char *event)
{
    int *mode = GESTURE_FW_MODE(ctx);
    if (event && event[0] == 4 && event[1] == 0 &&
        event[2] == 0x0d && event[3] == 0 && event[4] == 0 && event[5] == 0 &&
        mode && *mode == GESTURE_APP_EVENHUB && cfw_fb_lease_active()) {
        GESTURE_FW_SYSEVT(0, 0, 0, GESTURE_ET_RING_PRESS, 0, 4);
    }
    return mode;
}

__attribute__((naked)) int *gesture_ring_press_mode(void *ctx __attribute__((unused)))
{
    __asm volatile("mov r1, r6\n\tb gesture_ring_press_mode_impl");
}

/* Keep this symbol out of line: the naked entry shims call it from assembly. */
__attribute__((used, noinline)) static int faceclaw_gesture_event(
    const unsigned char *event, int event_type)
{
    if (cfw_fb_lease_active() && event) {
        int *mode = GESTURE_FW_MODE(GESTURE_UI_CTX);
        if (mode && *mode == GESTURE_APP_EVENHUB) {
            GESTURE_FW_SYSEVT(0, 0, 0, event_type, 0, event[0]);
            return 1;
        }
    }
    return 0;
}

/* BL sites: press 0x444940, release 0x444d08, tap-long 0x44499c.
 * Preserve an aligned shim frame, and restore it on either exit. The stock
 * success epilogue returns zero and unwinds the caller's 56-byte frame.
 * The unchanged path calls the displaced no-argument getter at 0x464826. */
#define GESTURE_ENTRY(name, event) \
__attribute__((naked)) int name(void) { \
    __asm volatile( \
        "push {r3, lr}\n\t" \
        "mov r0, r6\n\t" \
        "movs r1, #" event "\n\t" \
        "bl faceclaw_gesture_event\n\t" \
        "cmp r0, #0\n\t" \
        "beq 1f\n\t" \
        "add sp, #8\n\t" \
        "movw r0, #0x5183\n\t" \
        "movt r0, #0x0044\n\t" /* 0x00445183: success epilogue | Thumb */ \
        "bx r0\n" \
        "1:\n\t" \
        "movw r3, #0x4827\n\t" \
        "movt r3, #0x0046\n\t" /* 0x00464827: stock getter | Thumb */ \
        "blx r3\n\t" \
        "pop {r3, pc}" \
        ::: "memory"); \
}
GESTURE_ENTRY(gesture_press, "9")
GESTURE_ENTRY(gesture_release, "10")
GESTURE_ENTRY(gesture_short_long, "11")
#undef GESTURE_ENTRY

/* Head-up (IMU head-tilt) forwarding while an EvenHub page is on screen.
 *
 * The IMU driver posts sensor sub-event 6 (head up) to the display thread's
 * dispatcher as a type-7 message. That dispatcher first calls the stock gate
 * FUN_0046f402 (`bl` at 0x467cf4) and frees the message when the gate fails,
 * which it does while an EvenHub page is on screen: a phone that blanked the
 * page but kept the session (soft sleep) never hears about the head-up. With
 * no page on screen the stock idle path launches the dashboard instead, which
 * faceclaw_display_start_headup (settings_ext.c) already defers to the phone.
 *
 * headup_gate wraps the gate call: the stock result is passed through
 * unchanged, and a head-up is additionally forwarded as EvenHub sys event 12
 * under the framebuffer lease with EvenHub foreground (faceclaw_gesture_event),
 * when the stock head-up switch is on and the stock idle path would itself
 * have acted on it (FUN_004447e8 == 0: not mid-OTA/onboarding). */
#define HEADUP_SUBEVENT_UP    6

typedef unsigned (*headup_switch_t)(void);
typedef int (*headup_busy_t)(void);
#define HEADUP_FW_SWITCH ((headup_switch_t)0x00467291u) /* FUN_00467290: settings->head_up_switch */
#define HEADUP_FW_BUSY   ((headup_busy_t)0x004447e9u)   /* FUN_004447e8: 1 while the stock idle path ignores head-up */

int headup_gate_impl(int stock_result, const unsigned char *message);

int headup_gate_impl(int stock_result, const unsigned char *message)
{
    static const unsigned char headup_source[1] = { 0 };
    if (message) {
        const unsigned char *payload = *(const unsigned char *const *)(message + 8);
        if (stock_result == 1) {
            faceclaw_idle_input_forward(payload);
            return stock_result;
        }
        if (payload) {
            uint32_t subtype = (uint32_t)payload[4] | ((uint32_t)payload[5] << 8) |
                               ((uint32_t)payload[6] << 16) | ((uint32_t)payload[7] << 24);
            if (subtype == HEADUP_SUBEVENT_UP && HEADUP_FW_SWITCH() != 0 && HEADUP_FW_BUSY() == 0) {
                faceclaw_gesture_event(headup_source, GESTURE_ET_HEAD_UP);
            }
        }
    }
    return stock_result;
}

/* Replaces `bl FUN_0046f402` at 0x467cf4. r4 is dead at the site but is
 * preserved anyway; after our two-word push the caller's message slot
 * ([sp, #0xc] at the BL) sits at [sp, #0x14]. */
__attribute__((naked)) int headup_gate(void)
{
    __asm volatile(
        "push {r4, lr}\n\t"
        "ldr r4, [sp, #0x14]\n\t"
        "movw r0, #0xf403\n\t"
        "movt r0, #0x0046\n\t" /* 0x0046f402: stock idle gate | Thumb */
        "blx r0\n\t"
        "mov r1, r4\n\t"
        "bl headup_gate_impl\n\t"
        "pop {r4, pc}"
        ::: "memory");
}

/* Faceclaw/19: preserve each complete R1 report BEFORE the stock 100-tick
 * suppression at 0x47913e. While the framebuffer lease is held, these reports
 * go to the phone once, through the right lens. Left ingress relays the raw
 * report across the existing ordered frame bridge before phone serialization.
 * No lease (including expiry), legacy reports and other packet families replay
 * the exact original receiver. Nothing changes before Faceclaw takes ownership.
 *
 * EvenHub SysEvent field 100 is a 12-byte private extension:
 *   "RI", version=1, flags=1, wireType, aux, speed, reserved=0, tick LE32.
 * Tick is the original ring clock, NOT a glasses/phone epoch. Unknown wire
 * types use SysEvent 127: visible to diagnostics and useful for reproducing
 * stock suppression, but never interpreted as a click or system exit.
 */
#define FACECLAW_BRIDGE_RING_REPORT 3u

__attribute__((used, noinline)) static int faceclaw_ring_report(
    const unsigned char *report, unsigned length)
{
    if (!report || (uint16_t)length != 11 || report[0] != 0 ||
        report[1] != 9 || report[2] != 0x61 || report[3] != 0 ||
        !cfw_fb_lease_active()) return 0;
    /* The R1 may be connected to the LEFT lens. Stock INPM_EventSend uses
     * SendDataToBoth (service 0x108); bypassing it must retain that peer hop.
     * Only the right lens may use the phone protobuf notifier. The bridge
     * copies these bytes, and its local echo is ignored by the receiver below.
     * If enqueue fails, replay stock input rather than consuming the report. */
    unsigned side = FW_SIDE_ID();
    if (side == 2)
        return cfw_message_bridge_send(FACECLAW_BRIDGE_RING_REPORT,
                                      CFW_MESSAGE_LEFT, report, 11) == 0;
    if (side != 1) return 0;
    customCfwContext *ctx = faceclaw_context_if_valid();
    if (!ctx) return 0;
    unsigned event = 127;
    switch (report[4]) {
        case 0: event = 9; break;
        case 1: event = 0; break;
        case 2: event = 3; break;
        case 4: event = 1; break; /* ring swipe up -> scroll top */
        case 5: event = 2; break; /* ring swipe down -> scroll bottom */
        case 8: event = 10; break;
        case 9: event = 11; break;
        case 10: event = 14; break;
    }
    uint8_t *p = ctx->ring_notify_buf;
    p[0]=8; p[1]=2; p[2]=0x6a; p[3]=21; p[4]=0x1a; p[5]=19;
    p[6]=8; p[7]=event; p[8]=0x10; p[9]=2; /* explicit ring source */
    p[10]=0xa2; p[11]=6; p[12]=12;
    p[13]='R'; p[14]='I'; p[15]=1; p[16]=1;
    p[17]=report[4]; p[18]=report[5]; p[19]=report[6]; p[20]=0;
    for (unsigned i=0; i<4; ++i) p[21+i]=report[7+i];
    /* Match the stock SysEvent sender at 0x4efb92. FW_SEND creates a
     * reply (flag 0), which the phone correctly rejects as an input event. */
    ((send_fn)FW_NOTIFY_SEND)(1, 0xe0, p, 25);
    return 1;
}

/* B.W at receiver entry, before push {r3,r4,r5,lr}; sub sp,#16.
 * Preserve all argument registers for the stock path and callee-saved
 * registers on both paths. Six words keep the helper's stack aligned. */
__attribute__((naked)) int faceclaw_ring_receive(void)
{
    __asm volatile(
        "push {r0-r4, lr}\n\t"
        "bl faceclaw_ring_report\n\t"
        "cmp r0, #0\n\t"
        "beq 1f\n\t"
        "pop {r0-r4, lr}\n\t"
        "movs r0, #0\n\t"
        "bx lr\n"
        "1:\n\t"
        "pop {r0-r4, lr}\n\t"
        "push {r3,r4,r5,lr}\n\t"
        "sub sp, #16\n\t"
        "movw r3, #0x911f\n\t"
        "movt r3, #0x0047\n\t" /* 0x0047911f: original body | Thumb */
        "bx r3"
        ::: "memory");
}

/* Wrap the existing three ordered bridge-arrival hooks. Ring reports use a
 * new kind in the private SID-f0 envelope: [3, origin=LEFT, raw R1 bytes(11)].
 * Request/ACK traffic continues through the existing transport unchanged.
 * SendDataToBoth echoes locally: the left lens must neither send to the phone
 * nor enqueue again. The peer checks its own lease before notifying the phone,
 * so queued reports cannot outlive Faceclaw's ownership. */
uint32_t faceclaw_input_bridge_received(uint32_t app_id, const uint8_t *data,
                                        uint32_t length, uint16_t event)
{
    if (app_id != CFW_MESSAGE_SID || !data || !length ||
        data[0] != FACECLAW_BRIDGE_RING_REPORT)
        return cfw_message_bridge_received(app_id, data, length, event);
    if (length != 13) return 0xbu;
    if (data[1] != CFW_MESSAGE_LEFT) return 0xau;
    if (FW_SIDE_ID() == 1) faceclaw_ring_report(data + 2, 11);
    return 0;
}
