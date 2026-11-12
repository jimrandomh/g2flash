/* Phone-originated input injection, which stock firmware has no route for.
 *
 *     sid 0xf0, mode 32: [32][dev u8][event_id u32 LE][diffx u8][speed u8]
 *
 * Rebuilt into the stock 12-byte input record (+0 u16 dev_type, +4 u32
 * event_id, +8 u8 diffx, +9 u8 speed) and handed to SendDataToBoth as message
 * 0x108 — byte-identical to what the ring-touch driver sends, so the stock UI
 * cannot tell the difference.
 *
 * Addressed to the left lens alone. SendDataToBoth dispatches on both lenses,
 * so a record arriving on both makes the master dispatch twice and the UI
 * advance twice; the side guard drops the copy that lands on the right.
 *
 * Not gated on the framebuffer lease: that lease belongs to code driving
 * Faceclaw's framebuffer, and this drives the stock UI.
 */
#include <stdint.h>
#include "cfw_context.h"

#define INPM_EVENT_MSG_ID      0x108u
#define INJECT_MIN_INTERVAL_MS 20u

/* The fourth argument is a completion callback: stock stores it, carries it
 * into the TinyFrame record and calls it as void(*)(uint32_t) on ACK and with
 * -2 on timeout. It must be 0, because any non-zero value there is branched
 * to. */
typedef int32_t (*send_both_fn)(uint16_t type, const void *payload,
                                uint32_t length, void *done_cb);
#define FW_SEND_DATA_TO_BOTH ((send_both_fn)0x0046a861u)

/* Every return is 0, including every refusal.
 *
 * A non-zero return reaches cfw_message_complete, which resets the inflate
 * stream and clears context_valid. That flag is sticky and per ingress lens,
 * so one NACK kills the whole sid-0xf0 stream for that lens — injection and
 * image traffic alike — until a record carrying CFW_MESSAGE_RESET_CONTEXT
 * arrives. Refusal here is routine rather than exceptional: the rate limiter
 * drops over-rate injections, and a phone holding a direction down hits it
 * constantly. A routine drop must not be able to take the transport down.
 *
 * The cost is that for mode 32 an ACK means received, not injected. The phone
 * paces itself above the floor and judges an injection by the display.
 */
int cfw_inject_input(const uint8_t *src, uint32_t srclen)
{
    if (src == 0 || srclen < 8u) return 0;

    /* Left lens only; see the header. */
    if (FW_SIDE_ID() != 2u) return 0;

    uint32_t dev   = src[1];
    uint32_t event = (uint32_t)src[2]
                   | ((uint32_t)src[3] << 8)
                   | ((uint32_t)src[4] << 16)
                   | ((uint32_t)src[5] << 24);

    /* dev and event pass through unfiltered, because every consumer downstream
     * of SendDataToBoth already guards itself: the name lookups end in a
     * default case, the touch tracepoint returns on a negative device index,
     * and the both-long-press handler acts only on the two temple bars.
     *
     * The one combination that bites: PRESS on dev 0 and dev 1 within 2000 ms
     * arms the both-long-press check, which then rejects all input — the
     * physical bars included — and toggles silent mode, which persists.
     * Injecting RELEASE on both devs clears it. */

    customCfwContext *ctx = faceclaw_context_if_valid();
    if (ctx == 0) return 0;

    /* The master's handler tails into SendInputEventToPeers, whose queue-full
     * path masks interrupts, faults deliberately and spins; stock never
     * reaches it because its events come from a finger.
     *
     * The floor is soft. This read-compare-write is not atomic and nothing
     * serialises the two ingress paths — BLE receive and inter-arm bridge
     * receive are separate tasks — so a phone writing to both arms at once can
     * have two records pass on the same old tick. The excess is bounded at 2x,
     * an effective 10 ms, which is still far slower than the queue this
     * guards. An atomic compare-exchange on inject_last_tick makes it hard if
     * that ever stops being enough. */
    uint32_t now = FW_MS_TICK;
    if (ctx->inject_last_tick != 0u &&
        (uint32_t)(now - ctx->inject_last_tick) < INJECT_MIN_INTERVAL_MS)
        return 0;
    ctx->inject_last_tick = (now == 0u) ? 1u : now;

    uint32_t ev[3];
    ev[0] = dev;                                   /* +0 u16 dev_type, +2 pad */
    ev[1] = event;                                 /* +4 u32 event_id */
    ev[2] = (uint32_t)src[6] | ((uint32_t)src[7] << 8);  /* +8 diffx, +9 speed */

    /* The 12 is a safety invariant: the receiving handler memcpys `length`
     * bytes off the wire into 12 bytes of stack without bounding it against
     * the record size.
     *
     * The return is deliberately discarded. SendDataToBoth returns -1 when a
     * queue pointer is null, when CommonMalloc gives up, or when the queue put
     * times out, and propagating that would NACK — see above. A dropped event
     * is recoverable by sending another; a dead transport is not. The phone
     * cannot tell a failed send from a successful one, so the display is the
     * check. */
    (void)FW_SEND_DATA_TO_BOTH((uint16_t)INPM_EVENT_MSG_ID, ev, 12u, 0);
    return 0;
}
