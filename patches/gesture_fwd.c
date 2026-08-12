/* gesture_fwd.c — while a Faceclaw framebuffer lease owns the EvenHub session,
 * forward long-press + ring release-long-press to the phone as SysEvents instead
 * of opening the built-in "End this feature?" force-quit dialog. Without that
 * lease, preserve the stock dialog and release handling.
 *
 * Background (all confirmed on 2.2.4.34, see the g2-evenhub-input-event-map note):
 *   - Input dispatcher FUN_004424a2 turns each gesture into a UI event code and
 *     posts it via FUN_0045fc80(ctx, code, data).
 *   - Long-press = subtype 3. In EvenHub the branch calls FUN_0046a644 (dialog).
 *   - Ring release-long-press = subtype 0xe -> FUN_0045fc80(ctx, 0x4a, coords),
 *     which the EvenHub UI handler drops (no 0x4a case). Subtype 0xe is SHARED
 *     with a touchpad gesture, so we gate release forwarding on source==ring.
 *   - The UI-event dispatch (FUN_0045fc80 -> FUN_004505a4 -> FUN_004509a0 ->
 *     FUN_0045062c -> (*handler)()) is SYNCHRONOUS on the display thread, and the
 *     stock EvenHub SysEvent sender FUN_004ff232 is called from that same handler
 *     on that same thread. So we can build+send the SysEvent DIRECTLY here, from
 *     FUN_004424a2, with no cross-thread race and no change to the UI handler.
 *
 * Wire: FUN_004ff232(0,0,0, EventType, 0, 0) emits a g2.evenhub SysEvent
 * (Cmd=OS_NOITY_EVENT_TO_APP_PACKET, DevEvent.SysEvent) with EventType = the 4th
 * arg. We use the OsEventTypeList values Faceclaw already reserves:
 * 9 = RING_LONG_PRESS_EVENT, 10 = RING_LONG_PRESS_RELEASE_EVENT (8 is IMU report).
 * Both are gated to source==ring so press/release are symmetric (a touchpad
 * long-press won't emit an unpaired press). EventSource stays 0 for these custom
 * types, which is fine since we've already restricted them to the ring.
 */

typedef int  (*fc80_t)(void *ctx, int code, void *data);
typedef int *(*modelookup_t)(void *g);
typedef int  (*sysevt_t)(int, int, int, int, int, int);
typedef void (*longpress_fn)(unsigned command, unsigned app_id);

#define FW_FC80   ((fc80_t)0x0045f8fdu)        /* FUN_0045f8fc  post UI event (Thumb) */
#define FW_MODE   ((modelookup_t)0x0045f8e7u)  /* FUN_0045f8e6  foreground mode ctx (Thumb) */
#define FW_SYSEVT ((sysevt_t)0x004da16bu)      /* FUN_004da16a  send EvenHub SysEvent (Thumb) */
#define FW_LONGPRESS ((longpress_fn)0x0046ae9du) /* FUN_0046ae9c stock force-quit dialog */

/* Foreground UI ctx pointer: FUN_00442d86 loads r5 = *(0x00443750) = 0x200744d0,
 * then passes r5[0] as the ctx to FUN_0045f8fc / FUN_0045f8e6. */
#define UI_CTX   (*(void *volatile *)0x200744d0u)
/* Current input event record (sourced from the literal at 0x004444a4); byte 0 is the source:
 * 0/1 = left/right temple touchpad, 4 = R1 ring. */
#define EVT_SRC  (*(volatile unsigned char *)0x2034dc30u)

#define APP_EVENHUB 0xe0
#define ET_LONG     9    /* OsEventTypeList: RING_LONG_PRESS_EVENT */
#define ET_REL      10   /* OsEventTypeList: RING_LONG_PRESS_RELEASE_EVENT */
#define SRC_RING    4    /* input event source byte: R1 ring */

/* Replaces the subtype-3 EvenHub force-quit dialog call. Preserve that exact
 * stock behavior unless Faceclaw owns the active EvenHub display session. Under
 * Faceclaw, gate on source==ring so press pairs with the ring-only release below;
 * a touchpad long-press keeps Faceclaw's prior no-op behavior. */
void evenhub_longpress(unsigned command, unsigned app_id)
{
    if (!cfw_fb_lease_active()) {
        /* The patched stock call site supplies r0=1 and r1=APP_EVENHUB. The
         * dialog function serializes both values into its display command, so
         * preserve them across the lease check and forward them verbatim. */
        FW_LONGPRESS(command, app_id);
    } else if (EVT_SRC == SRC_RING) {
        FW_SYSEVT(0, 0, 0, ET_LONG, 0, 0);
    }
}

/* Wraps the subtype-0xe post (bl FUN_0045fc80(ctx, 0x4a, coords)). For a RING
 * release-long-press while an EvenHub app is foreground, emit the release
 * SysEvent and skip the (dropped-anyway) 0x4a post. Every other case — touchpad
 * subtype-0xe, or ring release outside EvenHub — falls through to the original
 * post so terminal/native/touchpad behavior is byte-for-byte unchanged. The
 * return value is ignored by the caller. */
int ring_release(void *ctx, int code, void *data)
{
    if (cfw_fb_lease_active() && EVT_SRC == SRC_RING) { /* Faceclaw-owned ring */
        int *mode = FW_MODE(UI_CTX);
        if (mode && *mode == APP_EVENHUB) {      /* EvenHub foreground */
            FW_SYSEVT(0, 0, 0, ET_REL, 0, 0);
            return 1;
        }
    }
    return FW_FC80(ctx, code, data);
}
