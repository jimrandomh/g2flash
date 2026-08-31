/* gesture_fwd.c — while a Faceclaw framebuffer lease owns the EvenHub session,
 * forward R1 long-press + release-long-press to the phone as SysEvents. Without
 * that lease, preserve the stock UI-event handling.
 *
 * In 2.2.9.22 the Menu gesture changed to tap-then-long-press. Plain subtype 3
 * remains LONG_PRESS but now posts UI event 8 rather than directly opening the
 * force-quit dialog; subtype 0xe still posts release event 0x4a. Both call sites
 * have r6 = the current input record, whose byte 0 is the source (4 = R1 ring).
 * The small naked shims below preserve the stock r0-r2 UI-event ABI and pass r6
 * as a fourth C argument, avoiding a version-specific absolute event-record global.
 *
 * Wire: FUN_004ebad2(0,0,0, EventType, 0, 0) emits a g2.evenhub SysEvent
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

#define FW_FC80   ((fc80_t)0x004622edu)        /* FUN_004622ec  post UI event (Thumb) */
#define FW_MODE   ((modelookup_t)0x004622d7u)  /* FUN_004622d6  foreground mode ctx (Thumb) */
#define FW_SYSEVT ((sysevt_t)0x004ebad3u)      /* FUN_004ebad2  send EvenHub SysEvent (Thumb) */

/* Foreground UI owner pointer. The dispatcher passes UI_CTX[0] to FW_FC80, while
 * FW_MODE consumes UI_CTX itself. Re-derived from four unanimous loader witnesses. */
#define UI_CTX   (*(void *volatile *)0x20076768u)

#define APP_EVENHUB 0xe0
#define ET_LONG     9    /* OsEventTypeList: RING_LONG_PRESS_EVENT */
#define ET_REL      10   /* OsEventTypeList: RING_LONG_PRESS_RELEASE_EVENT */
#define SRC_RING    4    /* input event source byte: R1 ring */

int ring_press(void *ctx, int code, void *data);
int ring_release(void *ctx, int code, void *data);
int ring_press_impl(void *ctx, int code, void *data, const unsigned char *event);
int ring_release_impl(void *ctx, int code, void *data, const unsigned char *event);

static int faceclaw_ring_event(const unsigned char *event, int event_type)
{
    if (cfw_fb_lease_active() && event && event[0] == SRC_RING) {
        int *mode = FW_MODE(UI_CTX);
        if (mode && *mode == APP_EVENHUB) {
            FW_SYSEVT(0, 0, 0, event_type, 0, 0);
            return 1;
        }
    }
    return 0;
}

/* r6 is live at both patched display-dispatch call sites but is not part of the
 * stock r0-r2 post-UI-event ABI. Pass it as the fourth argument to the C helpers. */
__attribute__((naked)) int ring_press(
    void *ctx __attribute__((unused)),
    int code __attribute__((unused)),
    void *data __attribute__((unused)))
{
    __asm volatile("mov r3, r6\n\tb ring_press_impl");
}

__attribute__((naked)) int ring_release(
    void *ctx __attribute__((unused)),
    int code __attribute__((unused)),
    void *data __attribute__((unused)))
{
    __asm volatile("mov r3, r6\n\tb ring_release_impl");
}

int ring_press_impl(void *ctx, int code, void *data, const unsigned char *event)
{
    if (faceclaw_ring_event(event, ET_LONG)) return 1;
    return FW_FC80(ctx, code, data);
}

int ring_release_impl(void *ctx, int code, void *data, const unsigned char *event)
{
    if (faceclaw_ring_event(event, ET_REL)) return 1;
    return FW_FC80(ctx, code, data);
}
