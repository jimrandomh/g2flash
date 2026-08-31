/* gesture_fwd.c — forward source-qualified gestures while Faceclaw owns the
 * EvenHub framebuffer lease, preserving stock behavior at every other time.
 *
 * The 2.2.9.22 input dispatcher (FUN_00444902) keeps the current raw input
 * record in r6. Byte 0 is the source (0/1 = temple touchpads, 4 = R1 ring) and
 * bytes 2..5 form the gesture subtype:
 *
 *   0x03  plain long press       -> stock UI event 0x08
 *   0x0e  long-press release     -> stock UI event 0x4a
 *   0x11  tap then long press    -> new stock Menu path
 *
 * The subtype-3 and subtype-0xe hooks replace calls to the normal UI-event
 * poster. Their naked shims preserve its r0-r2 ABI and pass r6 as a fourth C
 * argument. The subtype-0x11 hook replaces the first no-argument call in the
 * new Menu branch. If Faceclaw handles it, the shim restores its own stack and
 * branches to the dispatcher's common epilogue at 0x00444fe8, suppressing the
 * complete stock Menu path. Otherwise it calls the overwritten stock function
 * and returns to 0x00444a44 exactly as the original BL did.
 *
 * Wire: FUN_004ebad2(0,0,0,EventType,0,RawSource) emits a g2.evenhub SysEvent.
 * The stock sender maps raw sources 0/1/4 to the protobuf left/right/ring source
 * values. Event types 9 and 10 are the existing Faceclaw long-press pair; 11 is
 * a private extension for the newly distinct tap-then-long gesture. A subtype-
 * 0x0e event can follow either press kind and remains the generic release.
 */

typedef int  (*gesture_fc80_t)(void *ctx, int code, void *data);
typedef int *(*gesture_modelookup_t)(void *g);
typedef int  (*gesture_sysevt_t)(int, int, int, int, int, int);

#define GESTURE_FW_FC80   ((gesture_fc80_t)0x004622edu)
#define GESTURE_FW_MODE   ((gesture_modelookup_t)0x004622d7u)
#define GESTURE_FW_SYSEVT ((gesture_sysevt_t)0x004ebad3u)

/* Foreground UI owner pointer. The dispatcher passes UI_CTX[0] to FW_FC80,
 * while FW_MODE consumes UI_CTX itself. Re-derived from four loader witnesses. */
#define GESTURE_UI_CTX (*(void *volatile *)0x20076768u)

#define GESTURE_APP_EVENHUB   0xe0
#define GESTURE_ET_LONG       9
#define GESTURE_ET_RELEASE    10
#define GESTURE_ET_TAP_LONG   11

int gesture_press(void *ctx, int code, void *data);
int gesture_release(void *ctx, int code, void *data);
int gesture_press_impl(void *ctx, int code, void *data,
                       const unsigned char *event);
int gesture_release_impl(void *ctx, int code, void *data,
                         const unsigned char *event);
int gesture_short_long(void);
int gesture_short_long_impl(const unsigned char *event);

static int faceclaw_gesture_event(const unsigned char *event, int event_type)
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

/* r6 is live at both patched UI-event call sites but is outside the stock
 * r0-r2 ABI. Pass it as the fourth argument to the source-qualified helpers. */
__attribute__((naked)) int gesture_press(
    void *ctx __attribute__((unused)),
    int code __attribute__((unused)),
    void *data __attribute__((unused)))
{
    __asm volatile("mov r3, r6\n\tb gesture_press_impl");
}

__attribute__((naked)) int gesture_release(
    void *ctx __attribute__((unused)),
    int code __attribute__((unused)),
    void *data __attribute__((unused)))
{
    __asm volatile("mov r3, r6\n\tb gesture_release_impl");
}

int gesture_press_impl(void *ctx, int code, void *data,
                       const unsigned char *event)
{
    if (faceclaw_gesture_event(event, GESTURE_ET_LONG)) return 1;
    return GESTURE_FW_FC80(ctx, code, data);
}

int gesture_release_impl(void *ctx, int code, void *data,
                         const unsigned char *event)
{
    if (faceclaw_gesture_event(event, GESTURE_ET_RELEASE)) return 1;
    return GESTURE_FW_FC80(ctx, code, data);
}

int gesture_short_long_impl(const unsigned char *event)
{
    return faceclaw_gesture_event(event, GESTURE_ET_TAP_LONG);
}

/* The patched instruction is still a BL. Keep an aligned eight-byte shim frame
 * across the C helper. On pass-through, reproduce the overwritten call and pop
 * directly back to 0x00444a44 with its r0 result intact. On takeover, discard
 * the shim frame and leave the entire subtype-0x11 branch through its epilogue. */
__attribute__((naked)) int gesture_short_long(void)
{
    __asm volatile(
        "push {r3, lr}\n\t"
        "mov r0, r6\n\t"
        "bl gesture_short_long_impl\n\t"
        "cmp r0, #0\n\t"
        "beq 1f\n\t"
        "add sp, #8\n\t"
        "movw r0, #0x4fe9\n\t"
        "movt r0, #0x0044\n\t" /* 0x00444fe9: dispatcher epilogue | Thumb */
        "bx r0\n"
        "1:\n\t"
        "movw r3, #0x8c8b\n\t"
        "movt r3, #0x0046\n\t" /* 0x00468c8b: overwritten state getter | Thumb */
        "blx r3\n\t"
        "pop {r3, pc}"
        ::: "memory");
}
