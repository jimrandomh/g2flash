/* Stock-dashboard phone-link diagnostics, G2 2.2.9.22 only.
 * See connection_status.md for the disassembly/ABI evidence and limitations.
 * No heap, timers, GATT requests, BLE callbacks, or CFW app handshake. All UI
 * work runs on the existing display task. The shared UX readiness getter stays
 * unchanged. The host's GAP Device Name is not cached by the verified stock
 * discovery path, so use the current connection's peer address, never ours.
 */
#include <stdint.h>

#define CS_MAGIC 0x43534c31u
#define CS_POLL_MS 500u
#define CS_TIMEOUT (-2)
#define CS_STATE_ADDR 0x2029f4b0u /* +8 in the already reserved 1 KiB TLSF tail */

typedef struct {
    uint8_t linked, address_valid, address[6];
} cs_snapshot;
typedef struct {
    uint32_t magic;
    cs_snapshot rendered;
} cs_state;
_Static_assert(sizeof(cs_state) == 12, "reserved connection status ABI");

#ifndef CS_HOST_TEST
#define CS_STATE ((volatile cs_state *)(uintptr_t)CS_STATE_ADDR)
#define CS_READ8(a) (*(volatile uint8_t *)(uintptr_t)(a))
#define CS_READ32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define CS_QUEUE_GET(q,m,p,t) \
    (((int (*)(uint32_t,void *,uint8_t *,uint32_t))0x00443317u)(q,m,p,t))
/* xQueueSemaphoreTake returns 1 on success; this is NOT osSemaphoreAcquire. */
#define CS_TRY_LOCK(h) (((int (*)(uint32_t,uint32_t))0x00442389u)(h,0))
#define CS_CUSTOM_FRAME_ACTIVE() cs_custom_frame_active()
#define CS_UNLOCK() (((void (*)(void))0x004794cfu)())
#define CS_FLUSH(p,n) do { uint32_t d[2]={(uint32_t)(uintptr_t)(p),(n)}; \
    ((void (*)(void *))0x0047ce03u)(d); } while (0)
static int cs_custom_frame_active(void) {
    customCfwContext *ctx=peekCustomCfwContext();
    return ctx && (ctx->direct_pending || (ctx->direct_lease_deadline &&
        (int32_t)(ctx->direct_lease_deadline-FW_MS_TICK)>0));
}
static uint32_t cs_enter(void) {
    uint32_t key;
    __asm volatile("mrs %0, primask\n cpsid i" : "=r"(key) :: "memory");
    return key;
}
static void cs_leave(uint32_t key) {
    __asm volatile("msr primask, %0" :: "r"(key) : "memory");
}
#endif

/* Snapshot fixed stock control blocks under a short IRQ guard. No function
 * calls/allocations while masked. A live ID is enough to indicate occupancy;
 * an address is used only if Cordio confirms an in-use peripheral record.
 * Invalid/uninitialized IDs cannot index outside the three-record table.
 */
static cs_snapshot cs_read_link(void) {
    cs_snapshot s = {0,0,{0,0,0,0,0,0}};
    uint32_t key = cs_enter();
    uint32_t ctx = CS_READ32(0x20076554u);
    if ((ctx & 3u) == 0 && ctx >= 0x20000000u && ctx <= 0x207fffabu) {
        uint8_t id = CS_READ8(ctx + 0x54u);
        if (id >= 1 && id <= 3) {
            uint32_t rec = 0x20072d68u + (uint32_t)(id - 1u) * 0x30u;
            s.linked = 1;
            if (CS_READ8(rec + 0x16u) && CS_READ8(rec + 0x19u) != 1) {
                s.linked=0; /* a central/ring connection is not a phone */
            } else if (CS_READ8(rec + 0x16u)) {
                uint8_t any = 0, all = 0xff;
                for (unsigned i=0;i<6;i++) {
                    s.address[i] = CS_READ8(rec+i);
                    any |= s.address[i]; all &= s.address[i];
                }
                s.address_valid = any != 0 && all != 0xff;
            }
        }
    }
    cs_leave(key);
    return s;
}

/* Same state selection as getRunningAppID (0x445074), without diagnostics.
 * Startup type 1: background + optional foreground. Type 2: single foreground.
 * App ID 1 is dashboard. Never paint over another foreground app or sleep.
 */
static int cs_dashboard_visible(void) {
    if (CS_READ8(0x20077298u) != 1 || CS_CUSTOM_FRAME_ACTIVE()) return 0;
    uint32_t type = CS_READ32(0x20076774u);
    uint32_t base = CS_READ32(0x20076770u);
    uint32_t front = CS_READ32(0x20076778u);
    return (type == 2 && base == 1) ||
           (type == 1 && (front ? front : base) == 1);
}

static int cs_changed(const cs_snapshot *s) {
    volatile cs_state *state = CS_STATE;
    if (state->magic != CS_MAGIC || state->rendered.linked != s->linked ||
        state->rendered.address_valid != s->address_valid) return 1;
    if (s->address_valid)
        for (unsigned i=0;i<6;i++)
            if (state->rendered.address[i] != s->address[i]) return 1;
    return 0;
}

/* Only the display-driver task's existing queue receive is redirected here.
 * A timeout may become a type-3 refresh only after taking the SAME buffer
 * semaphore as stock producers, nonblocking. The normal consumer releases it.
 * Real queue messages and errors are returned untouched. This avoids timers,
 * concurrent LVGL access, synthetic BLE readiness, and wake/start requests.
 */
int connection_status_queue_get(uint32_t queue, void *message,
                                uint8_t *priority, uint32_t timeout) {
    int polling = timeout == 0xffffffffu && cs_dashboard_visible();
    int result = CS_QUEUE_GET(queue,message,priority,polling ? CS_POLL_MS : timeout);
    if (result != CS_TIMEOUT || !polling || !message || !cs_dashboard_visible())
        return result;
    cs_snapshot now = cs_read_link();
    if (!cs_changed(&now)) return result;
    uint32_t sem = CS_READ32(0x2007698cu);
    if (!sem || CS_TRY_LOCK(sem) != 1) return result;
    /* Visibility may have changed while trying the semaphore. */
    if (!cs_dashboard_visible()) { CS_UNLOCK(); return result; }
    uint32_t *m = (uint32_t *)message; /* stock receive buffer is word aligned */
    for (unsigned i=0;i<9;i++) m[i]=0;
    m[0]=3; m[5]=640; m[6]=480;
    if (priority) *priority=0;
    return 0;
}

static void cs_text(char out[48], const cs_snapshot *s) {
    const char *prefix = s->linked ? "Connected from: " : "Phone Bluetooth: disconnected";
    unsigned n=0;
    while (prefix[n]) { out[n]=prefix[n]; n++; }
    if (s->linked) {
        if (s->address_valid) {
            static const char hex[]="0123456789ABCDEF";
            for (unsigned i=0;i<6;i++) {
                uint8_t b=s->address[5-i]; /* Bluetooth byte order -> human MAC */
                out[n++]=hex[b>>4]; out[n++]=hex[b&15];
                if (i!=5) out[n++]=':';
            }
        } else {
            const char *unknown="address unavailable";
            for (unsigned i=0;unknown[i];i++) out[n++]=unknown[i];
        }
    }
    out[n]=0;
}

/* 2x Terminus 6x12, one line. The left/right optics can have different offsets:
 * use the same clamped/even origin as the stock 576x288 -> 640x480 copier.
 * The bottom 52 rows are reserved. Left/right render disjoint text lanes so
 * differing phone privacy addresses or one-sided links never overlap optically.
 * Always repaint from stock first; disconnect/shorter identities cannot leave
 * text remnants. Existing app-readiness icon remains a separate indicator.
 */
void connection_status_overlay(uint8_t *fb) {
    if (!fb || !cs_dashboard_visible()) { CS_STATE->magic=0; return; }
    cs_snapshot s=cs_read_link();
    uint32_t x=CS_READ32(0x20000744u)+CS_READ32(0x2000074cu);
    uint32_t y=CS_READ32(0x20000748u)+CS_READ32(0x20076760u);
    x=(x>64 ? 64:x)&~1u; y=(y>192 ? 192:y)&~1u;
    unsigned side=CS_READ8(0x200773e3u);
    if (side!=1 && side!=2) return;
    char text[48]; text[0]=side==2 ? 'L':'R'; text[1]=' ';
    cs_text(text+2,&s);
    unsigned len=0; while(text[len])len++;
    for (unsigned row=y+236;row<y+288;row++)
        for (unsigned col=x;col<x+576;col++) set_pixel4(fb,640,col,row,0);
    unsigned left=x+(576-len*12)/2, top=y+(side==2 ? 237:263);
    for (unsigned c=0;c<len;c++) {
        unsigned ch=(unsigned char)text[c];
        if (ch<FONT_FIRST || ch>FONT_LAST) ch='?';
        for (unsigned row=0;row<12;row++)
            for (unsigned col=0;col<6;col++)
                if (font6x12[ch-FONT_FIRST][row] & (0x80u>>col))
                    for(unsigned dy=0;dy<2;dy++)
                        for(unsigned dx=0;dx<2;dx++)
                            set_pixel4(fb,640,left+c*12+col*2+dx,top+row*2+dy,15);
    }
    CS_FLUSH(fb,640u*480u/2u);
    volatile cs_state *state=CS_STATE;
    state->rendered.linked=s.linked;
    state->rendered.address_valid=s.address_valid;
    for(unsigned i=0;i<6;i++)state->rendered.address[i]=s.address[i];
    state->magic=CS_MAGIC;
}
