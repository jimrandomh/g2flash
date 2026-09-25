#include "keyboard_core.h"
#include "cfw_context.h"
#include "malloc.h"
#include <stddef.h>
_Static_assert(sizeof(kb_hdr)==4 && sizeof(kb_att)==16, "Cordio event ABI");
_Static_assert(offsetof(kb_advert,addr)==12, "Cordio advertising ABI");

/* Exact 2.3.0.24 Cordio ABI. Guarded in stock_abi_230.json; evidence and donor
 * layouts are recorded in docs/keyboard-abi.md. No stack/controller limits,
 * global pairing configuration, ring singleton, or boot initialization changes. */
#define KB_ALLOC_MSG ((void *(*)(uint16_t))0x004d8e77u)
#define KB_SEND_MSG ((void (*)(uint8_t,void *))0x004d8e93u)
#define KB_APP_HANDLER (*(volatile uint8_t *)(0x20073e64u+0x56u))
#define KB_APP_CONN ((uint8_t *)0x20073908u)
#define KB_CCB ((volatile uint8_t *)0x20073c30u)
#define KB_SCAN_STATE (*(volatile uint8_t *)(0x200769b4u+0x18u))
#define KB_DB_FIND ((void *(*)(uint8_t,const uint8_t *))0x004843a9u)
#define KB_DB_COUNT ((uint8_t (*)(uint8_t))0x00483c2du)
#define KB_APP_OPEN ((uint8_t (*)(uint8_t,uint8_t,const uint8_t *,void *))0x0051d249u)
#define KB_DM_CLOSE ((void (*)(uint8_t,uint8_t,uint8_t))0x004ce3f5u)
#define KB_SEC_START ((void (*)(uint8_t,uint8_t,void *))0x0051c7e5u)
#define KB_AUTH_RSP ((void (*)(uint8_t,uint8_t,const uint8_t *))0x004e6685u)
#define KB_COMPARE_RSP ((void (*)(uint8_t,uint8_t))0x00550239u)
#define KB_SCAN_START ((void (*)(uint8_t,uint8_t,const uint8_t *,uint8_t,uint16_t,uint16_t))0x00579c13u)
#define KB_MASTER_DM ((void (*)(void *))0x0051d0c3u)
#define KB_MASTER_SEC ((void (*)(void *))0x0051d163u)
#define KB_ATT_ALLOC ((uint8_t *(*)(uint16_t))0x004c9fcdu)
#define KB_ATT_REQUEST ((void (*)(uint8_t,uint16_t,uint8_t,void *,uint8_t))0x004ca561u)
#define KB_ATT_CONFIRM ((void (*)(uint8_t))0x004ca759u)

static uint32_t kb_now(void) { return FW_MS_TICK; }
static void kb_kick(kb_state *s) {
    if (!__atomic_exchange_n(&s->timer_armed,1,__ATOMIC_ACQ_REL))
        if (FW_TIMER_START(s->timer,20)!=0)
            __atomic_store_n(&s->timer_armed,0,__ATOMIC_RELEASE);
}
/* Publish timer demand from the WSF-owned state. Timer callbacks only read the
 * atomic snapshot; they never race a connection-state byte or stop a producer's
 * newly started timer. Disabled/queried-only instances park after draining. */
static void kb_schedule(kb_state *s) {
    uint32_t active=s->enabled || s->conn || s->scanning || s->drops!=s->reported_drops;
    __atomic_store_n(&s->active,active,__ATOMIC_RELEASE);
    if (active || __atomic_load_n(&s->head,__ATOMIC_ACQUIRE)!=__atomic_load_n(&s->tail,__ATOMIC_ACQUIRE)) kb_kick(s);
}
static int kb_scan_available(void) {
    if (KB_SCAN_STATE) return 0;
    for (unsigned i=0;i<3;i++) {
        volatile uint8_t *c=KB_CCB+0x30*i;
        if (c[0x16] && c[0xc]==255 && c[0xd]==255) return 0;
    }
    return 1;
}
static void kb_scan_start(void) {
    const uint8_t active=1;
    KB_SCAN_START(1,0,&active,1,5000,0);
}
static int kb_db_space(void) {
    unsigned central=KB_DB_COUNT(1),peripheral=KB_DB_COUNT(0);
    return central<5 && central+peripheral<10;
}
static int kb_can_connect(uint8_t type, const uint8_t *addr) {
    if (!kb_scan_available()) return KB_BUSY;
    unsigned free=0;
    for (unsigned i=0;i<3;i++) if (!KB_CCB[0x30*i+0x16]) free++;
    if (!free) return KB_NO_SPACE;
    if (!KB_DB_FIND(type,addr) && !kb_db_space()) return KB_NO_SPACE;
    return KB_OK;
}
static uint8_t kb_open(uint8_t type, const uint8_t *addr) {
    return KB_APP_OPEN(1,type,addr,KB_DB_FIND(type,addr));
}
static void kb_close(uint8_t conn) { KB_DM_CLOSE(3,conn,0x13); }
static uint8_t *kb_app_conn(uint8_t conn) { return KB_APP_CONN+0x44u*(conn-1u); }
static int kb_pair(uint8_t conn) {
    uint8_t *app=kb_app_conn(conn);
    if (app[8]) return KB_BUSY; /* stock security operation in flight */
    if (!*(void **)app && !kb_db_space()) return KB_NO_SPACE;
    KB_SEC_START(conn,1,app);
    return KB_OK;
}
static void kb_auth(uint8_t conn, const uint8_t *passkey) { KB_AUTH_RSP(conn,3,passkey); }
static void kb_compare(uint8_t conn, uint8_t accept) { KB_COMPARE_RSP(conn,accept); }
static void kb_master_dm(kb_hdr *msg) { KB_MASTER_DM(msg); }
static int kb_master_security(kb_hdr *msg) {
    /* The stock security-request handler can create a bond too. Check at the
     * allocation point, not only at CONNECT (the ring may pair meanwhile). */
    if ((msg->event==0x27 || msg->event==0x32) &&
        !*(void **)kb_app_conn((uint8_t)msg->param) && !kb_db_space()) return 0;
    KB_MASTER_SEC(msg);
    return 1;
}
static void kb_ind_confirm(uint8_t conn) { KB_ATT_CONFIRM(conn); }
static int kb_request(uint8_t conn, uint8_t op, uint16_t handle, uint16_t arg,
                      const uint8_t *data, uint16_t len) {
    /* Same packet layout and attcRequest path as stock AttcFindInfoReq,
     * AttcReadReq and AttcWriteReq. Read Blob wrapper is omitted from this
     * donor, but its request/response table entries remain linked. Always
     * continuing=FALSE: one PDU per phone request, with explicit pagination. */
    unsigned size=op==KB_FIND || (op==KB_READ && arg)?5:op==KB_WRITE?3+len:3;
    uint8_t *p=KB_ATT_ALLOC((uint16_t)(size+8));
    if (!p) return 0;
    for (unsigned i=0;i<size+8;i++) p[i]=0;
    kb_w16(p,size);
    uint8_t event;
    if (op==KB_FIND) {
        kb_w16(p+2,handle); kb_w16(p+4,arg); p[8]=4; event=2;
    } else if (op==KB_READ) {
        p[8]=arg?12:10; kb_w16(p+9,handle); event=arg?6:5;
        if (arg) { kb_w16(p+2,arg); kb_w16(p+11,arg); }
    } else {
        p[8]=(arg&255)?0x52:0x12; event=(arg&255)?10:9;
        kb_w16(p+9,handle); kb_copy(p+11,data,len);
    }
    KB_ATT_REQUEST(conn,handle,event,p,0);
    return 1;
}

/* A private WSF message contains the validated original control. Allocation,
 * copying and posting are the only operations on the settings thread. */
static int kb_post(const uint8_t *data, uint32_t len) {
    uint8_t *p=KB_ALLOC_MSG((uint16_t)(8+len));
    if (!p) return 0;
    kb_w16(p,KB_WSF_MAGIC); p[2]=KB_WSF_EVENT; p[3]=0;
    kb_w16(p+4,len); p[6]=p[7]=0;
    if (len) kb_copy(p+8,data,len);
    KB_SEND_MSG(KB_APP_HANDLER,p);
    return 1;
}
/* Timer thread: no BLE host mutation and no queue writes here. The firmware
 * sender can wait on the BLE task, so it must never run inside kb_dispatch. */
__attribute__((used)) static void kb_tick(void *arg) {
    kb_state *s=arg;
    for (unsigned i=0;i<4;i++) {
        uint32_t t=__atomic_load_n(&s->tail,__ATOMIC_RELAXED);
        if (t==__atomic_load_n(&s->head,__ATOMIC_ACQUIRE)) break;
        kb_record *r=&s->queue[t%KB_SLOTS];
        uint8_t *p=s->notify_buf;
        p[0]=8; p[1]=3; p[2]=16; p[3]=0;
        unsigned len=pb_append_bytes_field(p,4,sizeof s->notify_buf,131,r->data,r->len);
        ((send_fn)FW_NOTIFY_SEND)(1,9,p,len);
        /* A failed send is a visible sequence gap; don't replay stale typing. */
        __atomic_store_n(&s->tail,t+1,__ATOMIC_RELEASE);
    }
    if (!__atomic_exchange_n(&s->poll_pending,1,__ATOMIC_ACQ_REL))
        if (!kb_post(0,0)) __atomic_store_n(&s->poll_pending,0,__ATOMIC_RELEASE);
    __atomic_store_n(&s->timer_armed,0,__ATOMIC_RELEASE);
    if (__atomic_load_n(&s->active,__ATOMIC_ACQUIRE) ||
        __atomic_load_n(&s->head,__ATOMIC_ACQUIRE)!=__atomic_load_n(&s->tail,__ATOMIC_ACQUIRE)) kb_kick(s);
}
void keyboard_apply_control(const uint8_t *data, uint32_t len) {
    if (!kb_valid_control(data,len) || FW_SIDE_ID()!=1) return;
    customCfwContext *ctx=getCustomCfwContext();
    if (!ctx) return;
    /* Allocate before publishing; concurrent settings workers may race, so
     * only the CAS winner keeps its timer/state. No published state is freed. */
    kb_state *s=__atomic_load_n((kb_state **)&ctx->keyboard,__ATOMIC_ACQUIRE);
    if (!s) {
        if (data[3]!=KB_ENABLE && data[3]!=KB_QUERY && data[3]!=KB_DISABLE) return;
        s=cfw_malloc(sizeof *s);
        if (!s) return;
        for (unsigned i=0;i<sizeof *s;i++) ((uint8_t *)s)[i]=0;
        s->timer=FW_TIMER_NEW(CFW_FN_ADDR(kb_tick),0,s,0);
        if (!s->timer) { FW_FREE(s); return; }
        kb_state *expected=0;
        if (!__atomic_compare_exchange_n((kb_state **)&ctx->keyboard,&expected,s,0,
                                         __ATOMIC_RELEASE,__ATOMIC_ACQUIRE)) {
            FW_TIMER_DELETE(s->timer); FW_FREE(s); s=expected;
        }
    }
    kb_post(data,len);
}
__attribute__((used,noinline)) int keyboard_dispatch(uint32_t mask, kb_hdr *msg) {
    (void)mask;
    customCfwContext *ctx=peekCustomCfwContext();
    kb_state *s=ctx?__atomic_load_n((kb_state **)&ctx->keyboard,__ATOMIC_ACQUIRE):0;
    if (!s || !msg) return 0;
    if (msg->event==KB_WSF_EVENT && msg->param==KB_WSF_MAGIC) {
        uint8_t *p=(uint8_t *)msg; unsigned len=kb_u16(p+4);
        if (!len) kb_poll(s);
        else if (len<=KB_CONTROL_MAX) kb_control(s,p+8,len);
        kb_schedule(s);
        return 1;
    }
    int handled=kb_event(s,msg);
    kb_schedule(s);
    return handled;
}
/* Replace precisely push {r3-r5,lr}; sub sp,#16. Restore arguments and original
 * LR on pass-through, then reproduce the displaced instructions. No heap or
 * BLE calls on the dormant path. The preserved 4-word push keeps 8B alignment. */
__attribute__((naked)) void keyboard_dispatch_entry(void) {
    __asm volatile(
        "push {r0-r2,lr}\n"
        "bl keyboard_dispatch\n"
        "cmp r0,#0\n"
        "pop {r0-r2,lr}\n"
        "bne 1f\n"
        "push {r3-r5,lr}\n"
        "sub sp,#16\n"
        "movw r12,#0xc1f1\n"
        "movt r12,#0x004c\n"
        "bx r12\n"
        "1: bx lr\n"
    );
}

/* Only the keyboard's own Pairing Request/Response advertises KeyboardDisplay.
 * Stock has NoInputNoOutput. Override the two I/O-capability loads, never the
 * shared pSmpCfg pointer: ring and phone pairing keep their original policy. */
__attribute__((used,noinline)) uint32_t keyboard_io_cap(const uint8_t *ccb, uint32_t stock) {
    customCfwContext *ctx=peekCustomCfwContext();
    kb_state *s=ctx?__atomic_load_n((kb_state **)&ctx->keyboard,__ATOMIC_ACQUIRE):0;
    return s && s->enabled && s->conn && ccb[0x3d]==s->conn ? 4u : stock;
}
__attribute__((naked)) void keyboard_io_cap_entry(void) {
    __asm volatile(
        "push {r0,r1,r3,r4,r12,lr}\n"
        "mrs r4,APSR\n"
        "mov r0,r5\n"
        "ldr r1,[r1]\n"
        "ldrb r1,[r1,#4]\n"
        "bl keyboard_io_cap\n"
        "mov r2,r0\n"
        "msr APSR_nzcvq,r4\n"
        "pop {r0,r1,r3,r4,r12,pc}\n"
    );
}

/* Observe failures in the stock ATT -> application copy queue too. Without
 * this seam a lost release upstream of kb_event would not consume a relay
 * sequence number and the next heartbeat could incorrectly preserve it. */
__attribute__((used,noinline)) void *keyboard_att_alloc(uint32_t size, const kb_att *event) {
    void *message=KB_ALLOC_MSG((uint16_t)size);
    if (!message) {
        customCfwContext *ctx=peekCustomCfwContext();
        kb_state *s=ctx?__atomic_load_n((kb_state **)&ctx->keyboard,__ATOMIC_ACQUIRE):0;
        if (s && s->conn && event->hdr.param==s->conn) { s->sequence++; s->drops++; kb_schedule(s); }
    }
    return message;
}
__attribute__((naked)) void keyboard_att_alloc_entry(void) {
    __asm volatile("mov r1,r5\n" "b keyboard_att_alloc\n");
}

/* DM_CLOSE must not be lost: otherwise a later ring could reuse the keyboard's
 * connId. Preserve normal queued dispatch; only allocation failure takes the
 * bounded keyboard path directly. This callback is already on WSF, after the
 * ATT/SMP DM clients (0/1) and before application client 3 returns. */
__attribute__((used,noinline)) void *keyboard_dm_alloc(uint32_t size, kb_hdr *event) {
    void *message=KB_ALLOC_MSG((uint16_t)size);
    if (!message) {
        customCfwContext *ctx=peekCustomCfwContext();
        kb_state *s=ctx?__atomic_load_n((kb_state **)&ctx->keyboard,__ATOMIC_ACQUIRE):0;
        if (s) { s->sequence++; s->drops++; kb_event(s,event); kb_schedule(s); }
    }
    return message;
}
__attribute__((naked)) void keyboard_dm_alloc_entry(void) {
    __asm volatile("mov r1,r6\n" "b keyboard_dm_alloc\n");
}
