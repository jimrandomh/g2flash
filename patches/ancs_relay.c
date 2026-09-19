/* ANCS/1: live, connection-bound relay. No notification history, bridge traffic,
 * or generic current-phone TX queue is permitted here. All entry points run on
 * the Cordio task. Submit directly to the ATT consumer on that task: the stock
 * AttsHandleValueNtf queue stores only a reusable connection ID, NOT a generation.
 * Never put sensitive data on that queue.
 * G2 2.3.0.24 addresses are authenticated by patch_compress.py. */
#include <stdint.h>
#include "cfw_context.h"

typedef struct {
    uint16_t connection;
    uint8_t event, status;
    const uint8_t *value;
    uint16_t length, handle;
} ancs_att_event;
/* Cordio att_api.h: 5 is ATTC_READ_RSP, not a write completion. */
enum { ANCS_WRITE_RSP = 9, ANCS_WRITE_CMD_RSP = 10,
       ANCS_VALUE_NTF = 13, ANCS_VALUE_IND = 14 };
enum { ANCS_DISABLING = 1, ANCS_ENABLING, ANCS_ACTIVE,
       ANCS_RESTORE_DISABLE, ANCS_RESTORE_ENABLE, ANCS_RESTORE_FAILED };
#ifndef ANCS_STOCK_GATE
#define ANCS_STOCK_GATE ((void (*)(ancs_att_event *))0x004d7bedu)
#define ANCS_STOCK_OPEN ((void (*)(uint8_t, uint16_t *))0x004d74c9u)
#define ANCS_STOCK_CLOSE ((void (*)(void))0x004d74d9u)
#define ANCS_STOCK_WRITE ((uint8_t (*)(uint8_t,uint16_t,uint8_t,uint16_t,uint16_t,const uint8_t *,const void *))0x004d6939u)
#define ANCS_WRITE ((void (*)(uint8_t,uint16_t,uint16_t,const uint8_t *))0x004ca6bdu)
/* Same packet ABI and allocator used by AttsHandleValueNtf, but synchronous
 * submission avoids its unsafe ID-only WSF queue. The consumer transfers packet
 * ownership to L2CAP on this connection, or frees it when flow control rejects. */
static void ancs_notify_now(uint8_t connection, uint16_t handle, uint16_t length, const uint8_t *data) {
    uint8_t *packet = ((uint8_t *(*)(uint16_t))0x004c9fcdu)(11 + length);
    if (!packet) return;
    for (unsigned i=0; i<11+length; ++i) packet[i] = 0;
    packet[0] = length+3; packet[1] = (length+3)>>8;
    packet[2] = handle; packet[3] = handle>>8;
    packet[8] = 0x1b; packet[9] = handle; packet[10] = handle>>8;
    for (unsigned i=0; i<length; ++i) packet[11+i] = data[i];
    struct { uint16_t connection; uint8_t event, status; uint8_t *packet; uint8_t slot, reserved[3]; }
        message = {connection,0x21,0,packet,0,{0,0,0}};
    _Static_assert(sizeof(message) == 12, "ATT message ABI");
    ((void (*)(void *))0x0055353du)(&message);
}
#define ANCS_NOTIFY ancs_notify_now
static uint16_t ancs_mtu(uint8_t id) {
    /* attsGetConnCb(id, bearer 0); +16 is pMainCcb, whose first halfword
     * is bearer[0].mtu. Verified against stock 0x5535fc..0x553608. */
    uint8_t *connection = ((uint8_t *(*)(uint8_t,uint8_t))0x00550865u)(id,0);
    return connection ? **(uint16_t **)(connection + 16) : 0;
}
#define ANCS_MTU ancs_mtu
#define ANCS_CCC ((uint8_t (*)(uint8_t,uint8_t))0x00548327u)
#define ANCS_CONNECTION (*(volatile uint8_t *)0x20065cf0u)
#define ANCS_HANDLES (*(uint16_t *volatile *)0x20065cf4u)
#define ANCS_CONTEXT() peekCustomCfwContext()
#define ANCS_ALLOC() getCustomCfwContext()
#define ANCS_LEASE(ctx) ((ctx)->direct_lease_deadline &&     (int32_t)((ctx)->direct_lease_deadline - FW_MS_TICK) > 0)
#endif

static void ancs_clear(customCfwContext *ctx) {
    if (ctx) {
        ctx->ancs_connection = ctx->ancs_stage = 0;
        ctx->ancs_token = 0;
        ctx->ancs_sequence = 0;
    }
}
void cfw_ancs_open(uint8_t connection, uint16_t *handles) {
    ancs_clear(ANCS_CONTEXT());
    ANCS_STOCK_OPEN(connection, handles);
}
void cfw_ancs_close(void) {
    ancs_clear(ANCS_CONTEXT());
    ANCS_STOCK_CLOSE();
}

/* Frames fit the negotiated fixed-bearer MTU (including the minimum MTU). The token is for stale-packet rejection, NOT authority:
 * authority is exclusively the ATT connection passed by Cordio. */
static void ancs_send(customCfwContext *ctx, uint8_t kind, const uint8_t *data, uint16_t length) {
    if (!ctx || !ctx->ancs_connection || ctx->ancs_connection != ANCS_CONNECTION ||
        !ANCS_CCC(ctx->ancs_connection, 2)) return;
    uint16_t mtu = ANCS_MTU(ctx->ancs_connection);
    if (mtu < 23) return;
    uint16_t capacity = mtu - 3;
    if (capacity > 244) capacity = 244;
    uint16_t offset = 0;
    do {
        uint16_t count = length - offset;
        if (count > capacity - 11) count = capacity - 11;
        uint16_t sequence = ctx->ancs_sequence++;
        uint32_t token = ctx->ancs_token;
        uint8_t packet[244];
        packet[0]='A'; packet[1]='N'; packet[2]=1; packet[3]=kind;
        packet[4]=token; packet[5]=token>>8; packet[6]=token>>16; packet[7]=token>>24;
        packet[8]=sequence; packet[9]=sequence>>8;
        packet[10]=(offset == 0 ? 1 : 0) | (offset + count == length ? 2 : 0);
        for (uint16_t i = 0; i < count; ++i) packet[11+i] = data[offset+i];
        ANCS_NOTIFY(ctx->ancs_connection, 0x844, 11+count, packet);
        offset += count;
    } while (offset < length);
}

/* Keep ownership while a CCC write is outstanding. STOP and lease expiry
 * must not race an earlier CCC=0 with a local stock-state reset. In particular,
 * ANCS_STOCK_OPEN does not write CCC and cannot restore a subscription. */
static void ancs_restore(customCfwContext *ctx) {
    if (ctx->ancs_stage == ANCS_RESTORE_DISABLE || ctx->ancs_stage == ANCS_RESTORE_ENABLE) return;
    if (ctx->ancs_stage == ANCS_DISABLING) {
        ctx->ancs_stage = ANCS_RESTORE_DISABLE; /* wait for CCC=0 completion */
    } else if (ctx->ancs_stage == ANCS_ENABLING) {
        ctx->ancs_stage = ANCS_RESTORE_ENABLE; /* CCC=1 already outstanding */
    } else {
        ctx->ancs_stage = ANCS_RESTORE_ENABLE;
        uint8_t enabled[2] = {1,0};
        ANCS_WRITE(ctx->ancs_connection,ANCS_HANDLES[1],2,enabled);
    }
}
static void ancs_restored(customCfwContext *ctx) {
    uint8_t connection = ctx->ancs_connection;
    uint16_t *handles = ANCS_HANDLES;
    ancs_clear(ctx);
    ANCS_STOCK_CLOSE(); ANCS_STOCK_OPEN(connection,handles);
}

/* Replace only the EUS registration pointer; original EUS handling (including
 * the existing SID-f0 hook) remains intact for every non-ANCS command. */
uint8_t cfw_ancs_write(uint8_t connection, uint16_t handle, uint8_t operation,
    uint16_t offset, uint16_t length, const uint8_t *value, const void *attribute) {
    /* 2.3.0's new security characteristic shares this callback. Its writes
     * must always retain the stock handle-specific behavior. */
    if (handle == 0x847 || !value || length < 2 || value[0] != 'A' || value[1] != 'N')
        return ANCS_STOCK_WRITE(connection,handle,operation,offset,length,value,attribute);
    if ((operation != 0x12 && operation != 0x52) || offset || length < 8 || value[2] != 1 || value[3] > 2) return 0x0d;
    /* Never accept app-supplied connection/peer identifiers. This check also
     * excludes the other lens and any other concurrently connected central. */
    if (!connection || connection != ANCS_CONNECTION || !ANCS_HANDLES || !ANCS_CCC(connection,2)) return 0x08;
    uint16_t *handles = ANCS_HANDLES;
    if (!handles[0] || !handles[1] || !handles[2] || !handles[3]) return 0x08;
    uint32_t token = (uint32_t)value[4] | (uint32_t)value[5]<<8 | (uint32_t)value[6]<<16 | (uint32_t)value[7]<<24;
    if (!token) return 0x0d;
    customCfwContext *ctx = ANCS_CONTEXT();
    if (value[3] == 0) {
        if (length != 8) return 0x0d;
        ctx = ANCS_ALLOC();
        if (!ctx) return 0x11;
        if (!ANCS_LEASE(ctx)) return 0x08;
        /* A previous unsubscribe/restore must finish before another START.
         * Otherwise an old CCC completion could be attributed to the new token. */
        if (ctx->ancs_connection && ctx->ancs_stage != ANCS_ACTIVE) {
            if (ctx->ancs_stage == ANCS_RESTORE_FAILED) ancs_restore(ctx);
            return 0x08;
        }
        /* Purge the stock pending list/parser/timer before taking ownership.
         * A new NS subscription asks iOS to enumerate its current notifications. */
        ANCS_STOCK_CLOSE(); ANCS_STOCK_OPEN(connection,handles);
        ancs_clear(ctx);
        ctx->ancs_connection = connection; ctx->ancs_token = token; ctx->ancs_stage = ANCS_DISABLING;
        uint8_t status = 0, disabled[2] = {0,0};
        ancs_send(ctx,0,&status,1);
        ANCS_WRITE(connection,handles[1],2,disabled);
        return 0;
    }
    if (!ctx || ctx->ancs_connection != connection || ctx->ancs_token != token) return 0x08;
    if (value[3] == 1) {
        if (length != 8) return 0x0d;
        ancs_restore(ctx);
        return 0;
    }
    /* Only ANCS CP commands; never expose an arbitrary GATT read/write API. */
    if (ctx->ancs_stage != ANCS_ACTIVE || length < 9 || length > 80) return 0x0d;
    const uint8_t *cp = value + 8; uint16_t size = length - 8;
    if (!((cp[0] == 0 && size >= 6) ||
          (cp[0] == 1 && size >= 4 && cp[size-2] == 0 && cp[size-1] == 0) ||
          (cp[0] == 2 && size == 6 && cp[5] <= 1))) return 0x0d;
    ANCS_WRITE(connection,handles[2],size,cp);
    return 0;
}

void cfw_ancs_event(ancs_att_event *event) {
    customCfwContext *ctx = ANCS_CONTEXT();
    if (!ctx || !ctx->ancs_connection) {
        /* The newly routed write events were discarded by stock. Preserve
         * that behavior whenever Faceclaw has not taken over ANCS. */
        if (event && event->event != ANCS_WRITE_RSP && event->event != ANCS_WRITE_CMD_RSP)
            ANCS_STOCK_GATE(event);
        return;
    }
    if (!event || event->connection != ctx->ancs_connection ||
        event->connection != ANCS_CONNECTION || !ANCS_HANDLES) return;
    uint16_t *handles = ANCS_HANDLES;
    if (ctx->ancs_stage < ANCS_RESTORE_DISABLE && !ANCS_LEASE(ctx)) {
        uint8_t status = 2; ancs_send(ctx,0,&status,1);
        ancs_restore(ctx);
        /* Continue: this very event may complete the outstanding CCC write. */
    }
    if (event->event == ANCS_WRITE_RSP && event->handle == handles[1]) {
        if (ctx->ancs_stage == ANCS_DISABLING || ctx->ancs_stage == ANCS_RESTORE_DISABLE) {
            int restoring = ctx->ancs_stage == ANCS_RESTORE_DISABLE || event->status;
            if (event->status) { uint8_t status = 2; ancs_send(ctx,0,&status,1); }
            ctx->ancs_stage = restoring ? ANCS_RESTORE_ENABLE : ANCS_ENABLING;
            uint8_t enabled[2] = {1,0}; ANCS_WRITE(ctx->ancs_connection,handles[1],2,enabled);
        } else if (ctx->ancs_stage == ANCS_ENABLING || ctx->ancs_stage == ANCS_RESTORE_ENABLE) {
            if (event->status) {
                int restoring = ctx->ancs_stage == ANCS_RESTORE_ENABLE;
                ctx->ancs_stage = ANCS_RESTORE_FAILED;
                uint8_t status = 2; ancs_send(ctx,0,&status,1);
                /* One restoration attempt after an initial subscribe failure.
                 * If restoring also fails, retain explicit failure state; a
                 * later START/STOP retries it instead of claiming success. */
                if (!restoring) ancs_restore(ctx);
            } else if (ctx->ancs_stage == ANCS_RESTORE_ENABLE) {
                ancs_restored(ctx);
            } else {
                ctx->ancs_stage = ANCS_ACTIVE;
                uint8_t status = 1; ancs_send(ctx,0,&status,1);
            }
        }
        return;
    }
    if (ctx->ancs_stage >= ANCS_RESTORE_DISABLE) return;
    if (event->event == ANCS_WRITE_RSP) {
        if (ctx->ancs_stage == ANCS_ACTIVE && event->handle == handles[2]) {
            if (event->status) { uint8_t status = 2; ancs_send(ctx,0,&status,1); }
            else ancs_send(ctx,3,0,0);
        }
        return;
    }
    if (event->event != ANCS_VALUE_NTF && event->event != ANCS_VALUE_IND) return;
    if (event->status) { uint8_t status = 2; ancs_send(ctx,0,&status,1); return; }
    if (!event->value || !event->length || event->length > 512) return;
    if (event->handle == handles[0] && event->length == 8)
        ancs_send(ctx,1,event->value,event->length);
    else if (event->handle == handles[3]) ancs_send(ctx,2,event->value,event->length);
    /* The phone owns attribute requests/actions while enabled. Do not run the
     * stock parser concurrently or allow its completion path to dismiss items. */
}
