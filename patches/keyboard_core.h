#pragma once
#include <stdint.h>

/* Private sid 09 fields 130 (control) / 131 (event), KB v1.
 * All state except the SPSC indices and poll flag belongs to the WSF task.
 * See docs/keyboard.md for the wire format and hardware validation boundary. */
#define KB_CHUNK 96u
#define KB_HEADER 24u
#define KB_SLOTS 32u
#define KB_CONTROL_MAX 80u
#define KB_WSF_EVENT 0xfeu
#define KB_WSF_MAGIC 0x4b42u

enum { KB_QUERY, KB_ENABLE, KB_DISABLE, KB_SCAN, KB_CONNECT, KB_DISCONNECT,
       KB_PAIR, KB_AUTH, KB_COMPARE, KB_FIND, KB_READ, KB_WRITE, KB_WATCH };
enum { KB_STATUS=1, KB_ATT, KB_INPUT, KB_DM, KB_ADVERT };
enum { KB_OK, KB_BAD_REQUEST, KB_OFF, KB_BUSY, KB_STALE, KB_NO_SPACE,
       KB_NOT_CONNECTED, KB_NOT_ENCRYPTED, KB_TIMEOUT, KB_NO_MEMORY };

typedef struct { uint16_t param; uint8_t event, status; } kb_hdr;
typedef struct {
    kb_hdr hdr;
    const uint8_t *value;
    uint16_t len, handle;
    uint8_t continuing;
    uint16_t mtu;
} kb_att;
typedef struct {
    kb_hdr hdr;
    const uint8_t *data;
    uint8_t len;
    int8_t rssi;
    uint8_t type, addr_type, addr[6];
} kb_advert;
typedef struct {
    uint8_t len, data[KB_HEADER + KB_CHUNK];
} kb_record;
typedef struct {
    uint32_t head, tail;                 /* release/acquire publication */
    uint32_t sequence, drops, reported_drops, deadline, heartbeat, scan_deadline;
    uint32_t timer, poll_pending, timer_armed, active;
    uint16_t epoch, pending, pending_handle;
    uint16_t watches[16];
    uint8_t enabled, conn, connected, encrypted, scanning, closing;
    uint8_t pending_event, watch_count, auth, compare;
    uint8_t notify_buf[KB_HEADER + KB_CHUNK + 10];
    kb_record queue[KB_SLOTS];
} kb_state;

static uint16_t kb_u16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static void kb_w16(uint8_t *p, uint32_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static void kb_w32(uint8_t *p, uint32_t v) {
    kb_w16(p,v); kb_w16(p+2,v>>16);
}
static void kb_copy(uint8_t *to, const uint8_t *from, uint32_t n) {
    for (uint32_t i=0;i<n;i++) to[i]=from[i];
}

/* Platform contracts: called only on WSF, except kb_now and kb_send on timer. */
static uint32_t kb_now(void);
static void kb_kick(kb_state *s);
static int kb_scan_available(void);
static void kb_scan_start(void);
static int kb_can_connect(uint8_t type, const uint8_t *addr);
static uint8_t kb_open(uint8_t type, const uint8_t *addr);
static void kb_close(uint8_t conn);
static int kb_pair(uint8_t conn);
static void kb_auth(uint8_t conn, const uint8_t *passkey);
static void kb_compare(uint8_t conn, uint8_t accept);
static int kb_request(uint8_t conn, uint8_t op, uint16_t handle, uint16_t arg,
                      const uint8_t *data, uint16_t len);
static void kb_ind_confirm(uint8_t conn);
static void kb_master_dm(kb_hdr *msg);
static int kb_master_security(kb_hdr *msg);

/* Queue an entire event or none of it. Long ATT values are fragmented without
 * truncation. A sequence is consumed even on overflow; a heartbeat exposes a
 * final lost release even when no further keys are pressed. */
static void kb_emit(kb_state *s, uint8_t kind, uint16_t request, uint16_t code,
                    uint16_t value, const uint8_t *data, uint16_t len) {
    uint32_t seq=++s->sequence;
    uint32_t count=len ? (len+KB_CHUNK-1)/KB_CHUNK : 1;
    uint32_t h=__atomic_load_n(&s->head,__ATOMIC_RELAXED);
    uint32_t t=__atomic_load_n(&s->tail,__ATOMIC_ACQUIRE);
    if (len>512 || count>KB_SLOTS-(h-t) || (len && !data)) { s->drops++; return; }
    uint32_t stamp=kb_now();
    for (uint32_t off=0,i=0;i<count;i++) {
        uint32_t n=len-off; if (n>KB_CHUNK) n=KB_CHUNK;
        kb_record *r=&s->queue[(h+i)%KB_SLOTS];
        uint8_t *p=r->data;
        p[0]='K'; p[1]='B'; p[2]=1; p[3]=kind;
        kb_w32(p+4,seq); kb_w16(p+8,s->epoch); kb_w16(p+10,request);
        kb_w16(p+12,code); kb_w16(p+14,value); kb_w16(p+16,len);
        kb_w16(p+18,off); kb_w32(p+20,stamp);
        if (n) kb_copy(p+KB_HEADER,data+off,n);
        r->len=(uint8_t)(KB_HEADER+n); off+=n;
    }
    __atomic_store_n(&s->head,h+count,__ATOMIC_RELEASE);
    kb_kick(s);
}
static void kb_status(kb_state *s, uint16_t req, uint16_t error) {
    uint8_t p[10]={s->enabled,s->conn,s->connected,s->encrypted,s->scanning,s->pending_event};
    kb_w32(p+6,s->drops);
    uint32_t before=s->drops;
    kb_emit(s,KB_STATUS,req,error,0,p,sizeof p);
    if (s->drops==before) s->reported_drops=before;
}
static void kb_clear_link(kb_state *s) {
    s->conn=s->connected=s->encrypted=s->closing=0;
    s->pending=s->pending_event=s->watch_count=s->auth=s->compare=0;
    s->deadline=0;
}
static void kb_disconnect(kb_state *s) {
    s->watch_count=s->encrypted=s->auth=s->compare=0;
    if (s->conn && !s->closing) { s->closing=1; kb_close(s->conn); }
}
static void kb_poll(kb_state *s) {
    __atomic_store_n(&s->poll_pending,0,__ATOMIC_RELEASE);
    uint32_t now=kb_now();
    if (s->deadline && (int32_t)(now-s->deadline)>=0) {
        kb_status(s,s->pending,KB_TIMEOUT);
        s->deadline=0; s->pending=s->pending_event=0;
        /* Do not reuse the ATT transaction or connection id after a timeout.
         * Keep ownership until DM_CLOSE; a late callback stays isolated. */
        kb_disconnect(s);
    }
    if (s->scanning && (int32_t)(now-s->scan_deadline)>=0 && kb_scan_available()) {
        s->scanning=0; kb_status(s,0,KB_TIMEOUT);
    }
    if ((int32_t)(now-s->heartbeat)>=1000) {
        s->heartbeat=now;
        if (s->enabled || s->conn || s->scanning || s->drops!=s->reported_drops) kb_status(s,0,KB_OK);
    }
}
static int kb_valid_control(const uint8_t *p, uint32_t n) {
    if (!p || n<8 || n>KB_CONTROL_MAX || p[0]!='K' || p[1]!='B' || p[2]!=1 || p[3]>KB_WATCH)
        return 0;
    uint32_t body=n-8;
    switch (p[3]) {
    case KB_CONNECT: return body==7 && p[8]<=3;
    case KB_AUTH: return body==3 && ((uint32_t)p[8]|((uint32_t)p[9]<<8)|((uint32_t)p[10]<<16))<1000000;
    case KB_COMPARE: return body==1 && p[8]<=1;
    case KB_FIND: return body==4 && kb_u16(p+8) && kb_u16(p+10)>=kb_u16(p+8);
    case KB_READ: return body==4 && kb_u16(p+8) && kb_u16(p+10)<=512;
    case KB_WRITE: return body>=4 && body<=22 && kb_u16(p+8) && p[10]<=1 && p[11]==body-4;
    case KB_WATCH: return body>=1 && p[8]<=16 && body==1+2u*p[8];
    default: return body==0;
    }
}
static void kb_control(kb_state *s, const uint8_t *p, uint32_t n) {
    if (!kb_valid_control(p,n)) return;
    uint8_t op=p[3]; uint16_t req=kb_u16(p+4),epoch=kb_u16(p+6),error=KB_OK;
    const uint8_t *b=p+8;
    if (op==KB_QUERY) { kb_status(s,req,KB_OK); return; }
    if (op==KB_ENABLE) { s->enabled=1; kb_status(s,req,KB_OK); return; }
    if (epoch!=s->epoch) { kb_status(s,req,KB_STALE); return; }
    if (op==KB_DISABLE) {
        s->enabled=0; kb_disconnect(s);
        /* A bounded scan is allowed to finish; never stop somebody else's scan. */
    } else if (!s->enabled) error=KB_OFF;
    else if (op==KB_SCAN) {
        if (s->scanning || s->conn || !kb_scan_available()) error=KB_BUSY;
        else { s->scanning=1; s->scan_deadline=kb_now()+7000; kb_scan_start(); }
    } else if (op==KB_CONNECT) {
        if (s->conn || s->scanning) error=KB_BUSY;
        else if ((error=(uint16_t)kb_can_connect(b[0],b+1))==KB_OK) {
            uint8_t conn=kb_open(b[0],b+1);
            if (!conn || conn>3) error=KB_NO_SPACE;
            else {
                s->conn=conn; s->epoch++; if (!s->epoch) s->epoch++;
                s->deadline=kb_now()+30000; s->pending=req;
            }
        }
    } else if (op==KB_DISCONNECT) kb_disconnect(s);
    else if (!s->connected || s->closing) error=KB_NOT_CONNECTED;
    else if (op==KB_PAIR) {
        error=(uint16_t)kb_pair(s->conn);
        if (!error) s->deadline=kb_now()+60000;
    } else if (op==KB_AUTH) {
        if (!s->auth) error=KB_BAD_REQUEST;
        else { s->auth=0; kb_auth(s->conn,b); }
    } else if (op==KB_COMPARE) {
        if (!s->compare) error=KB_BAD_REQUEST;
        else { s->compare=0; kb_compare(s->conn,b[0]); }
    } else if (!s->encrypted) error=KB_NOT_ENCRYPTED;
    else if (op==KB_WATCH) {
        for (unsigned i=0;i<b[0];i++) if (!kb_u16(b+1+2*i)) error=KB_BAD_REQUEST;
        if (!error) {
            s->watch_count=b[0];
            for (unsigned i=0;i<s->watch_count;i++) s->watches[i]=kb_u16(b+1+2*i);
        }
    } else if (s->pending_event) error=KB_BUSY;
    else {
        uint16_t handle=kb_u16(b),arg=kb_u16(b+2);
        s->pending=req; s->pending_handle=handle;
        s->pending_event=op==KB_FIND?2:op==KB_READ?(arg?6:5):(b[2]?10:9);
        s->deadline=kb_now()+30000;
        if (!kb_request(s->conn,op,handle,arg,b+4,(uint16_t)(n-12))) {
            s->pending=s->pending_event=0; s->deadline=0; error=KB_NO_MEMORY;
        } else return; /* asynchronous ATT response, not a premature success */
    }
    kb_status(s,req,error);
}
/* Return 1 only for events owned by the keyboard. All other links and global
 * events retain the original stock dispatch path. */
static int kb_event(kb_state *s, kb_hdr *h) {
    if (!s || !h) return 0;
    uint8_t e=h->event;
    if (e==0x20) {
        kb_clear_link(s); s->scanning=s->enabled=0; s->epoch++;
        kb_status(s,0,KB_OK); return 0;
    }
    if (s->scanning && e>=0x24 && e<=0x26) {
        if (e==0x25 || (e==0x24 && h->status)) s->scanning=0;
        if (e==0x26 && s->enabled) {
            const kb_advert *a=(const kb_advert *)h;
            if (a->len<=31 && a->addr_type<=3 && a->data) {
                uint8_t b[41]; b[0]=a->addr_type; kb_copy(b+1,a->addr,6);
                b[7]=(uint8_t)a->rssi; b[8]=a->type; b[9]=a->len;
                kb_copy(b+10,a->data,a->len);
                kb_emit(s,KB_ADVERT,0,0,0,b,(uint16_t)(10+a->len));
            }
        } else kb_status(s,0,h->status);
        return 1;
    }
    if (!s->conn || h->param!=s->conn) return 0;
    if (e>=2 && e<0x19) {
        const kb_att *a=(const kb_att *)h;
        if (e==13 || e==14) {
            if (s->enabled && s->encrypted && !s->closing)
                for (unsigned i=0;i<s->watch_count;i++) if (s->watches[i]==a->handle) {
                    kb_emit(s,KB_INPUT,0,e,a->handle,a->value,a->len); break;
                }
            if (e==14) kb_ind_confirm(s->conn);
        } else if (s->pending_event==e) {
            uint16_t req=s->pending;
            s->pending=s->pending_event=0; s->deadline=0;
            kb_emit(s,KB_ATT,req,(uint16_t)(e|(h->status<<8)),a->handle,a->value,a->len);
        }
        return 1;
    }
    /* Connection-scoped DM only: don't interpret global HCI params as IDs. */
    if ((e>=0x27 && e<=0x33) || e==0x35 || e==0x36 || e==0x39 || e==0x40 || e==0x57) {
        kb_master_dm(h);
        if (!kb_master_security(h)) kb_disconnect(s);
        if (e==0x27) {
            s->connected=h->status==0;
            s->deadline=0;
            if (h->status) kb_clear_link(s);
            else if (!s->enabled || s->closing) { s->closing=0; kb_disconnect(s); }
        } else if (e==0x28) kb_clear_link(s);
        else if (e==0x2c) { s->encrypted=h->status==0; s->deadline=0; }
        else if (e==0x2b || e==0x2d) { s->deadline=0; kb_disconnect(s); }
        else if (e==0x2e) {
            if (((uint8_t *)h)[4]) kb_disconnect(s); /* OOB is not a passkey */
            else s->auth=1;
        }
        else if (e==0x35) s->compare=1;
        /* Never relay bonding keys. Only auth prompts have payloads. */
        uint8_t *body=(uint8_t *)h+4, compare[4];
        if (e==0x35) {
            /* DmSecGetCompareValue: big-endian low word of the 128-bit
             * confirm value, modulo 1,000,000. Only send the display value. */
            uint32_t value=((uint32_t)body[12]<<24)|((uint32_t)body[13]<<16)|
                           ((uint32_t)body[14]<<8)|body[15];
            kb_w32(compare,value%1000000u); body=compare;
        }
        kb_emit(s,KB_DM,0,(uint16_t)(e|(h->status<<8)),0,body,e==0x2e?2:e==0x35?4:0);
        kb_status(s,0,KB_OK);
        return 1;
    }
    return 0;
}
