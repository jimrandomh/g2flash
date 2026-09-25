#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "keyboard_core.h"
#include "protobuf.c"

static uint32_t now=100;
static unsigned scans,closes,requests,confirms,dm_calls,sec_calls,auth_calls;
static int scan_ok=1,space=KB_OK,security_ok=1,alloc_ok=1;
static uint32_t kb_now(void) { return now; }
static unsigned timer_pending,timer_starts,phone_sends,poll_queued;
static int fail_timer,fail_post,in_wsf,inject_after_drain;
static kb_state *timer_state;
static int host_timer_start(uint32_t timer,uint32_t ms) {
    (void)timer;assert(ms==20);timer_starts++;
    if(fail_timer)return -1;
    timer_pending=1;return 0;
}
static int host_notify(int type,int sid,uint8_t *p,unsigned n) {
    assert(!in_wsf && type==1 && sid==9 && n>=7 && n<=130);
    assert(p[0]==8 && p[1]==3 && p[4]==0x9a && p[5]==8 && p[6]==n-7);
    phone_sends++;return 0;
}
static int kb_post(const uint8_t *data,uint32_t len) {
    assert(!data && !len);
    if(inject_after_drain) { inject_after_drain=0;kb_status(timer_state,1,0); }
    if(fail_post)return 0;
    poll_queued++;return 1;
}
typedef int (*send_fn)(int,int,uint8_t *,unsigned);
#define FW_TIMER_START host_timer_start
#define FW_NOTIFY_SEND host_notify
#include "keyboard_timer_functions.inc"
static int kb_scan_available(void) { return scan_ok; }
static void kb_scan_start(void) { scans++; }
static int kb_can_connect(uint8_t t,const uint8_t *a) { (void)t;(void)a;return space; }
static uint8_t kb_open(uint8_t t,const uint8_t *a) { (void)t;(void)a;return 3; }
static void kb_close(uint8_t c) { assert(c==3);closes++; }
static int kb_pair(uint8_t c) { assert(c==3);return space; }
static void kb_auth(uint8_t c,const uint8_t *p) { assert(c==3);(void)p;auth_calls++; }
static void kb_compare(uint8_t c,uint8_t a) { assert(c==3 && a<=1); }
static int kb_request(uint8_t c,uint8_t op,uint16_t h,uint16_t a,const uint8_t *p,uint16_t n) {
    assert(c==3 && op>=KB_FIND && op<=KB_WRITE && h);(void)a;(void)p;(void)n;requests++;return alloc_ok;
}
static void kb_ind_confirm(uint8_t c) { assert(c==3);confirms++; }
static void kb_master_dm(kb_hdr *h) { assert(h->param==3);dm_calls++; }
static int kb_master_security(kb_hdr *h) { assert(h->param==3);sec_calls++;return security_ok; }
static kb_state s;
static uint8_t control[80];
static uint16_t req;
static void ctl(uint8_t op,const uint8_t *p,unsigned n) {
    memset(control,0,sizeof control);control[0]='K';control[1]='B';control[2]=1;control[3]=op;
    kb_w16(control+4,++req);kb_w16(control+6,s.epoch);
    if(n)memcpy(control+8,p,n);
    in_wsf=1;kb_control(&s,control,8+n);kb_schedule(&s);in_wsf=0;
}
static kb_record *last(void) { assert(s.head!=s.tail);return &s.queue[(s.head-1)%KB_SLOTS]; }
static void drain(void) { __atomic_store_n(&s.tail,s.head,__ATOMIC_RELEASE); }
static void dm(uint8_t event) { kb_hdr h={3,event,0};assert(kb_event(&s,&h)); }
static void connect(void) {
    uint16_t epoch=s.epoch;uint8_t peer[7]={1,1,2,3,4,5,6};ctl(KB_ENABLE,0,0);ctl(KB_CONNECT,peer,7);
    assert(s.conn==3 && !s.connected && s.epoch==epoch+1);dm(0x27);assert(s.connected);dm(0x2c);assert(s.encrypted);drain();
}
static void isolation(void) {
    kb_hdr h={1,0x27,0};assert(!kb_event(0,&h));assert(!kb_event(&s,&h));
    for(unsigned e=0;e<256;e++) { h.event=e;assert(!kb_event(&s,&h) || e==0x20); }
    connect();
    unsigned before=dm_calls;
    for(unsigned id=0;id<5;id++) if(id!=3) {
        h.param=id;
        for(unsigned e=2;e<0x7c;e++) if(e!=0x20) {h.event=e;assert(!kb_event(&s,&h));}
    }
    assert(dm_calls==before);
    h.param=3;h.event=0x34;assert(!kb_event(&s,&h)); /* global ECC */
    h.event=0x79;assert(!kb_event(&s,&h));
    ctl(KB_DISABLE,0,0);assert(s.conn==3 && s.closing && !s.enabled);
    kb_att att={{3,13,0},(const uint8_t *)"abc",3,15,0,23};
    before=s.head;assert(kb_event(&s,&att.hdr));assert(s.head==before);
    dm(0x28);assert(!s.conn);h.event=0x27;assert(!kb_event(&s,&h));
}
static void commands(void) {
    memset(&s,0,sizeof s);scan_ok=0;ctl(KB_ENABLE,0,0);ctl(KB_SCAN,0,0);
    assert(kb_u16(last()->data+12)==KB_BUSY && !scans);
    scan_ok=1;ctl(KB_SCAN,0,0);assert(scans==1 && s.scanning);
    kb_hdr stop={0,0x25,0};assert(kb_event(&s,&stop));assert(!s.scanning);drain();
    uint8_t peer[7]={1};space=KB_NO_SPACE;ctl(KB_CONNECT,peer,7);assert(!s.conn);
    space=KB_OK;ctl(KB_CONNECT,peer,7);dm(0x27);
    uint8_t read[4]={12,0,0,0};ctl(KB_READ,read,4);assert(!requests && kb_u16(last()->data+12)==KB_NOT_ENCRYPTED);
    dm(0x2c);drain();
    ctl(KB_READ,read,4);assert(requests==1 && s.pending_event==5);
    ctl(KB_READ,read,4);assert(requests==1 && kb_u16(last()->data+12)==KB_BUSY);
    kb_att response={{3,5,0},(const uint8_t *)"map",3,12,0,23};
    assert(kb_event(&s,&response.hdr));assert(!s.pending_event && last()->data[3]==KB_ATT);
    /* Stale epochs cannot change an owned connection. */
    control[3]=KB_DISCONNECT;kb_w16(control+6,s.epoch+1);
    unsigned before=closes;kb_control(&s,control,8);assert(closes==before);
    assert(kb_u16(last()->data+12)==KB_STALE);
    ctl(KB_READ,read,4);now+=30001;kb_poll(&s);assert(s.closing && closes==before+1 && s.conn==3);
    assert(kb_event(&s,&response.hdr)); /* late callback swallowed after timeout */
    dm(0x28);assert(!s.conn);
}
static void reports(void) {
    memset(&s,0,sizeof s);connect();
    uint8_t watches[3]={1,15,0};ctl(KB_WATCH,watches,3);drain();
    uint8_t data[512];for(unsigned i=0;i<sizeof data;i++) data[i]=(uint8_t)i;
    kb_att a={{3,13,0},data,512,15,0,517};assert(kb_event(&s,&a.hdr));
    assert(s.head-s.tail==6);
    for(unsigned i=0;i<6;i++) {
        kb_record *r=&s.queue[(s.tail+i)%KB_SLOTS];
        assert(kb_u16(r->data+16)==512 && kb_u16(r->data+18)==i*96);
        assert(!memcmp(r->data+24,data+i*96,r->len-24));
    }
    drain();a.handle=16;assert(kb_event(&s,&a.hdr));assert(s.head==s.tail);
    a.hdr.event=14;assert(kb_event(&s,&a.hdr));assert(confirms==1);
    a.handle=15;a.hdr.event=13;a.len=513;assert(kb_event(&s,&a.hdr));assert(s.drops==1 && s.head==s.tail);
    a.len=1;
    for(unsigned i=0;i<KB_SLOTS;i++) assert(kb_event(&s,&a.hdr));
    assert(s.head-s.tail==KB_SLOTS);
    unsigned drops=s.drops,sequence=s.sequence;
    assert(kb_event(&s,&a.hdr));assert(s.drops==drops+1 && s.sequence==sequence+1);
    drain();now+=1001;kb_poll(&s);assert(last()->data[3]==KB_STATUS);
    assert(s.drops==s.reported_drops);
    /* No renewal lease: reports still relay hours after setup. */
    now+=3600000;drain();assert(kb_event(&s,&a.hdr));assert(last()->data[3]==KB_INPUT);
    /* Unsigned indices can wrap without overwriting unread entries. */
    s.head=s.tail=0xfffffff0u;
    for(unsigned i=0;i<32;i++) kb_emit(&s,KB_INPUT,0,13,15,data,1);
    assert(s.head-s.tail==32 && s.head==16);
}
static void prompts(void) {
    memset(&s,0,sizeof s);connect();
    uint8_t evt[20]={3,0,0x2e,0,0,1};assert(kb_event(&s,(kb_hdr *)evt));assert(s.auth);
    uint8_t pin[3]={0x40,0x42,0xf};ctl(KB_AUTH,pin,3);assert(s.auth && !auth_calls); /* 1,000,000 invalid */
    pin[0]--;ctl(KB_AUTH,pin,3);assert(!s.auth && auth_calls==1);
    evt[2]=0x35;evt[16]=0x12;evt[17]=0x34;evt[18]=0x56;evt[19]=0x78;
    assert(kb_event(&s,(kb_hdr *)evt));
    kb_record *r=&s.queue[(s.head-2)%KB_SLOTS];
    assert(r->data[3]==KB_DM && kb_u16(r->data+16)==4);
    uint32_t v=(uint32_t)kb_u16(r->data+24)|((uint32_t)kb_u16(r->data+26)<<16);
    assert(v==0x12345678u%1000000u);
    evt[2]=0x2f;assert(kb_event(&s,(kb_hdr *)evt));
    r=&s.queue[(s.head-2)%KB_SLOTS];assert(kb_u16(r->data+16)==0); /* keys never exported */
    security_ok=0;evt[2]=0x32;assert(kb_event(&s,(kb_hdr *)evt));assert(s.closing);
}
static void malformed(void) {
    uint32_t x=1;
    for(unsigned trial=0;trial<20000;trial++) {
        uint8_t p[81];for(unsigned i=0;i<sizeof p;i++){x=x*1664525u+1013904223u;p[i]=(uint8_t)(x>>24);}
        unsigned n=trial%82;
        if (trial&1) {p[0]='K';p[1]='B';p[2]=1;p[3]=(uint8_t)(trial%14);}
        if (n<=sizeof p && kb_valid_control(p,n)) { /* lengths cannot cause OOB reads */
            kb_state fresh={0};kb_control(&fresh,p,n);
        }
    }
    assert(!kb_valid_control(0,80));
}
static void fire_timer(void) {
    assert(timer_pending);timer_pending=0;kb_tick(&s);
}
static void process_poll(void) {
    assert(poll_queued);poll_queued--;in_wsf=1;kb_poll(&s);kb_schedule(&s);in_wsf=0;
}
static void timers(void) {
    memset(&s,0,sizeof s);timer_state=&s;
    timer_pending=timer_starts=phone_sends=poll_queued=0;
    ctl(KB_ENABLE,0,0);assert(timer_pending && !phone_sends);
    fire_timer();assert(phone_sends==1 && poll_queued==1 && timer_pending);
    process_poll();
    ctl(KB_DISABLE,0,0);fire_timer();process_poll();
    assert(!timer_pending && !s.timer_armed && !s.active);
    /* A producer arriving after drain but before parking cannot lose its wake. */
    ctl(KB_QUERY,0,0);inject_after_drain=1;fire_timer();
    assert(timer_pending && s.head!=s.tail);process_poll();fire_timer();process_poll();
    assert(!timer_pending && s.head==s.tail);
    /* Failed start remains retryable, rather than permanently marking armed. */
    fail_timer=1;ctl(KB_QUERY,0,0);assert(!s.timer_armed && !timer_pending);
    fail_timer=0;ctl(KB_QUERY,0,0);assert(timer_pending);fire_timer();process_poll();
    ctl(KB_ENABLE,0,0);fail_post=1;fire_timer();
    assert(!s.poll_pending && timer_pending);fail_post=0;fire_timer();process_poll();
    ctl(KB_DISABLE,0,0);fire_timer();process_poll();assert(!timer_pending);
}
int main(void) {
    isolation();commands();reports();prompts();malformed();timers();
    puts("PASS keyboard: isolation, commands, timeout ownership, encrypted reports, queue overflow/wrap, pairing prompts, malformed controls, timer drain/parking/retry");
}
