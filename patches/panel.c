#include "panel.h"

/* G2 2.3.0.24 only. ABIs recovered from the actual donor, including the PIO
 * word load at 0x4d9068: buffers must be word-aligned/padded, even for one byte.
 * Request templates and all callees are pinned in stock_abi_230.json. */
#ifndef PANEL_READ
#define PANEL_READ ((int (*)(uint32_t, void *, uint32_t))0x005b20fbu)
#define PANEL_READ_ADDRESS ((int (*)(uint32_t, void *, uint32_t, uint32_t))0x005b212fu)
#define PANEL_WRITE ((int (*)(uint32_t, const void *, uint32_t))0x005b208bu)
#define PANEL_REFRESH ((int (*)(uint32_t,uint32_t,uint32_t,uint32_t,uint32_t,uint32_t))0x005b279fu)
#define PANEL_RECOVER ((void (*)(void))0x005b2f95u)
#define PANEL_DELAY ((void (*)(uint32_t))0x00442b2bu)
#define PANEL_BACKEND (*(volatile uint32_t *)0x20077868u == 0x0072bd7cu && \
                       *(volatile uint32_t *)0x2007785cu != 0)
#endif

static uint32_t panel_u32(const uint8_t *b) {
    return (uint32_t)b[0] | (uint32_t)b[1]<<8 | (uint32_t)b[2]<<16 | (uint32_t)b[3]<<24;
}
static void panel_put32(uint8_t *b, uint32_t v) {
    for (unsigned i=0;i<4;i++) b[i]=(uint8_t)(v>>(8*i));
}
static int panel_read(uint8_t command, uint8_t *out, uint8_t length, uint32_t address) {
    uint32_t words[8] = {0};
    int r = command == 0x81 ? PANEL_READ_ADDRESS(command, words, length, address << 8)
                            : PANEL_READ(command, words, length);
    if (!r) memcpy(out, words, length);
    return r;
}
static int panel_write(uint8_t command, const uint8_t *data, uint8_t length) {
    uint32_t words[2] = {0};
    if (length) memcpy(words, data, length);
    return PANEL_WRITE(command, words, length);
}
static int panel_register(uint8_t command, const uint8_t *data, uint8_t length) {
    int r = panel_write(6, 0, 0);
    if (!r) r = panel_write(command, data, length);
    return r;
}
static int panel_redraw(void) {
    if (!FW_DISPLAY_FB) return -1;
    uint32_t desc[2] = {(uint32_t)(uintptr_t)FW_DISPLAY_FB, PANEL_BYTES};
    FW_FLUSH(desc);
    return PANEL_REFRESH(0, 0, 0, 0, PANEL_W, PANEL_H);
}
static int panel_capture(cfw_panel_state *p) {
    if (p->saved) return 0;
    uint8_t b[7]={0};
    if (panel_read(5,b,1,0) || panel_read(0x35,b+1,1,0) ||
        panel_read(0x37,b+2,2,0) || panel_read(0x47,b+4,1,0)) return -1;
    memcpy(p->baseline,b,7);
    p->saved=1;
    return 0;
}
static int panel_restore(cfw_panel_state *p) {
    if (!p->saved) return 0;
    uint8_t sr1=p->baseline[0]&0x78, sr2=p->baseline[1]&7;
    int r=panel_register(1,&sr1,1);
    if (p->saved_sr3) r |= panel_register(0x57,p->baseline+5,1);
    r |= panel_register(0x31,&sr2,1);
    r |= panel_register(0x46,p->baseline+4,1);
    /* Restore luminance LAST: status writes reset it. Bytes are wire order. */
    r |= panel_register(0x36,p->baseline+2,2);
    r |= panel_write(0xa3,0,0);
    r |= panel_write(0x97,0,0);
    PANEL_DELAY(2);
    if (!r) { p->saved=p->saved_sr3=0; p->register_deadline=0; }
    return r;
}
static int panel_valid(const uint8_t *s, uint32_t n) {
    if (n!=12 || !(s[1]|s[2]) || !s[3] || (s[3]&~3u)) return 0;
    uint8_t op=s[4], len=s[5]; uint32_t a=panel_u32(s+6);
    unsigned v=s[10]|s[11]<<8;
    if (s[0]==23) {
        if (v) return 0;
        if (op==0) return !len && !a;
        if (op==0xfe) return len && len<=32 && a<=PANEL_BYTES-len;
        if (op==0x81) return len==12 && a<=0x1fff;
        if (a) return 0;
        return ((op==5 || op==0x35 || op==0x59 || op==0x47) && len==1) ||
               ((op==0x37 || op==0xc1) && len==2) || (op==0x9f && len==4);
    }
    if (s[0]==25) return !len && !a && (op<=18) && (op==18?v<=15:!v);
    if (s[0]!=24 || len || a) return 0;
    if (op==0) return v<=1;
    if (op==1) return !(v&~0x78u);
    if (op==0x31) return v<=7;
    if (op==0x46) return v<=63;
    if (op==0x57) return v<=255;
    if (op==0x36) return 1; /* refresh-dependent bound checked on display task */
    return !v && (op==0x97 || op==0xfe || op==0xfd);
}
static void panel_watchdog(void *unused) {
    (void)unused;
    /* Only schedule: SPI is never touched from the timer thread. */
    FW_DISPLAY_QUEUE(0,0,0,0,PANEL_W,PANEL_H);
}
/* The image mutex is held; the display gate is released by the stock task.
 * Only copy request bytes here: no SPI calls from BLE/bridge task contexts. */
static int panel_queue(const uint8_t *src, uint32_t size, uint8_t origin) {
    if (!panel_valid(src,size)) return -1;
    uint8_t here=FW_SIDE()==1 ? 2 : 1;
    if (!(src[3]&here)) return 0;
    customCfwContext *ctx=getCustomCfwContext();
    if (!ctx) return -1;
    FW_DISPLAY_WAIT();
    if (ctx->direct_pending || ctx->panel.pending) return -1;
    if (src[0]!=23) {
        if (!ctx->panel.watchdog) ctx->panel.watchdog=FW_TIMER_NEW((void *)&panel_watchdog,1,0,0);
        if (!ctx->panel.watchdog || FW_TIMER_START(ctx->panel.watchdog,1000)!=0) {
            FW_DISPLAY_SIGNAL(); return -1;
        }
    }
    memcpy(ctx->panel.request,src,12);
    ctx->panel.origin=origin;
    __atomic_store_n(&ctx->panel.pending,1,__ATOMIC_RELEASE);
    if (FW_DISPLAY_QUEUE(0,0,0,0,PANEL_W,PANEL_H)!=0) {
        ctx->panel.pending=0; FW_DISPLAY_SIGNAL(); return -1;
    }
    return 0;
}
static void panel_snapshot(cfw_panel_state *p) {
    uint8_t *b=p->result+8; unsigned valid=0;
    static const uint8_t commands[]={0x9f,5,0x35,0x59,0x37,0x47,0xc1,0x81};
    static const uint8_t lengths[]={4,1,1,1,2,1,2,12};
    unsigned offset=2;
    for(unsigned i=0;i<8;i++) {
        if (!panel_read(commands[i],b+offset,lengths[i],i==7?0x1fff:0)) valid|=1u<<i;
        offset+=lengths[i];
    }
    b[0]=(uint8_t)valid; b[1]=(uint8_t)(valid>>8);
    uint16_t crc=0xffff;
    if (FW_DISPLAY_FB) for (unsigned i=0;i<PANEL_BYTES;i++) {
        crc^=(uint16_t)FW_DISPLAY_FB[i]<<8;
        for(unsigned j=0;j<8;j++) crc=(uint16_t)((crc<<1)^((crc&0x8000)?0x1021:0));
    }
    b[offset++]=(uint8_t)crc; b[offset++]=(uint8_t)(crc>>8);
    b[offset++]=p->pattern; b[offset++]=p->saved;
    p->result_length=(uint8_t)(8+offset);
    if (valid!=255) p->result[1]=4; /* partial snapshot, inspect validity mask */
}
static int panel_poke(cfw_panel_state *p) {
    uint8_t *s=p->request,op=s[4]; unsigned value=s[10]|s[11]<<8;
    if (op==0xfd) { PANEL_RECOVER(); p->saved=p->saved_sr3=0; p->register_deadline=0; return 0; }
    if (op==0xfe) return panel_restore(p);
    if (op==0x97) { int r=panel_write(0x97,0,0); PANEL_DELAY(2); return r; }
    uint8_t id[4];
    if (panel_read(0x9f,id,4,0) || id[0]!=0xbd || id[1]!=0x40 || id[2]!=0x10) return -2;
    if (panel_capture(p)) return -1;
    p->register_deadline=FW_MS_TICK+15000;
    uint8_t lum[2],cur,sr1;
    if (panel_read(0x37,lum,2,0) || panel_read(0x47,&cur,1,0) || panel_read(5,&sr1,1,0)) return -1;
    if (op==0) { op=1; value=(sr1&0x38)|(value?0x40:0); }
    static const uint16_t maxima[]={21331,10664,7109,5331,4264,3366,2907,2558};
    if (op==0x36 && value>maxima[(sr1>>3)&7]) return -2;
    /* An explicit RFFQ change must still accommodate the preserved luminance. */
    if (op==1 && ((sr1^value)&0x38) &&
        ((unsigned)lum[0]<<8 | lum[1])>maxima[(value>>3)&7]) return -2;
    if (op==0x57 && !p->saved_sr3) {
        if (panel_read(0x59,p->baseline+5,1,0)) return -1;
        p->saved_sr3=1;
    }
    uint8_t data[2]={(uint8_t)value,0}; unsigned length=1;
    if (op==0x36) { data[0]=(uint8_t)(value>>8); data[1]=(uint8_t)value; length=2; }
    int r=panel_register(op,data,length);
    if (op==1 || op==0x57 || op==0x31) {
        r |= panel_register(0x46,&cur,1);
        r |= panel_register(0x36,lum,2);
    }
    r |= panel_write(0xa3,0,0); r |= panel_write(0x97,0,0); PANEL_DELAY(2);
    return r;
}
static void panel_pattern(cfw_panel_state *p) {
    uint8_t *fb=FW_DISPLAY_FB;
    for (unsigned y=0;y<PANEL_H;y++) for(unsigned x=0;x<PANEL_W;x+=2) {
        unsigned c=p->pattern-1;
        if (p->pattern==17) c=x/40;
        if (p->pattern==18) c=(x>=240 && x<400 && y>=160 && y<320)?p->request[10]:0;
        fb[y*320+x/2]=(uint8_t)(c*17);
    }
}
/* Runs only at the existing display-copy hook: previous DMA has completed.
 * The periodic stock display maintenance also checks diagnostic expiry here. */
static void panel_service(void) {
    customCfwContext *ctx=peekCustomCfwContext(); if (!ctx) return;
    cfw_panel_state *p=&ctx->panel;
    if (p->saved && PANEL_BACKEND &&
        ((int32_t)(p->register_deadline-FW_MS_TICK)<=0 || !cfw_fb_lease_active())) {
        if (panel_restore(p)) { PANEL_RECOVER(); p->saved=p->saved_sr3=0; }
    }
    if (p->pattern && ((int32_t)(p->pattern_deadline-FW_MS_TICK)<=0 || !cfw_fb_lease_active())) {
        p->pattern=0; ctx->direct_active=0;
    }
    if (!__atomic_load_n(&p->pending,__ATOMIC_ACQUIRE)) {
        if (!p->saved && !p->pattern && p->watchdog) FW_TIMER_STOP(p->watchdog);
        return;
    }
    uint8_t *s=p->request; uint16_t id=s[1]|s[2]<<8;
    int same=p->result_length && p->cached_origin==p->origin;
    for(unsigned i=0;i<12;i++) if(s[i]!=p->cached_request[i]) same=0;
    if (!same) {
        for(unsigned i=0;i<CFW_PANEL_REPLY_MAX;i++) p->result[i]=0;
        p->result[0]=1; p->result[2]=s[0]; p->result[3]=s[4];
        panel_put32(p->result+4,FW_MS_TICK); p->result_length=8;
        if (!PANEL_BACKEND || !FW_DISPLAY_FB) p->result[1]=2;
        else if(s[0]==23) {
            if (!s[4]) panel_snapshot(p);
            else {
                unsigned len=s[5];
                if(s[4]==0xfe) memcpy(p->result+8,FW_DISPLAY_FB+panel_u32(s+6),len);
                else if(panel_read(s[4],p->result+8,len,panel_u32(s+6))) p->result[1]=3;
                p->result_length=(uint8_t)(8+len);
            }
        } else if(s[0]==24) {
            int r=panel_poke(p);
            if(r) p->result[1]=r==-2?2:3;
            if(!r && panel_redraw()) p->result[1]=3;
        } else {
            p->pattern=s[4]; p->pattern_deadline=FW_MS_TICK+60000;
            if(p->pattern) { panel_pattern(p); if(panel_redraw()) p->result[1]=3; }
            else ctx->direct_active=0;
        }
        memcpy(p->cached_request,s,12); p->cached_origin=p->origin;
    }
    panel_emit(p->origin,id,p->result,p->result_length);
    __atomic_store_n(&p->pending,0,__ATOMIC_RELEASE);
}
