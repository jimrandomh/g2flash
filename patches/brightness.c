#include "brightness.h"

/* Donor 2.3.0.24: use the complete calibrated stock brightness sequence,
 * including display enable and SYNC. Revision 31's register-only writer could
 * leave the panel dark: matching brightness registers do not establish that
 * scan/display is enabled. Keep the known-working revision-30 hardware path.
 * This bypasses the persistent setting and ALS learning, and runs after DMA. */
#ifndef BRIGHTNESS_CONVERT
#define BRIGHTNESS_CONVERT ((void (*)(uint32_t,uint32_t *,uint32_t *))0x00470461u)
#define BRIGHTNESS_APPLY ((void (*)(uint32_t,uint32_t,uint32_t))0x004e3d9bu)
#define BRIGHTNESS_STOCK_LEVEL (*(volatile uint8_t *)0x20075e09u)
#define BRIGHTNESS_DISPLAY_ON (*(volatile uint32_t *)0x20077818u)
#endif

#define BRIGHTNESS_RELEASE 0x80000000u

/* Smoothstep, evaluated from elapsed time rather than callback count. */
static uint8_t brightness_interpolate(uint8_t from, uint8_t to, uint32_t elapsed, uint32_t duration) {
    if (!duration || elapsed >= duration) return to;
    uint32_t t=elapsed*1024u/duration;
    uint32_t gain=t*t*(3072u-2u*t)/(1024u*1024u);
    int32_t delta=((int32_t)to-from)*(int32_t)gain;
    int32_t value=(int32_t)from+(delta+(delta<0?-512:512))/1024;
    return (uint8_t)value;
}

static void brightness_apply(uint8_t level) {
    uint32_t current=0, luminance=0;
    BRIGHTNESS_CONVERT(level,&current,&luminance);
    BRIGHTNESS_APPLY(level,luminance,current);
}

static void brightness_tick(void *unused) {
    (void)unused;
    /* Match panel_watchdog: only the display task may touch the panel. */
    FW_DISPLAY_QUEUE(0,0,0,0,PANEL_W,PANEL_H);
}

/* [30][version=1][level:2..100][visible:0|1][duration:u16 <=3000].
 * Ordered/coalescible and idempotent, like the frame it accompanies. */
__attribute__((noinline)) static int brightness_control(const uint8_t *src, uint32_t size) {
    if (size!=6 || src[1]!=1 || src[2]<2 || src[2]>100 || src[3]>1) return -1;
    uint32_t duration=(uint32_t)src[4]|(uint32_t)src[5]<<8;
    if (duration>3000 || !cfw_fb_lease_active()) return -1;
    customCfwContext *ctx=getCustomCfwContext();
    if (!ctx) return -1;
    cfw_brightness_state *b=&ctx->brightness;
    if (!b->timer) b->timer=FW_TIMER_NEW((void *)&brightness_tick,1,0,0);
    if (!b->timer || FW_TIMER_START(b->timer,40)!=0) return -1;
    uint32_t request=src[2]|(uint32_t)src[3]<<8|duration<<9|
        (__atomic_load_n(&b->present_epoch,__ATOMIC_ACQUIRE)&1023u)<<21;
    __atomic_store_n(&b->request,request,__ATOMIC_RELEASE);
    return 0;
}

/* Return true when the physical frame must remain frozen (fading out), or
 * black (asleep). Updates still stage in screen_buffer for the next wake. */
static int brightness_service(void) {
    customCfwContext *ctx=peekCustomCfwContext();
    if (!ctx) return 0;
    cfw_brightness_state *b=&ctx->brightness;
    uint32_t request=__atomic_exchange_n(&b->request,0,__ATOMIC_ACQ_REL);
    uint32_t now=FW_MS_TICK;
    if (request==BRIGHTNESS_RELEASE || !cfw_fb_lease_active()) {
        if (b->active && BRIGHTNESS_DISPLAY_ON) brightness_apply(BRIGHTNESS_STOCK_LEVEL);
        b->active=b->waiting=0;
        if (b->timer) FW_TIMER_STOP(b->timer);
        return 0;
    }
    if (request) {
        uint8_t visible=(request>>8)&1, desired=request&255;
        uint16_t duration=(request>>9)&4095;
        if (!b->active) {
            b->active=1; b->from=2; b->level=2; b->visible=0; b->target=2; b->last_apply=0;
        }
        if (visible!=b->visible || (visible && desired!=b->desired)) {
            int wake=visible && !b->visible;
            b->from=b->level;
            b->target=visible?desired:2;
            b->duration=duration; b->started=now;
            if (wake) { b->waiting=1; b->wait_epoch=(request>>21)&1023; }
        }
        if (!visible) b->waiting=0;
        b->visible=visible; b->desired=desired;
    }
    if (!b->active) return 0;
    if (b->waiting && ctx->direct_pending &&
        (__atomic_load_n(&b->present_epoch,__ATOMIC_ACQUIRE)&1023u)!=b->wait_epoch) {
        b->waiting=0; b->started=now;
    }
    uint8_t level=b->waiting?b->level:brightness_interpolate(b->from,b->target,now-b->started,b->duration);
    /* Reassert after stock panel recovery as well as at each changed step. */
    if (BRIGHTNESS_DISPLAY_ON && (level!=b->level || !b->last_apply || now-b->last_apply>=1000)) {
        brightness_apply(level); b->last_apply=now?now:1;
    }
    b->level=level;
    /* Keep idle ownership cheap: only active transitions need 25 Hz. */
    FW_TIMER_START(b->timer,(!b->waiting && level!=b->target)?40:1000);
    if (b->waiting) return 1;
    if (!b->visible) {
        if (level==2 && FW_DISPLAY_FB) {
            bzero(FW_DISPLAY_FB,PANEL_BYTES);
            uint32_t desc[2]={(uint32_t)(uintptr_t)FW_DISPLAY_FB,PANEL_BYTES};
            FW_FLUSH(desc);
        }
        return 1;
    }
    return 0;
}

static void brightness_cleanup(void) {
    customCfwContext *ctx=peekCustomCfwContext();
    if (!ctx || !ctx->brightness.timer) return;
    __atomic_store_n(&ctx->brightness.request,BRIGHTNESS_RELEASE,__ATOMIC_RELEASE);
    FW_DISPLAY_QUEUE(0,0,0,0,PANEL_W,PANEL_H);
}
