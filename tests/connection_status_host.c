#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
static uint8_t ram[0x800000];
static uint32_t state_memory[3];
static uint8_t *readptr(uint32_t a) {
    assert(a>=0x20000000u && a<0x20800000u);
    return ram+(a-0x20000000u);
}
#define CS_HOST_TEST
#define CS_READ8(a) (*readptr(a))
#define CS_READ32(a) (*(uint32_t *)(void *)readptr(a))
#define CS_STATE ((volatile cs_state *)(void *)state_memory)
static int qresult, lockresult, locks, unlocks, flushes, custom_frame, hide_on_lock;
static uint32_t passed_timeout;
static int receive(uint32_t q,void *m,uint8_t *p,uint32_t t) {
    assert(q==123); (void)p; passed_timeout=t;
    if(!qresult) ((uint32_t *)m)[0]=0xabcdef;
    return qresult;
}
static int lock(uint32_t h) {
    assert(h==456); locks++;
    if(hide_on_lock) CS_READ8(0x20077298)=0;
    return lockresult;
}
#define CS_QUEUE_GET(q,m,p,t) receive(q,m,p,t)
#define CS_TRY_LOCK(h) lock(h)
#define CS_UNLOCK() (unlocks++)
#define CS_FLUSH(p,n) do {assert((p)!=NULL && (n)==153600);flushes++;}while(0)
#define CS_CUSTOM_FRAME_ACTIVE() custom_frame
static unsigned masked;
static uint32_t cs_enter(void) { assert(!masked); masked=1; return 0x42; }
static void cs_leave(uint32_t k) { assert(masked && k==0x42);masked=0; }
#include "../patches/draw.c"
#include "../patches/connection_status.c"
static void reset(void) {
    memset(ram,0,sizeof ram);memset(state_memory,0,sizeof state_memory);
    qresult=-2;lockresult=1;locks=unlocks=flushes=custom_frame=hide_on_lock=0;
    CS_READ8(0x20077298)=1;CS_READ32(0x20076774)=1;
    CS_READ32(0x20076770)=1;CS_READ32(0x2007698c)=456;
    CS_READ32(0x20076554)=0x20070000;CS_READ8(0x200773e3)=2;
}
static void phone(unsigned id) {
    CS_READ8(0x20070054)=id;
    if(id<1 || id>3)return;
    uint32_t rec=0x20072d68+(id-1)*0x30;
    CS_READ8(rec+0x16)=1;CS_READ8(rec+0x19)=1;
    for(unsigned i=0;i<6;i++)CS_READ8(rec+i)=(uint8_t)(0xa0+i);
}
static void snapshots(void) {
    reset();cs_snapshot s=cs_read_link();assert(!s.linked);
    for(unsigned id=1;id<=3;id++) {
        phone(id);s=cs_read_link();assert(s.linked && s.address_valid);
        char t[48];cs_text(t,&s);assert(!strcmp(t,"Connected from: A5:A4:A3:A2:A1:A0"));
    }
    for(unsigned id=4;id<256;id++){phone(id);assert(!cs_read_link().linked);}
    phone(1);CS_READ8(0x20072d81)=0;assert(!cs_read_link().linked); /* ring */
    CS_READ8(0x20072d81)=1;
    for(unsigned i=0;i<6;i++)CS_READ8(0x20072d68+i)=0;
    s=cs_read_link();assert(s.linked && !s.address_valid);
    char text[48];cs_text(text,&s);assert(!strcmp(text,"Connected from: address unavailable"));
    for(unsigned i=0;i<6;i++)CS_READ8(0x20072d68+i)=255;
    assert(!cs_read_link().address_valid);
    CS_READ8(0x20072d7e)=0;assert(cs_read_link().linked); /* staged link, no addr */
    uint32_t invalid[]={0,0x20000001,0x20800000,0xffffffff};
    for(unsigned i=0;i<4;i++){CS_READ32(0x20076554)=invalid[i];assert(!cs_read_link().linked);}
}
static void routing(void) {
    reset();assert(cs_dashboard_visible());
    CS_READ32(0x20076778)=7;assert(!cs_dashboard_visible());
    CS_READ32(0x20076774)=2;assert(cs_dashboard_visible());
    CS_READ32(0x20076770)=7;assert(!cs_dashboard_visible());
    CS_READ32(0x20076774)=1;CS_READ32(0x20076778)=1;
    assert(cs_dashboard_visible()); /* dashboard is the foreground app */
    reset();custom_frame=1;assert(!cs_dashboard_visible());custom_frame=0;
    CS_READ8(0x20077298)=0;assert(!cs_dashboard_visible());
}
static void queue_contract(void) {
    uint32_t message[11]; uint8_t priority=9;
    reset();phone(1);memset(message,0x55,sizeof message);
    assert(connection_status_queue_get(123,message,&priority,0xffffffff)==0);
    assert(passed_timeout==500 && locks==1 && !unlocks && priority==0);
    assert(message[0]==3 && message[5]==640 && message[6]==480);
    assert(message[7]==0 && message[8]==0 && message[9]==0x55555555);
    CS_STATE->rendered=cs_read_link();CS_STATE->magic=CS_MAGIC;
    assert(connection_status_queue_get(123,message,NULL,0xffffffff)==-2 && locks==1);
    phone(0);lockresult=0;
    assert(connection_status_queue_get(123,message,NULL,0xffffffff)==-2);
    lockresult=1;assert(connection_status_queue_get(123,message,NULL,0xffffffff)==0);
    qresult=0;assert(connection_status_queue_get(123,message,NULL,0xffffffff)==0);
    assert(message[0]==0xabcdef); /* real messages are never overwritten */
    qresult=-3;assert(connection_status_queue_get(123,message,NULL,0xffffffff)==-3);
    qresult=-2;CS_READ8(0x20077298)=0;
    assert(connection_status_queue_get(123,message,NULL,0xffffffff)==-2);
    assert(passed_timeout==0xffffffff);
    CS_READ8(0x20077298)=1;
    assert(connection_status_queue_get(123,message,NULL,17)==-2 && passed_timeout==17);
    reset();phone(1);hide_on_lock=1;
    assert(connection_status_queue_get(123,message,NULL,0xffffffff)==-2);
    assert(unlocks==1); /* never leak a lock when visibility changes */
}
static uint8_t framebuffer[153600+64];
static void render(const char *path) {
    reset();phone(1);memset(framebuffer,0x55,sizeof framebuffer);
    connection_status_overlay(framebuffer+32);
    assert(flushes==1 && !cs_changed(&(cs_snapshot){1,1,{0xa0,0xa1,0xa2,0xa3,0xa4,0xa5}}));
    for(unsigned i=0;i<32;i++)assert(framebuffer[i]==0x55 && framebuffer[153632+i]==0x55);
    /* Raster must stay below row 236 and within the 576-pixel dashboard. */
    for(unsigned i=0;i<236*320;i++)assert(framebuffer[32+i]==0x55);
    for(unsigned y=236;y<480;y++)for(unsigned x=576/2;x<320;x++)assert(framebuffer[32+y*320+x]==0x55);
    /* A real repaint starts with stock pixels, including a disconnect repaint. */
    memset(framebuffer+32,0x55,153600);phone(0);connection_status_overlay(framebuffer+32);
    assert(CS_STATE->rendered.linked==0);
    /* maximum lens offsets and longer fallback string: ASan catches overflow */
    phone(1);CS_READ8(0x20072d7e)=0;
    CS_READ32(0x20000744)=999;CS_READ32(0x20000748)=999;
    CS_READ8(0x200773e3)=1;connection_status_overlay(framebuffer+32);
    /* Export a synthetic full dashboard-space preview from actual C raster. */
    reset();phone(1);memset(framebuffer+32,0,153600);
    CS_READ32(0x20000744)=32;CS_READ32(0x20000748)=96;
    connection_status_overlay(framebuffer+32);
    if(path){FILE *f=fopen(path,"wb");assert(f);fprintf(f,"P5\n640 480\n255\n");
        for(unsigned i=0;i<153600;i++){uint8_t v=framebuffer[32+i];fputc((v>>4)*17,f);fputc((v&15)*17,f);}fclose(f);}
    memset(framebuffer+32,0x55,153600);CS_READ8(0x20077298)=0;
    connection_status_overlay(framebuffer+32);assert(!CS_STATE->magic);
    for(unsigned i=0;i<153600;i++)assert(framebuffer[32+i]==0x55);
}
int main(int argc,char **argv) {
    snapshots();routing();queue_contract();render(argc>1?argv[1]:NULL);
    puts("connection-status host contracts: PASS");return 0;
}
