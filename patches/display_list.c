/* Revision 25. All calls inherit a target; a call-local override is scoped. */
#include "resource_cache.h"
static int decode_image_rle(const uint8_t *,uint32_t,uint8_t *,uint32_t,uint32_t,uint32_t);
#define CFW_DRAW_MAX_OPS 4096u
#define CFW_DRAW_MAX_DEPTH 8u
#define CFW_DRAW_SCREEN 65535u
#define CFW_DRAW_CURRENT 65534u

typedef struct { uint8_t *pixels; uint32_t width, height, stride; } cfw_draw_target;
typedef struct { uint32_t remaining, depth; uint16_t ancestors[CFW_DRAW_MAX_DEPTH]; uint8_t *references; } cfw_draw_walk;

static int cfw_draw_resource_target(customCfwContext *ctx, uint32_t id, cfw_draw_target *out) {
    uint32_t size;
    const uint8_t *data = cfw_resource_get(ctx, id, &size);
    cfw_cached_image image;
    if (!data || (data[0] & 11u) || !cfw_texture_image_at(data, size, 0, &image)) return -1;
    out->pixels = (uint8_t *)image.rle; out->width = image.width; out->height = image.height;
    out->stride = (image.width + 1u) >> 1;
    return 0;
}
static void cfw_draw_ref(cfw_draw_walk *walk, uint32_t id) {
    if (walk->references && id < CFW_RESOURCE_COUNT) walk->references[id >> 3] |= 1u << (id & 7u);
}
static uint8_t cfw_draw_pixel(cfw_draw_target *t, uint32_t x, uint32_t y) {
    return (t->pixels[y * t->stride + (x >> 1)] >> ((x & 1u) ? 0 : 4)) & 15u;
}
static void cfw_draw_put(cfw_draw_target *t, int32_t x, int32_t y, uint8_t value) {
    if (x < 0 || y < 0 || (uint32_t)x >= t->width || (uint32_t)y >= t->height) return;
    uint8_t *p = t->pixels + (uint32_t)y * t->stride + ((uint32_t)x >> 1);
    if (x & 1) *p = (*p & 0xf0u) | value;
    else *p = (*p & 15u) | (value << 4);
}
static int cfw_draw_sequence(customCfwContext *, const uint8_t *, uint32_t, cfw_draw_target, int, cfw_draw_walk *);
static int cfw_draw_list(customCfwContext *ctx, uint32_t id, cfw_draw_target target, int apply, cfw_draw_walk *walk) {
    if (walk->depth >= CFW_DRAW_MAX_DEPTH) return -1;
    for (uint32_t i = 0; i < walk->depth; i++) if (walk->ancestors[i] == id) return -1;
    uint32_t size;
    const uint8_t *data = cfw_resource_get(ctx, id, &size);
    if (!data || size < 3 || data[0] != 2) return -1;
    cfw_draw_ref(walk, id);
    walk->ancestors[walk->depth++] = id;
    int result = cfw_draw_sequence(ctx, data + 1, size - 1, target, apply, walk);
    walk->depth--;
    return result;
}
static int cfw_draw_call(customCfwContext *ctx, const uint8_t *p, uint32_t n,
                          cfw_draw_target target, int apply, cfw_draw_walk *walk) {
    if (n < 2 || !walk->remaining--) return -1;
    uint32_t op = p[0], flags = p[1]; p += 2; n -= 2;
    if (flags & ~1u) return -1;
    if (flags & 1u) {
        if (n < 2) return -1;
        uint32_t id = rd16(p); p += 2; n -= 2;
        if (cfw_draw_resource_target(ctx, id, &target)) return -1;
        cfw_draw_ref(walk, id);
    }
    if (op == 1) { /* bbox: flags, compact or u16 xywh, pixel RLE (no row padding). */
        if (!n || p[0] > 1) return -1;
        uint32_t wide = p[0], header = wide ? 9u : 5u;
        if (n < header) return -1;
        uint32_t x = wide ? rd16(p+1) : p[1]*4u, y = wide ? rd16(p+3) : p[2]*2u;
        uint32_t w = wide ? rd16(p+5) : p[3]*4u, h = wide ? rd16(p+7) : p[4]*2u;
        if (!w || !h || x > target.width || w > target.width-x || y > target.height || h > target.height-y) return -1;
        uint32_t pos = header, pixel = 0, total = w*h;
        while (pos < n && pixel < total) {
            uint32_t used, count; uint8_t color;
            if (!cfw_texture_rle_token(p+pos, n-pos, &used, &count, &color) || count > total-pixel) return -1;
            if (apply && wide) for (uint32_t i=0; i<count; i++) cfw_draw_put(&target, x+(pixel+i)%w, y+(pixel+i)/w, color);
            pixel += count; pos += used;
        }
        if(pos!=n || pixel!=total) return -1;
        if(apply && !wide && !decode_image_rle(p+header,n-header,target.pixels+y*target.stride+(x>>1),target.stride,w>>1,h)) return -1;
        return 0;
    }
    if (op == 2) { /* source id, source xywh, destination xy. */
        if (n != 14) return -1;
        cfw_draw_target source = target;
        uint32_t id=rd16(p);
        if (id == CFW_DRAW_SCREEN) { source.pixels=ctx->screen_buffer; source.width=640; source.height=480; source.stride=320; }
        else if (id != CFW_DRAW_CURRENT) { if (cfw_draw_resource_target(ctx,id,&source)) return -1; cfw_draw_ref(walk,id); }
        uint32_t x=rd16(p+2),y=rd16(p+4),w=rd16(p+6),h=rd16(p+8);
        int32_t dx=(int16_t)rd16(p+10),dy=(int16_t)rd16(p+12);
        if (!source.pixels || !w || !h || x>source.width || w>source.width-x || y>source.height || h>source.height-y) return -1;
        if (apply) {
            /* memmove semantics also defined for same-buffer scrolls. */
            int reverse = source.pixels == target.pixels && (dy>(int32_t)y || (dy==(int32_t)y && dx>(int32_t)x));
            for (uint32_t i=0;i<w*h;i++) { uint32_t j=reverse?w*h-1-i:i;
                cfw_draw_put(&target,dx+(int32_t)(j%w),dy+(int32_t)(j/w),cfw_draw_pixel(&source,x+j%w,y+j/w)); }
        }
        return 0;
    }
    if (op == 3) {
        if (n<6 || n!=6u+p[5]) return -1;
        uint32_t pos=6;
        while(pos<n) { uint32_t used,cp; if(p[pos]>=1 && p[pos]<=31) { pos++; continue; }
            if(!cfw_texture_utf8(p+pos,n-pos,&used,&cp)) return -1; pos+=used; }
        if (!apply) return 0;
        cfw_rectlist rl; rl.n=0;
        return cfw_builtin_draw_string(target.pixels,target.stride,target.width,target.height,p,n,&rl);
    }
    if (op == 4 || op == 5) {
        if ((op==4 && n!=7) || (op==5 && (n<8 || n!=8u+p[7]))) return -1;
        uint32_t id=rd16(p),size; const uint8_t *data=cfw_resource_get(ctx,id,&size);
        if (!data) return -1;
        cfw_draw_ref(walk,id);
        cfw_cached_image image;
        if(op==4) { if(!cfw_texture_image_at(data,size,0,&image)) return -1; }
        else {
            if(size<193 || data[0]!=1) return -1;
            for(uint32_t i=8;i<n;i++) { uint32_t ch=p[i]; if(ch>=1 && ch<=31) continue;
                if(ch<32 || ch>127) return -1; uint32_t offset=rd16(data+1+(ch-32)*2);
                if(offset<193 || !cfw_texture_image_at(data,size,offset,&image)) return -1; }
        }
        if(!apply) return 0;
        cfw_rectlist rl; rl.n=0;
        return op==4 ? cfw_texture_draw_image(target.pixels,target.stride,target.width,target.height,p,n,&rl)
                     : cfw_texture_draw_string(target.pixels,target.stride,target.width,target.height,p,n,&rl);
    }
    if (op == 6) { /* xywh + sixteen packed LUT nibbles. */
        if(n!=16) return -1;
        uint32_t x=rd16(p),y=rd16(p+2),w=rd16(p+4),h=rd16(p+6);
        if(!w || !h || x>target.width || w>target.width-x || y>target.height || h>target.height-y) return -1;
        if(apply) for(uint32_t yy=y;yy<y+h;yy++) for(uint32_t xx=x;xx<x+w;xx++) {
            uint8_t v=cfw_draw_pixel(&target,xx,yy),lut=p[8+(v>>1)];
            cfw_draw_put(&target,xx,yy,(lut>>((v&1)?0:4))&15); }
        return 0;
    }
    if (op == 7 && n==2) return cfw_draw_list(ctx,rd16(p),target,apply,walk);
    return -1;
}
static int cfw_draw_sequence(customCfwContext *ctx,const uint8_t *p,uint32_t n,cfw_draw_target target,int apply,cfw_draw_walk *walk) {
    if(n<2) return -1;
    uint32_t count=rd16(p),pos=2;
    for(uint32_t i=0;i<count;i++) {
        if(n-pos<2) return -1; uint32_t length=rd16(p+pos);pos+=2;
        if(length>n-pos || cfw_draw_call(ctx,p+pos,length,target,apply,walk)) return -1;
        pos+=length;
    }
    return pos==n?0:-1;
}
static int cfw_draw_run(customCfwContext *ctx,const uint8_t *p,uint32_t n,cfw_draw_target target,int apply,uint8_t *refs) {
    cfw_draw_walk walk; walk.remaining=CFW_DRAW_MAX_OPS;walk.depth=0;walk.references=refs;
    return cfw_draw_sequence(ctx,p,n,target,apply,&walk);
}
static int cfw_draw_root(customCfwContext *ctx,uint32_t id,cfw_draw_target target,int apply,uint8_t *refs) {
    if(id==CFW_DRAW_SCREEN) return 0;
    cfw_draw_walk walk;walk.remaining=CFW_DRAW_MAX_OPS;walk.depth=0;walk.references=refs;
    return cfw_draw_list(ctx,id,target,apply,&walk);
}
