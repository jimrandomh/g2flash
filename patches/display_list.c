/* Revision 26. All calls inherit a target; a call-local override is scoped. */
#include "resource_cache.h"
static int cfw_draw_right_lens(void);
static int decode_image_rle(const uint8_t *,uint32_t,uint8_t *,uint32_t,uint32_t,uint32_t);
#define CFW_DRAW_MAX_OPS 4096u
#define CFW_DRAW_MAX_DEPTH 8u
#define CFW_DRAW_SCREEN 65535u
#define CFW_DRAW_CURRENT 65534u

/* Keep names and wire IDs in sync with Faceclaw DrawCallType.kt. */
typedef enum {
    DRAW_OP_BOUNDING_BOX = 1,
    DRAW_OP_RECT_COPY = 2,
    DRAW_OP_STOCK_FONT_STRING = 3,
    DRAW_OP_IMAGE = 4,
    DRAW_OP_TEXT = 5,
    DRAW_OP_REMAP_COLORS = 6,
    DRAW_OP_DISPLAY_LIST = 7,
    DRAW_OP_ROUNDED_RECT = 8,
} cfw_draw_op;

/* Keep flag names and values in sync with Faceclaw DrawFlags.kt. */
typedef enum {
    DRAW_FLAG_RESOURCE_TARGET = 1,
    DRAW_FLAG_DEPTH = 2,
    DRAW_FLAGS_MASK = DRAW_FLAG_RESOURCE_TARGET | DRAW_FLAG_DEPTH,
} cfw_draw_call_flags;
typedef enum {
    DRAW_BBOX_FLAG_U16 = 1,
} cfw_draw_bbox_flags;
#define DRAW_ROUNDED_RECT_NO_BORDER 16u

typedef struct { uint8_t *pixels; uint32_t width, height, stride; int32_t shift_x; } cfw_draw_target;
typedef struct { uint32_t remaining, depth; uint16_t ancestors[CFW_DRAW_MAX_DEPTH]; uint8_t *references; } cfw_draw_walk;

static int cfw_draw_resource_target(customCfwContext *ctx, uint32_t id, cfw_draw_target *out) {
    uint32_t size;
    const uint8_t *data = cfw_resource_get(ctx, id, &size);
    cfw_cached_image image;
    if (!data || (data[0] & (CFW_RESOURCE_TYPE_MASK | CFW_RESOURCE_FLAG_RLE)) || !cfw_texture_image_at(data, size, 0, &image)) return -1;
    out->pixels = (uint8_t *)image.rle; out->width = image.width; out->height = image.height;
    out->stride = (image.width + 1u) >> 1; out->shift_x = 0;
    return 0;
}
static void cfw_draw_ref(cfw_draw_walk *walk, uint32_t id) {
    if (walk->references && id < CFW_RESOURCE_COUNT) walk->references[id >> 3] |= 1u << (id & 7u);
}
static uint8_t cfw_draw_pixel(cfw_draw_target *t, uint32_t x, uint32_t y) {
    return (t->pixels[y * t->stride + (x >> 1)] >> ((x & 1u) ? 0 : 4)) & 15u;
}
static void cfw_draw_put(cfw_draw_target *t, int32_t x, int32_t y, uint8_t value) {
    x += t->shift_x;
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
    if (!data || size < 3 || data[0] != CFW_RESOURCE_TYPE_DISPLAY_LIST) return -1;
    cfw_draw_ref(walk, id);
    walk->ancestors[walk->depth++] = id;
    int result = cfw_draw_sequence(ctx, data + 1, size - 1, target, apply, walk);
    walk->depth--;
    return result;
}
static int cfw_draw_rounded_contains(int32_t x,int32_t y,int32_t w,int32_t h,int32_t radius) {
    if(x<0 || y<0 || x>=w || y>=h) return 0;
    int32_t r=radius;if(r>w/2) r=w/2;if(r>h/2) r=h/2;
    int32_t ex=x<w-1-x?x:w-1-x,ey=y<h-1-y?y:h-1-y;
    int32_t dx=2*r-(2*ex+1),dy=2*r-(2*ey+1);if(dx<0) dx=0;if(dy<0) dy=0;
    return dx*dx+dy*dy<=4*r*r;
}
static int cfw_draw_call(customCfwContext *ctx, const uint8_t *p, uint32_t n,
                          cfw_draw_target target, int apply, cfw_draw_walk *walk) {
    if (n < 2 || !walk->remaining--) return -1;
    cfw_draw_op op = (cfw_draw_op)p[0];
    uint32_t flags = p[1]; p += 2; n -= 2;
    if (flags & ~DRAW_FLAGS_MASK) return -1;
    if (flags & DRAW_FLAG_RESOURCE_TARGET) {
        if (n < 2) return -1;
        uint32_t id = rd16(p); p += 2; n -= 2;
        int32_t inherited_shift=target.shift_x;
        if (cfw_draw_resource_target(ctx, id, &target)) return -1;
        target.shift_x=inherited_shift;
        cfw_draw_ref(walk, id);
    }
    if (flags & DRAW_FLAG_DEPTH) {
        if (!n) return -1;
        int32_t depth=(int8_t)*p++; n--;
        int32_t value=cfw_draw_right_lens()?depth+1:depth;
        int32_t half=value<0?(value-1)/2:value/2;
        target.shift_x+=cfw_draw_right_lens()?-half:half;
    }
    if (op == DRAW_OP_BOUNDING_BOX) { /* bbox: flags, compact or u16 xywh, pixel RLE (no row padding). */
        if (!n || (p[0] & ~DRAW_BBOX_FLAG_U16)) return -1;
        uint32_t wide = p[0] & DRAW_BBOX_FLAG_U16, header = wide ? 9u : 5u;
        if (n < header) return -1;
        uint32_t x = wide ? rd16(p+1) : p[1]*4u, y = wide ? rd16(p+3) : p[2]*2u;
        uint32_t w = wide ? rd16(p+5) : p[3]*4u, h = wide ? rd16(p+7) : p[4]*2u;
        if (!w || !h || x > target.width || w > target.width-x || y > target.height || h > target.height-y) return -1;
        int fast = !wide && !(target.shift_x & 1) && (int32_t)x+target.shift_x>=0 && (int32_t)(x+w)+target.shift_x<=(int32_t)target.width;
        uint32_t pos = header, pixel = 0, total = w*h;
        while (pos < n && pixel < total) {
            uint32_t used, count; uint8_t color;
            if (!cfw_texture_rle_token(p+pos, n-pos, &used, &count, &color) || count > total-pixel) return -1;
            if (apply && !fast) for (uint32_t i=0; i<count; i++) cfw_draw_put(&target, x+(pixel+i)%w, y+(pixel+i)/w, color);
            pixel += count; pos += used;
        }
        if(pos!=n || pixel!=total) return -1;
        if(apply && fast && !decode_image_rle(p+header,n-header,target.pixels+y*target.stride+((x+target.shift_x)>>1),target.stride,w>>1,h)) return -1;
        return 0;
    }
    if (op == DRAW_OP_RECT_COPY) { /* source id, source xywh, destination xy. */
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
            int reverse = source.pixels == target.pixels && (dy>(int32_t)y || (dy==(int32_t)y && dx+target.shift_x>(int32_t)x));
            for (uint32_t i=0;i<w*h;i++) { uint32_t j=reverse?w*h-1-i:i;
                cfw_draw_put(&target,dx+(int32_t)(j%w),dy+(int32_t)(j/w),cfw_draw_pixel(&source,x+j%w,y+j/w)); }
        }
        return 0;
    }
    if (op == DRAW_OP_STOCK_FONT_STRING) {
        if (n<6 || n!=6u+p[5]) return -1;
        uint32_t pos=6;
        while(pos<n) { uint32_t used,cp; if(p[pos]>=1 && p[pos]<=31) { pos++; continue; }
            if(!cfw_texture_utf8(p+pos,n-pos,&used,&cp)) return -1; pos+=used; }
        if (!apply) return 0;
        cfw_rectlist rl; rl.n=0;
        return cfw_builtin_draw_string_shifted(target.pixels,target.stride,target.width,target.height,p,n,&rl,target.shift_x);
    }
    if (op == DRAW_OP_IMAGE || op == DRAW_OP_TEXT) {
        if ((op == DRAW_OP_IMAGE && n!=7) || (op == DRAW_OP_TEXT && (n<8 || n!=8u+p[7]))) return -1;
        uint32_t id=rd16(p),size; const uint8_t *data=cfw_resource_get(ctx,id,&size);
        if (!data) return -1;
        cfw_draw_ref(walk,id);
        cfw_cached_image image;
        if(op == DRAW_OP_IMAGE) { if(!cfw_texture_image_at(data,size,0,&image)) return -1; }
        else {
            if(size<193 || data[0]!=CFW_RESOURCE_TYPE_FONT) return -1;
            for(uint32_t i=8;i<n;i++) { uint32_t ch=p[i]; if(ch>=1 && ch<=31) continue;
                if(ch<32 || ch>127) return -1; uint32_t offset=rd16(data+1+(ch-32)*2);
                if(offset<193 || !cfw_texture_image_at(data,size,offset,&image)) return -1; }
        }
        if(!apply) return 0;
        cfw_rectlist rl; rl.n=0;
        return op == DRAW_OP_IMAGE ? cfw_texture_draw_image_shifted(target.pixels,target.stride,target.width,target.height,p,n,&rl,target.shift_x)
                     : cfw_texture_draw_string_shifted(target.pixels,target.stride,target.width,target.height,p,n,&rl,target.shift_x);
    }
    if (op == DRAW_OP_REMAP_COLORS) { /* xywh + sixteen packed LUT nibbles. */
        if(n!=16) return -1;
        uint32_t x=rd16(p),y=rd16(p+2),w=rd16(p+4),h=rd16(p+6);
        if(!w || !h || x>target.width || w>target.width-x || y>target.height || h>target.height-y) return -1;
        if(apply) for(uint32_t yy=y;yy<y+h;yy++) for(uint32_t xx=x;xx<x+w;xx++) {
            int32_t tx=(int32_t)xx+target.shift_x;
            if(tx<0 || tx>=(int32_t)target.width) continue;
            uint8_t v=cfw_draw_pixel(&target,tx,yy),lut=p[8+(v>>1)];
            cfw_draw_put(&target,xx,yy,(lut>>((v&1)?0:4))&15); }
        return 0;
    }
    if (op == DRAW_OP_ROUNDED_RECT) { /* signed xy, u16 wh/radius, max-blended fill, outline (16 = none). */
        if(n!=12) return -1;
        int32_t x=(int16_t)rd16(p),y=(int16_t)rd16(p+2);
        uint32_t w=rd16(p+4),h=rd16(p+6),radius=rd16(p+8),fill=p[10],border=p[11];
        if(!w || !h || w>640 || h>480 || fill>15 || border>DRAW_ROUNDED_RECT_NO_BORDER) return -1;
        if(apply) for(uint32_t yy=0;yy<h;yy++) for(uint32_t xx=0;xx<w;xx++) {
            if(!cfw_draw_rounded_contains(xx,yy,w,h,radius)) continue;
            int32_t tx=x+(int32_t)xx+target.shift_x,ty=y+(int32_t)yy;
            if(tx<0 || ty<0 || tx>=(int32_t)target.width || ty>=(int32_t)target.height) continue;
            int edge=!cfw_draw_rounded_contains((int32_t)xx-1,(int32_t)yy-1,(int32_t)w-2,(int32_t)h-2,radius?radius-1:0);
            uint8_t old=cfw_draw_pixel(&target,tx,ty);
            cfw_draw_put(&target,x+(int32_t)xx,ty,edge && border<DRAW_ROUNDED_RECT_NO_BORDER?border:(old>fill?old:fill));
        }
        return 0;
    }
    if (op == DRAW_OP_DISPLAY_LIST && n==2) return cfw_draw_list(ctx,rd16(p),target,apply,walk);
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
