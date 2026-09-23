/* Revision 27 extended integers and typed, bounded expression VM.
 * Included after cfw_reader in display_list.c. See docs/draw-expressions.md.
 * Opcode IDs and f32 evaluation order match Faceclaw DrawExpression.kt. */
#define CFW_EXPR_MAX_BYTES 1024u
#define CFW_EXPR_MAX_STACK 32u

typedef struct {
    uint32_t elapsed_ms;
    int animation_pending;
} cfw_expression_frame;

typedef enum {
    EX_PUSH_I32=1, EX_PUSH_F32=2, EX_DUP=3, EX_DROP=4, EX_SWAP=5,
    EX_IADD=16, EX_ISUB=17, EX_IMUL=18, EX_IDIV=19, EX_IMOD=20,
    EX_INEG=21, EX_IMIN=22, EX_IMAX=23,
    EX_FADD=32, EX_FSUB=33, EX_FMUL=34, EX_FDIV=35,
    EX_FNEG=36, EX_FMIN=37, EX_FMAX=38,
    EX_I2F=48, EX_F2I=49, EX_TIME=50,
    EX_LERP=64, EX_SMOOTHSTEP=65, EX_EASE_IN_QUAD=66, EX_EASE_OUT_QUAD=67,
    EX_EASE_IN_OUT_QUAD=68, EX_EASE_IN_CUBIC=69, EX_EASE_OUT_CUBIC=70,
    EX_EASE_IN_OUT_CUBIC=71,
} cfw_expression_opcode;

typedef union { uint32_t u; int32_t i; float f; } cfw_expression_value;

static uint32_t cfw_extended_integer(cfw_reader *r, uint32_t first, int is_signed) {
    uint32_t bytes, bits, value;
    if (first < 0x80) { bytes=1; bits=7; value=first; }
    else if (first < 0xc0) { bytes=2; bits=14; value=first & 0x3f; }
    else if (first < 0xe0) { bytes=3; bits=21; value=first & 0x1f; }
    else if (first < 0xf0) { bytes=4; bits=28; value=first & 0x0f; }
    else if (first == 0xf0) { bytes=5; bits=32; value=0; }
    else { r->failed=1; return 0; }
    for (uint32_t i=1; i<bytes; i++) value=(value<<8) | cfw_read_u8(r);
    if (is_signed && bits<32 && (value & (1u<<(bits-1)))) value |= ~((1u<<bits)-1u);
    return value;
}

static int cfw_expression_finite(float value) {
    cfw_expression_value v;
    v.f=value;
    return (v.u & 0x7f800000u) != 0x7f800000u;
}

static int cfw_expression_to_i32(float value, int32_t *out) {
    if (!cfw_expression_finite(value) || value < -2147483648.0f || value >= 2147483648.0f) return 0;
    *out=(int32_t)value; /* truncate toward zero */
    return 1;
}

__attribute__((noinline)) static int32_t cfw_expression_eval(cfw_reader r, cfw_expression_frame *frame) {
    cfw_expression_value stack[CFW_EXPR_MAX_STACK];
    uint8_t types[CFW_EXPR_MAX_STACK]; /* 0=i32, 1=f32 */
    uint32_t size=0;
    int pending=0;
    if (READ_REMAINING(r)>CFW_EXPR_MAX_BYTES) return 0;
    while (READ_REMAINING(r) && !r.failed) {
        uint32_t op=READ_U8(r);
        cfw_expression_value a, b, result;
        uint8_t type=0;
        if (op==EX_PUSH_I32 || op==EX_PUSH_F32) {
            if (op==EX_PUSH_I32) {
                uint32_t first=READ_U8(r);
                result.u=cfw_extended_integer(&r,first,1);
            } else {
                result.u=READ_U8(r);
                result.u|=READ_U8(r)<<8;
                result.u|=READ_U8(r)<<16;
                result.u|=READ_U8(r)<<24;
                type=1;
            }
        } else if (op==EX_DUP) {
            if (!size) return 0;
            result=stack[size-1]; type=types[size-1];
        } else if (op==EX_DROP) {
            if (!size) return 0;
            size--; continue;
        } else if (op==EX_SWAP) {
            if (size<2) return 0;
            a=stack[size-1]; type=types[size-1];
            stack[size-1]=stack[size-2]; types[size-1]=types[size-2];
            stack[size-2]=a; types[size-2]=type;
            continue;
        } else if (op>=EX_IADD && op<=EX_IMAX) {
            if (!size || types[size-1]) return 0;
            b=stack[--size];
            if (op==EX_INEG) result.u=0u-b.u;
            else {
                if (!size || types[size-1]) return 0;
                a=stack[--size];
                switch (op) {
                case EX_IADD: result.u=a.u+b.u; break;
                case EX_ISUB: result.u=a.u-b.u; break;
                case EX_IMUL: result.u=a.u*b.u; break;
                case EX_IDIV:
                case EX_IMOD:
                    if (!b.i) return 0;
                    if (a.u==0x80000000u && b.i==-1) result.u=op==EX_IDIV?a.u:0;
                    else result.i=op==EX_IDIV?a.i/b.i:a.i%b.i;
                    break;
                case EX_IMIN: result.i=a.i<b.i?a.i:b.i; break;
                default: result.i=a.i>b.i?a.i:b.i; break;
                }
            }
        } else if (op>=EX_FADD && op<=EX_FMAX) {
            if (!size || !types[size-1]) return 0;
            b=stack[--size]; type=1;
            if (op==EX_FNEG) result.f=-b.f;
            else {
                if (!size || !types[size-1]) return 0;
                a=stack[--size];
                switch (op) {
                case EX_FADD: result.f=a.f+b.f; break;
                case EX_FSUB: result.f=a.f-b.f; break;
                case EX_FMUL: result.f=a.f*b.f; break;
                case EX_FDIV: if (b.f==0.0f) return 0; result.f=a.f/b.f; break;
                case EX_FMIN: result.f=a.f<b.f?a.f:b.f; break;
                default: result.f=a.f>b.f?a.f:b.f; break;
                }
            }
        } else if (op==EX_I2F || op==EX_F2I || op==EX_TIME) {
            if (!size || types[size-1]!=(op==EX_F2I)) return 0;
            a=stack[--size];
            if (op==EX_I2F) { result.f=(float)a.i; type=1; }
            else if (op==EX_F2I) { if (!cfw_expression_to_i32(a.f,&result.i)) return 0; }
            else {
                if (a.i<0) return 0;
                result.u=frame->elapsed_ms<a.u?frame->elapsed_ms:a.u;
                if (result.u!=a.u) pending=1;
            }
        } else if (op==EX_LERP) {
            if (size<3 || !types[size-1] || !types[size-2] || !types[size-3]) return 0;
            float t=stack[--size].f;
            b=stack[--size]; a=stack[--size];
            result.f=a.f+(b.f-a.f)*t; type=1;
        } else if (op>=EX_SMOOTHSTEP && op<=EX_EASE_IN_OUT_CUBIC) {
            if (!size || !types[size-1]) return 0;
            float t=stack[--size].f;
            t=t<0.0f?0.0f:t>1.0f?1.0f:t;
            float u=1.0f-t;
            type=1;
            switch (op) {
            case EX_SMOOTHSTEP: result.f=t*t*(3.0f-2.0f*t); break;
            case EX_EASE_IN_QUAD: result.f=t*t; break;
            case EX_EASE_OUT_QUAD: result.f=1.0f-u*u; break;
            case EX_EASE_IN_OUT_QUAD: result.f=t<0.5f?2.0f*t*t:1.0f-2.0f*u*u; break;
            case EX_EASE_IN_CUBIC: result.f=t*t*t; break;
            case EX_EASE_OUT_CUBIC: result.f=1.0f-u*u*u; break;
            default: result.f=t<0.5f?4.0f*t*t*t:1.0f-4.0f*u*u*u; break;
            }
        } else return 0;
        if (r.failed || size==CFW_EXPR_MAX_STACK || (type && !cfw_expression_finite(result.f))) return 0;
        stack[size]=result; types[size++]=type;
    }
    if (r.failed || !size) return 0;
    int32_t value=stack[size-1].i;
    if (types[size-1] && !cfw_expression_to_i32(stack[size-1].f,&value)) return 0;
    frame->animation_pending |= pending;
    return value;
}

static int32_t cfw_read_extended(cfw_reader *r, cfw_expression_frame *frame) {
    uint32_t first=cfw_read_u8(r);
    if (first!=0xff) return (int32_t)cfw_extended_integer(r,first,1);
    first=cfw_read_u8(r);
    uint32_t length=cfw_extended_integer(r,first,0);
    const uint8_t *program=cfw_read_bytes(r,length);
    if (r->failed) return 0;
    return cfw_expression_eval(cfw_reader_init(program,length),frame);
}
