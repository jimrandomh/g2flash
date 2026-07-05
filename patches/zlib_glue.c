#include <stdint.h>

/*
 * zlib (DEFLATE) image support for the G2 CFW — multi-mode load wrapper.
 *
 * Replaces the one BMP-loader call FUN_0050164a(state, buf, len) in FUN_004ae69c.
 * Dispatch on the first byte of the reassembled image buffer:
 *
 *   'B' (0x42)  -> raw BMP: decode with our own fast 4bpp decoder (load_bmp_fast).
 *   1           -> [1][zlib]   4bpp BMP: inflate into the recon buffer's tail,
 *                              then decode with load_bmp_fast.
 *   2           -> [2][zlib]   8bpp full frame: inflate straight into the display
 *                              buffer (state+0x8, w*h, 1 byte/pixel), then push.
 *   3           -> [3][l/4][t/2][w/4][h/2][zlib]  bounding-box partial update:
 *                              inflate a tight-4bpp rectangle and expand it into
 *                              just that region of the (persistent) display buffer,
 *                              leaving the rest of the previous frame; then push.
 *                              Box origin/size are quantized (left/width *4, so
 *                              even; top/height *2) to fit one byte each. Rejected
 *                              if the box is not wholly within the container.
 *   4           -> [4][zlib]   8bpp stereo pair (left frame then right frame, w*h
 *                              each); each lens keeps only its half (FW_SIDE).
 *   5           -> [5][...]    play a UI sound on the arm buzzer (no display change).
 *                              The G2 "speaker" is a PWM piezo buzzer — it can only
 *                              emit square-wave tones, not PCM/WAV — so this drives
 *                              the firmware's own buzzer driver instead of streaming
 *                              samples. Sub-dispatch on src[1]:
 *                                0 [0][type]            -> DRV_BuzzerPlayAfterQueue:
 *                                     play preset voice `type` (0..8) from the flash
 *                                     preset table (single beep / alarm / ringtone).
 *                                1 [1][note][oct][beat] -> DRV_BuzzerPlayNote: one
 *                                     tone. note 1..7, oct 0..3 (freq from the 28-
 *                                     entry note table), beat = duration in ~62ms
 *                                     units. Good for click/beep on tap/notification.
 *                                2 [2]                  -> stop/silence the buzzer.
 *                                3 [3][freqLo][freqHi][duty][msLo][msHi] -> raw tone:
 *                                     program the PWM to an ARBITRARY frequency
 *                                     (1..20000 Hz, 16-bit LE) at `duty` percent
 *                                     (0..100) for `ms` milliseconds (16-bit LE).
 *                                     Bypasses the 7-note x 4-octave lookup table
 *                                     entirely (that table is just a convenience);
 *                                     the hardware timer takes any Hz. Auto-stops
 *                                     by arming the buzzer's own osTimer with a
 *                                     null note list so the driver's timer callback
 *                                     shuts the PWM off after `ms`. Enables fine /
 *                                     microtonal pitch, chirps and pitch sweeps
 *                                     (send a run of these), and sub-62ms durations.
 *                              The preset/note/stop entries are self-contained fw
 *                              entries that queue into the buzzer's osTimer; the
 *                              raw-tone entry drives the low-level PWM start and
 *                              arms that same osTimer for auto-stop. None spin or
 *                              block here. Returns 0 (success).
 *   6           -> [6][zlib]   headerless 4bpp full frame: inflate the tightly
 *                              packed 4bpp pixels (h * ceil(w/2) bytes, top-down)
 *                              and expand them into the 8bpp display buffer, push.
 *   anything else / too short  -> load_bmp_fast (rejects cleanly if not a BMP).
 *
 * The image dimensions come from the container state (state+0x40/0x42), so no
 * header is needed on 8bpp payloads — the sender just deflates w*h raw bytes
 * (mode 2); mode 3 carries its own 4-byte box header, and mode 6 tight 4bpp.
 *
 * Every invocation (any mode) first kicks the EvenHub keepalive: stock firmware
 * resets the ticks-since-heartbeat counter only on the sid-0x0c heartbeat msg, so
 * a client streaming image updates to maximize throughput would otherwise trip the
 * "Connection lost" teardown. See FW_KEEPALIVE_RESET at the top of load_image_z.
 *
 * BMP handling (modes 'B' and 1) does NOT use the stock loader FUN_0050164a: that
 * decoder runs two non-inlined function calls PER PIXEL (palette pack + luminance
 * blend) plus a whole-buffer CRC, which costs more CPU than the airtime it saves.
 * load_bmp_fast instead does a direct 4bpp-nibble -> 8bpp (nibble*17) expand,
 * ignoring the palette (the sender only ever uses the standard gray ramp). The
 * stock loader is kept only as a fallback for non-4bpp or mismatched-size BMPs.
 *
 * Modes 2/3/4/6 and the BMP decoder replicate the stock loader's tail to get the
 * buffer onto the panel: dcache-clean the display buffer, set the LVGL image
 * descriptor (state+0x24..0x34), lv_image_set_src, lv_obj_invalidate.
 *
 * Self-contained: no external symbols, no writable globals. Firmware entry points
 * are called by absolute constant address (movw/movt + blx, no relocation). The
 * only placement-dependent constants are ZWRAP_ALLOC_ADDR / ZWRAP_FREE_ADDR,
 * filled in on the 2nd build pass (zwrap_alloc/zwrap_free are emitted first so
 * their offsets, hence addresses, are stable).
 */

typedef void *(*malloc_fn)(uint32_t);
typedef void (*free_fn)(void *);
typedef int (*inflateInit2_fn)(void *strm, int windowBits, const char *ver, int ssize);
typedef int (*inflate_fn)(void *strm, int flush);
typedef int (*inflateEnd_fn)(void *strm);
typedef int (*loadbmp_fn)(void *state, void *bmp, uint32_t len);
typedef void (*cacheflush_fn)(void *desc);          /* desc = uint32[2]{ptr,size} */
typedef void (*lv_set_src_fn)(uint32_t obj, void *desc);
typedef void (*lv_invalidate_fn)(uint32_t obj);
typedef uint32_t (*lens_side_fn)(void);             /* 2 = LEFT lens, 1 = RIGHT lens */
typedef void (*buzz_preset_fn)(uint32_t type);      /* DRV_BuzzerPlayAfterQueue */
typedef void (*buzz_note_fn)(uint32_t note, uint32_t tone, uint32_t beat); /* DRV_BuzzerPlayNote */
typedef void (*buzz_reset_fn)(void);                /* buzzer stop/reset */
typedef void (*buzz_raw_fn)(uint32_t freq, uint32_t duty);   /* reset+power+PWM(freq,duty) */
typedef int  (*timer_start_fn)(uint32_t handle, uint32_t ms); /* osTimer start (one-shot) */
typedef void (*keepalive_reset_fn)(void);           /* zero the EvenHub keepalive counter */

/* firmware entry points (Thumb bit set for blx via constant pointer) */
#define FW_MALLOC  ((malloc_fn)0x00472b6fU)         /* FUN_00472b6e malloc(size) */
#define FW_FREE    ((free_fn)0x00472bb3U)           /* FUN_00472bb2 free(ptr) */
#define FW_INIT2   ((inflateInit2_fn)0x005be643U)   /* FUN_005be642 inflateInit2_ */
#define FW_INFLATE ((inflate_fn)0x005be711U)        /* FUN_005be710 inflate */
#define FW_END     ((inflateEnd_fn)0x005be607U)     /* FUN_005be606 inflateEnd */
#define FW_LOADBMP ((loadbmp_fn)0x0050164bU)        /* FUN_0050164a BMP decoder */
#define FW_FLUSH   ((cacheflush_fn)0x00472f87U)     /* FUN_00472f86 dcache clean range */
#define FW_SETSRC  ((lv_set_src_fn)0x004b0f01U)     /* FUN_004b0f00 lv_image_set_src */
#define FW_INVAL   ((lv_invalidate_fn)0x004405f7U)  /* FUN_004405f6 lv_obj_invalidate */
#define FW_SIDE    ((lens_side_fn)0x0045a8edU)       /* FUN_0045a8ec -> 2=left, 1=right */
#define FW_BUZZ_PRESET ((buzz_preset_fn)0x004e97efU) /* FUN_004e97ee DRV_BuzzerPlayAfterQueue(type 0..8) */
#define FW_BUZZ_NOTE   ((buzz_note_fn)0x004e988dU)   /* FUN_004e988c DRV_BuzzerPlayNote(note,tone,beat) */
#define FW_BUZZ_RESET  ((buzz_reset_fn)0x004e9759U)  /* FUN_004e9758 buzzer stop/reset */
#define FW_BUZZ_RAW    ((buzz_raw_fn)0x004e991dU)     /* FUN_004e991c reset+power+PWM(freq,duty) */
#define FW_TIMER_START ((timer_start_fn)0x004484ebU)  /* FUN_004484ea osTimerStart(handle,ms) */
#define FW_KEEPALIVE_RESET ((keepalive_reset_fn)0x00505cc3U) /* FUN_00505cc2: EvenHub keepalive
                                                     * counter (@0x200744e8) = 0. This is the exact
                                                     * leaf the stock sid-0x0c heartbeat handler in
                                                     * FUN_004ae69c calls; it takes no args and reads
                                                     * the counter pointer from its own literal pool. */
#define BUZZ_TIMER_ADDR 0x20074440U                   /* RAM: buzzer osTimer handle (DAT_004e996c) */
#define ZLIB_VER   ((const char *)0x007885e4U)      /* "1.1.4" */

#ifndef ZWRAP_ALLOC_ADDR
#define ZWRAP_ALLOC_ADDR 0u   /* pass-2 placeholders */
#define ZWRAP_FREE_ADDR  0u
#endif

/* z_stream (zlib 1.1.4, sizeof = 0x38) field offsets */
#define ZS_NEXT_IN   0x00
#define ZS_AVAIL_IN  0x04
#define ZS_NEXT_OUT  0x0c
#define ZS_AVAIL_OUT 0x10
#define ZS_TOTAL_OUT 0x14
#define ZS_ZALLOC    0x20
#define ZS_ZFREE     0x24
#define ZS_OPAQUE    0x28
#define ZS_SIZE      0x38

#define XOR_CHUNK 256   /* mode-4 inflate scratch (stack); any size works */

void *zwrap_alloc(void *opaque, uint32_t items, uint32_t size) {
    (void)opaque;
    return FW_MALLOC(items * size);
}

void zwrap_free(void *opaque, void *ptr) {
    (void)opaque;
    FW_FREE(ptr);
}

static void push_display(uint8_t *state, uint8_t *disp, uint32_t w, uint32_t h);
static void unpack4bpp(uint8_t *dst, uint32_t dst_stride, const uint8_t *pix,
                       uint32_t w, uint32_t h, uint32_t src_stride, int bottom_up);
static int load_bmp_fast(uint8_t *state, const uint8_t *bmp, uint32_t len);

int load_image_z(void *state_, uint8_t *src, uint32_t srclen) {
    /* An inbound image message proves the phone is still connected, so kick the
     * EvenHub keepalive back to life exactly as the stock heartbeat handler does.
     * Stock firmware resets the ticks-since-last-heartbeat counter (@0x200744e8)
     * ONLY on the sid-0x0c heartbeat message; the periodic evenhub_ui_event_handler
     * (FUN_00506460, param_1==4) increments it every tick and, once it passes 899,
     * fires FUN_004fee62(0,0) -> "DISPLAY_AUTO_REFLASH heartbeat timeout" -> the
     * "Connection lost" context teardown. A client streaming image updates to
     * maximize throughput would otherwise have to interleave heartbeats to avoid
     * that teardown; resetting here lets a steady image stream keep the context
     * alive on its own. Placed before any dispatch so every mode (image, delta,
     * stereo, sound, BMP) counts as liveness. Runs on the same evenhub task that
     * owns the counter, so no locking is needed. */
    FW_KEEPALIVE_RESET();

    uint8_t *state = (uint8_t *)state_;
    if (src == 0 || srclen < 1) return load_bmp_fast(state, src, srclen);

    uint8_t mode = src[0];
    if (mode == 0x42) return load_bmp_fast(state, src, srclen);       /* raw BMP */

    if (mode == 5) {
        /* play a UI sound on the buzzer; no display change. [5][kind][args...].
         * These entry points copy their args into firmware-owned storage (the
         * preset table lives in flash; PlayNote copies note/tone/beat into an
         * 8-byte scratch), so the recon buffer is free to be reused immediately
         * and the async buzzer timer never reads from it. */
        uint8_t kind = (srclen >= 2) ? src[1] : 0xffu;
        if (kind == 0 && srclen >= 3) {                 /* preset 0..8 */
            if (src[2] <= 8) FW_BUZZ_PRESET(src[2]);
        } else if (kind == 1 && srclen >= 5) {          /* single tone */
            uint8_t note = src[2], oct = src[3], beat = src[4];
            /* note 1..7 x oct 0..3 keeps the freq-table index in [0,27] so the
             * driver's `1000000 / (0xffff - table[idx])` can never divide by 0 */
            if (note >= 1 && note <= 7 && oct <= 3 && beat != 0)
                FW_BUZZ_NOTE(note, oct, beat);
        } else if (kind == 2) {                         /* stop / silence */
            FW_BUZZ_RESET();
        } else if (kind == 3 && srclen >= 7) {          /* raw tone: freq/duty/ms */
            uint32_t freq = (uint32_t)src[2] | ((uint32_t)src[3] << 8);
            uint32_t duty = src[4];
            uint32_t ms   = (uint32_t)src[5] | ((uint32_t)src[6] << 8);
            if (freq < 1) freq = 1;                     /* freq 0 -> bad PWM period */
            if (freq > 20000) freq = 20000;             /* hw range per AT^BUZZER */
            if (duty > 100) duty = 100;                 /* duty is a 0..100 percent */
            if (ms < 1) ms = 1;
            FW_BUZZ_RAW(freq, duty);                    /* reset+power+PWM; note list now null */
            uint32_t h = *(volatile uint32_t *)BUZZ_TIMER_ADDR;
            if (h) FW_TIMER_START(h, ms);               /* callback stops PWM after ms */
        }
        return 0;
    }

    if (mode < 1 || mode > 6 || srclen < 3) return load_bmp_fast(state, src, srclen);

    const uint8_t *zsrc = src + 1;
    uint32_t zlen = srclen - 1;
    uint32_t w = *(uint16_t *)(state + 0x40);
    uint32_t h = *(uint16_t *)(state + 0x42);
    uint32_t wh = w * h;

    uint8_t strm[ZS_SIZE];
    for (uint32_t i = 0; i < ZS_SIZE; i++) strm[i] = 0;
    *(const uint8_t **)(strm + ZS_NEXT_IN) = zsrc;
    *(uint32_t *)(strm + ZS_AVAIL_IN) = zlen;
    *(uint32_t *)(strm + ZS_ZALLOC) = ZWRAP_ALLOC_ADDR;
    *(uint32_t *)(strm + ZS_ZFREE) = ZWRAP_FREE_ADDR;
    *(uint32_t *)(strm + ZS_OPAQUE) = 0;

    if (mode == 1 || mode == 6) {
        /* Both inflate a whole payload into the recon buffer's tail (no scratch
         * unless it won't fit), then hand off: mode 1 is a full 4bpp BMP decoded
         * by load_bmp_fast; mode 6 is headerless tight 4bpp expanded directly. */
        uint32_t out_max;
        if (mode == 1) {
            uint32_t row_stride = (((w + 1) >> 1) + 3) & ~3u;   /* BMP 4-byte-padded */
            out_max = 118 + row_stride * h + 64;
        } else {
            out_max = ((w + 1) >> 1) * h;                       /* tight 4bpp */
        }
        uint8_t *dst;
        int allocated = 0;
        if ((uint64_t)out_max + zlen + 1 <= wh) {     /* +1: input starts at src[1] */
            dst = src + (wh - out_max);
        } else {
            dst = (uint8_t *)FW_MALLOC(out_max);
            if (dst == 0) return -1;
            allocated = 1;
        }
        *(uint8_t **)(strm + ZS_NEXT_OUT) = dst;
        *(uint32_t *)(strm + ZS_AVAIL_OUT) = out_max;
        int ret = -1;
        if (FW_INIT2(strm, 15, ZLIB_VER, ZS_SIZE) == 0) {
            int r = FW_INFLATE(strm, 4);              /* Z_FINISH (whole frame fits) */
            uint32_t out = *(uint32_t *)(strm + ZS_TOTAL_OUT);
            FW_END(strm);
            if (r == 1) {
                if (mode == 1) {
                    ret = load_bmp_fast(state, dst, out);
                } else if (out == out_max) {
                    uint8_t *disp = *(uint8_t **)(state + 0x8);
                    unpack4bpp(disp, w, dst, w, h, (w + 1) >> 1, 0);
                    push_display(state, disp, w, h);
                    ret = 0;
                }
            }
        } else {
            FW_END(strm);
        }
        if (allocated) FW_FREE(dst);
        return ret;
    }

    if (mode == 3) {
        /* bounding-box partial update — rewrites just one rectangle of the
         * (persistent) display buffer; the rest of the previous frame stays.
         *   [3][left/4][top/2][width/4][height/2][zlib tight-4bpp box pixels]
         * left/width are stored /4 and top/height /2 so each fits one byte; the
         * *4 on left/width also guarantees they're even, so the box's source rows
         * are whole bytes (bw/2) and it starts on a byte boundary. The 4bpp box is
         * inflated into the recon-buffer tail, then expanded into the display
         * buffer at (left,top) with the full width as the destination stride.
         * Rejected (old frame left on screen) if the box isn't wholly in bounds. */
        if (srclen < 6) return -1;                    /* 4 hdr bytes + some zlib */
        uint32_t left = (uint32_t)src[1] * 4;
        uint32_t top  = (uint32_t)src[2] * 2;
        uint32_t bw   = (uint32_t)src[3] * 4;
        uint32_t bh   = (uint32_t)src[4] * 2;
        if (bw == 0 || bh == 0 || left + bw > w || top + bh > h) return -1;

        uint32_t packed = (bw >> 1) * bh;             /* bw even -> bw/2 B per row */
        const uint8_t *bzsrc = src + 5;
        uint32_t bzlen = srclen - 5;
        uint8_t *dst;
        int allocated = 0;
        if ((uint64_t)packed + bzlen + 5 <= wh) {     /* +5: input starts at src[5] */
            dst = src + (wh - packed);
        } else {
            dst = (uint8_t *)FW_MALLOC(packed);
            if (dst == 0) return -1;
            allocated = 1;
        }
        *(const uint8_t **)(strm + ZS_NEXT_IN) = bzsrc;  /* zlib is past the 4 hdr B */
        *(uint32_t *)(strm + ZS_AVAIL_IN) = bzlen;
        *(uint8_t **)(strm + ZS_NEXT_OUT) = dst;
        *(uint32_t *)(strm + ZS_AVAIL_OUT) = packed;
        int ret = -1;
        if (FW_INIT2(strm, 15, ZLIB_VER, ZS_SIZE) == 0) {
            int r = FW_INFLATE(strm, 4);              /* Z_FINISH (whole box fits) */
            uint32_t out = *(uint32_t *)(strm + ZS_TOTAL_OUT);
            FW_END(strm);
            if (r == 1 && out == packed) {
                uint8_t *disp = *(uint8_t **)(state + 0x8);
                unpack4bpp(disp + top * w + left, w, dst, bw, bh, bw >> 1, 0);
                push_display(state, disp, w, h);
                ret = 0;
            }
        } else {
            FW_END(strm);
        }
        if (allocated) FW_FREE(dst);
        return ret;
    }

    /* modes 2 & 4: 8bpp straight into the display buffer (separate from recon) */
    uint8_t *disp = *(uint8_t **)(state + 0x8);
    if (FW_INIT2(strm, 15, ZLIB_VER, ZS_SIZE) != 0) {
        FW_END(strm);
        return -1;
    }

    int ok = 0;
    if (mode == 2) {
        /* full frame: inflate directly into the display buffer */
        *(uint8_t **)(strm + ZS_NEXT_OUT) = disp;
        *(uint32_t *)(strm + ZS_AVAIL_OUT) = wh;
        int r = FW_INFLATE(strm, 4);                  /* Z_FINISH */
        ok = (r == 1 && *(uint32_t *)(strm + ZS_TOTAL_OUT) == wh);
    } else {
        /* mode 4: stereo pair [left frame (w*h)][right frame (w*h)]; this lens
         * keeps only its half. FW_SIDE(): 2=left -> first half, else -> second. */
        uint32_t want_lo = (FW_SIDE() == 2) ? 0 : wh;
        uint32_t want_hi = want_lo + wh;
        uint8_t chunk[XOR_CHUNK];
        uint32_t off = 0;
        for (;;) {
            *(uint8_t **)(strm + ZS_NEXT_OUT) = chunk;
            *(uint32_t *)(strm + ZS_AVAIL_OUT) = XOR_CHUNK;
            int r = FW_INFLATE(strm, 0);              /* Z_NO_FLUSH */
            uint32_t got = (uint32_t)(*(uint8_t **)(strm + ZS_NEXT_OUT) - chunk);
            for (uint32_t j = 0; j < got; j++) {
                uint32_t p = off + j;
                if (p >= want_lo && p < want_hi) disp[p - want_lo] = chunk[j];
            }
            off += got;
            if (off >= want_hi) { ok = 1; break; }    /* copied our whole half */
            if (r != 0 || got == 0) break;            /* end/error before full half */
        }
    }
    FW_END(strm);
    if (!ok) return -1;                               /* leave the old frame on screen */

    push_display(state, disp, w, h);
    return 0;
}

/* Replicate FUN_0050164a's tail: clean the display buffer out of dcache, set the
 * LVGL image descriptor for an 8bpp (cf=0x619) w*h image, rebind and invalidate.
 * (Defined after load_image_z so the entry offset, hence the bl target, is fixed.) */
static void push_display(uint8_t *state, uint8_t *disp, uint32_t w, uint32_t h) {
    uint32_t wh = w * h;
    uint32_t desc[2];
    desc[0] = (uint32_t)(uintptr_t)disp;
    desc[1] = wh;
    FW_FLUSH(desc);

    *(uint32_t *)(state + 0x24) = 0x619u;                                  /* cf/header */
    *(uint32_t *)(state + 0x2c) = (*(uint32_t *)(state + 0x2c) & 0xffff0000u) | w;
    *(uint32_t *)(state + 0x28) = (h << 16) | w;
    *(uint32_t *)(state + 0x30) = wh;
    *(uint32_t *)(state + 0x34) = (uint32_t)(uintptr_t)disp;

    uint32_t obj = *(uint32_t *)(state + 4);
    FW_SETSRC(obj, state + 0x24);
    FW_INVAL(obj);
}

/* little-endian unaligned reads (byte-wise; -mno-unaligned-access safe) */
static uint32_t rd16(const uint8_t *p) { return (uint32_t)p[0] | ((uint32_t)p[1] << 8); }
static uint32_t rd32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Expand a w*h block of 4bpp pixels (2 px/byte, high nibble = left pixel) into an
 * 8bpp destination: nibble n (0..15) -> n*17 (== (n<<4)|n) so 0->0, 15->255.
 * `src_stride` is bytes per source row; `dst_stride` is bytes per destination row
 * (= the full display width when writing a sub-rectangle); `bottom_up` flips the
 * source row order (BMP). */
static void unpack4bpp(uint8_t *dst, uint32_t dst_stride, const uint8_t *pix,
                       uint32_t w, uint32_t h, uint32_t src_stride, int bottom_up) {
    for (uint32_t y = 0; y < h; y++) {
        uint32_t srcY = bottom_up ? (h - 1 - y) : y;
        const uint8_t *row = pix + srcY * src_stride;
        uint8_t *out = dst + y * dst_stride;
        for (uint32_t x = 0; x < w; x++) {
            uint8_t b = row[x >> 1];
            uint8_t nib = (x & 1) ? (b & 0x0f) : (uint8_t)(b >> 4);
            out[x] = (uint8_t)(nib * 17);
        }
    }
}

/* Fast replacement for the stock BMP loader FUN_0050164a: decode a 4bpp indexed
 * BMP straight into the 8bpp display buffer via unpack4bpp, ignoring the palette
 * (always the gray ramp) and skipping the per-pixel color calls + CRC pass. Only
 * width/height/bpp/pixel-offset are read from the header. Falls back to the stock
 * loader for anything that isn't a 4bpp BMP matching the container dimensions. */
static int load_bmp_fast(uint8_t *state, const uint8_t *bmp, uint32_t len) {
    if (bmp == 0 || len < 0x36 || bmp[0] != 0x42 || bmp[1] != 0x4d)  /* "BM" */
        return FW_LOADBMP(state, (void *)bmp, len);
    if (rd16(bmp + 0x1c) != 4)                                       /* not 4bpp */
        return FW_LOADBMP(state, (void *)bmp, len);

    uint32_t dataoff = rd32(bmp + 0x0a);
    int32_t bh_signed = (int32_t)rd32(bmp + 0x16);
    uint32_t w = rd32(bmp + 0x12);
    uint32_t h = (bh_signed < 0) ? (uint32_t)(-bh_signed) : (uint32_t)bh_signed;
    int bottom_up = bh_signed > 0;

    /* Dimensions must match the container's display buffer, else let the stock
     * loader handle (and reject) it — avoids writing past the display buffer. */
    if (w != *(uint16_t *)(state + 0x40) || h != *(uint16_t *)(state + 0x42) ||
        (uint64_t)dataoff >= len)
        return FW_LOADBMP(state, (void *)bmp, len);

    uint32_t stride = (((w + 1) >> 1) + 3) & ~3u;   /* BMP rows padded to 4 bytes */
    uint8_t *disp = *(uint8_t **)(state + 0x8);
    unpack4bpp(disp, w, bmp + dataoff, w, h, stride, bottom_up);
    push_display(state, disp, w, h);
    return 0;
}
