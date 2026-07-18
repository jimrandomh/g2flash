// CFW capability advertisement.
//
// Appends one extra protobuf field to the sid=0x09 device-settings READ response
// (G2SettingPackage) right before it is framed and sent, so a connected app can
// detect this custom firmware and discover which extensions it supports without
// any timeout-based probing. The field is:
//
//   field 100, wire type 2 (length-delimited string):
//     "EVENCFW/<ver> <space-separated feature tokens>"
//
// Tag 100 is far above the stock message's fields (1..19), so stock decoders and
// the phone bridge skip it as an unknown field -- fully backward compatible.
//
// HOOK: the settings responder FUN_004b42b4 ends with
//     r0=type(1) r1=sid(9) r2=buf r3=len ; bl FUN_00475b14   ; aa21 send
// We retarget that one `bl` to settings_send_wrapper. The 4 send args are already
// in r0..r3, so the wrapper appends to `buf` (a 256-byte static response buffer
// at 0x200706ec that only uses ~40 B) and tail-calls the real sender with the
// grown length. Only this call site is redirected, but we still guard on sid==9.

typedef int (*send_fn)(int type, int sid, unsigned char *buf, unsigned len);

#define FW_SEND 0x00475b15 /* FUN_00475b14 | thumb bit */

// Capability string "EVENCFW/<ver> <space-separated feature tokens>":
//   EVENCFW/1  -> magic prefix + contract version (detect: starts-with "EVENCFW/")
//   img576     -> 576x288 image containers (vs stock 288x144 cap)
//   imgz       -> zlib (DEFLATE) compressed image payloads
//   xordelta   -> 8bpp XOR-delta frame updates (modes 2/3)
//   stereo     -> per-lens stereo image pairs (mode 4)
//
// The string is a normal rodata literal now that build.py emits/relocates .rodata
// (earlier this had to be spelled out byte-by-byte to avoid a rodata section). strlcpy
// comes from zlib_glue.c, which shares this translation unit via patches_main.c.
int settings_send_wrapper(int type, int sid, unsigned char *buf, unsigned len) {
    if (sid == 9) {
        static const char caps[] = "EVENCFW/2 img576 imgz rle";
        unsigned char *p = buf + len;
        p[0] = 0xA2; p[1] = 0x06;                          // field 100, wire type 2: tag 802
        unsigned clen = strlcpy((char *)(p + 3), caps, sizeof(caps));
        p[2] = (unsigned char)clen;                        // length-delimited payload length
        len += 3 + clen;
    }
    return ((send_fn)FW_SEND)(type, sid, buf, len);
}
