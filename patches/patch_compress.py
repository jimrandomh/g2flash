#!/usr/bin/env python3
"""
Build a CFW image for g2_2.2.4.34 with:
  (1) the 576x288 image-container size lift (same 3 edits as
      patches/patch_img_container_576.py),
  (2) 1bpp->4bpp image decompression on ImageRawDataUpdate.CompressMode, and
  (3) a CFW capability-advertisement field (protobuf field 100, a feature-token
      string) appended to the sid=0x09 settings READ response, so a connected
      app can detect this firmware and which extensions it supports.

(2) injects frag_write() (patches/decompress.c, built by build.py) over the
unused production-test handler set_aging_test_info (ghidra 0x491340, ~2KB, zero
xrefs) and retargets the three per-fragment memcpy calls (FUN_00439be4) in the
ImageRawDataUpdate handler FUN_004ff8fc to it.

frag_write keeps the memcpy ABI (r0=dst,r1=src,r2=len) and reads CompressMode
itself from the decoded-message buffer (*(0x0050066c)+0x1c). When CompressMode!=0
it treats len as the OUTPUT (4bpp) size and expands len/4 1bpp source bytes; the
sender therefore declares MapTotalSize / MapFragmentPacketSize in 4bpp units and
puts 1bpp bytes in MapRawData. CompressMode==0 is a byte-identical plain copy, so
stock (uncompressed) image updates are unaffected.

Length-preserving; recomputes the EVENOTA component CRC32C (TOC + sub-header echo)
and the mainApp preamble zlib-CRC32, exactly like patch_img_container_576.py.
"""
import sys, struct, zlib

DELTA = 0x39E680  # file_off = ghidra_addr - DELTA  (OTA mainApp component)

def g2f(addr):
    return addr - DELTA

# zlib image glue (patches/zlib_glue.c -> build.py pass2, 1014 B) placed at
# ghidra 0x491400 (= bufbase set_aging_test_info tail, after frag_write). Exports
# zwrap_alloc@0x491400, zwrap_free@0x49140e, load_image_z@0x49141a. Mode dispatch:
# 'B'=raw BMP, 1=zlib 4bpp BMP (recon-tail), 2=zlib 8bpp full frame ->display buffer,
# 3=8bpp XOR delta; 4=8bpp stereo pair (per-lens half via FW_SIDE); 2/3/4 push via
# loader tail; 5=play a buzzer UI sound (0 preset / 1 note / 2 stop; no display change).
ZLIB_GLUE = bytes.fromhex(
    "02fb01f042f66f31c0f247010847084642f6b331c0f2470108472de9f04fd3b0"
    "814629b122b10d78052d0ad0422d1ad141f24b63c0f25003484653b0bde8f04f"
    "1847022ac0f09e81487844d0002842d18878082800f2968149f25971c0f24e01"
    "963188478ee1032ae2d3681f10f1050fded9b9f84060b9f84240002045ab0027"
    "d8550137382ffbd1501e41f20142c0f249024d920e324b1c04fb06f84e920022"
    "012dcde945304f9230d1731c032707eb530323f00303634303f1b60616eb000a"
    "42f1000bbaeb08007bf100003ad2a8eb06000d1840e0052a2cd301282ad18878"
    "0024421e062a00f25181cb78032b00f24d810a79002a00f0498149f25971c0f2"
    "4e0101f59a771946b8473be14ef2076bc0f25b0b48f2e452d9f808a00bf13c07"
    "45a8c0f278020f213823b84740b345a8d84729e1022860d149f25970c0f24e00"
    "80471fe142f66f31c0f24701304688470546002800f018814ef20767c0f25b07"
    "48f2e452cde9485607f13c0645a8c0f278020f213823b047b8b145a8b8472be0"
    "032d72d0022d40f0a2800bf5857245a80421cde948a890470128c8d14a98a0eb"
    "0800b0fa80f04509e3e045ac07f585722046042190474a99064620460491b847"
    "012e09d141f24b63049ac0f25003484629469847044601e04ff0ff34baeb0800"
    "7bf10000c0f0d28042f66f30c0f2470000f1440128468847c8e0072a4ff00004"
    "c0f0c480032840f0c1808878cb780a7940ea03204d798e7901284ff0010498bf"
    "204644f62061884228bf084649f25971c0f24e01642a28bf642201f5e2731146"
    "984744f24040c2f207000068002800f0998045ea062148f2eb42012988bf0c46"
    "c0f24402214690478ce0cdf80ca0cde9016400260df1140a0bf5857ba8eb0600"
    "b0f5807f28bf4ff48070cde948a045a80021d8474899b1eb0a040cd003995346"
    "8a19214613f8015b1778013987ea050702f8017bf6d1012826444dd000284ff0"
    "00054ed1b4fa84f04009d7d049e04af6ed00c0f24500cde901648047861e5846"
    "18bf464606eb0807cdf80ca0aaeb060800250df1140b00f585704ff0000a0490"
    "4ff48070049a499045a80021cdf820b19047ddf820e1beeb0b0c10d053465c46"
    "624600bfb34204d3bb423cbf217808f80310013a04f1010403f10103f2d1e244"
    "beeb0b0118bf0121ba4528bf012508d238b90029d4d104e0a6eb0800b0fa80f0"
    "4509dde901644ef2076bddf80ca0c0f25b0b45a8d8473db14846514632462346"
    "00f008f8002401e04ff0ff34204653b0bde8f08f2de9f04182b003fb02f68846"
    "cde9001642f6877107466846c0f247011c461546884740f2196047f8240f7889"
    "40f6017245ea0040b86045ea044057f8204c7860c0f24b0220463946fe60c7f8"
    "1080904740f2f751c0f244012046884702b0bde8f081"
)

# settings capability-advertisement wrapper (patches/settings_ext.c ->
# build.py, 256 B) placed at ghidra 0x4917f8 (dead set_aging_test_info tail,
# after the zlib glue). Hooks the one `bl FUN_0047398c` (aa21 send) at the tail
# of the settings responder FUN_004b42b4: appends protobuf field 100 (string
# "EVENCFW/1 img576 imgz xordelta stereo") to the sid=0x09 READ response so a
# connected app can detect this CFW and its extensions, then tail-calls send.
SETTINGS_EXT = bytes.fromhex(
    "092978d12de9f04102eb030c56248cf804404e248cf8064043248cf8074046248c"
    "f8084057248cf809402f248cf80a4031244ff0a20e8cf80b406924352702f803e0"
    "4ff0060e8cf80d408cf8107037278cf814407a248cf801e04ff0250e67268cf811"
    "7036278cf8174078248cf802e04ff0450e6d258cf80f608cf812708cf816608cf8"
    "194064266c2761248cf803e08cf805e04ff0200e8cf80e508cf815504ff06f0872"
    "258cf81c6065268cf81e7074278cf82040732428338cf80ce08cf813e08cf818e0"
    "8cf81a808cf81b508cf81d608cf81f708cf821e08cf822408cf823708cf824608c"
    "f825508cf826608cf82780bde8f04143f68d1cc0f2470c6047"
)

# frag_write machine code (patches/decompress.c -> build.py, text+0x80, 106 B)
FRAG_WRITE = bytes.fromhex(
    "f0b540f26c63c0f250031b681b6a13b35fea920c08bff0bd00238646ca5c0725"
    "744600bf22fa05f606f001066f1e764222fa07f726f00f06ff0718bf0f3604f8"
    "016b012da5f10205ecd1013363450ef1040ee3d108e03ab1034600bf11f8015b"
    "013a03f8015bf9d1f0bd"
)

# (file_offset, expected_original, new_bytes, description)
PATCHES = [
    # --- 576x288 image-container size lift ---
    (g2f(0x501062), "bd f8 2c 10", "40 f2 41 20", "container width  <= 576"),
    (g2f(0x50112a), "bd f8 2e 00", "40 f2 21 11", "container height movw #0x121"),
    (g2f(0x50112e), "91 28",       "88 42",       "container height cmp r0,r1"),
    # --- inject frag_write over set_aging_test_info (production-test, unused) ---
    (g2f(0x491340), "ab f7 97 fe", FRAG_WRITE.hex(), "frag_write() decompress shim"),
    # --- retarget the 3 per-fragment memcpy calls -> frag_write ---
    (g2f(0x500984), "39 f7 2e f9", "90 f7 dc fc", "bl frag_write (first fragment)"),
    (g2f(0x500b60), "39 f7 40 f8", "90 f7 ee fb", "bl frag_write (new-stream restart)"),
    (g2f(0x500d7c), "38 f7 32 ff", "90 f7 e0 fa", "bl frag_write (append fragment)"),
    # --- allow per-fragment CompressMode to vary within one image session ---
    # The append path stashes fragment 0's CompressMode and rejects any later
    # fragment whose CompressMode differs (cmp at 0x500c0e). Make that branch
    # unconditional (beq->b) so a sender can mix a verbatim CompressMode=0 BMP
    # header fragment with CompressMode=1 1bpp pixel fragments in one stream.
    (g2f(0x500c10), "3b d0", "3b e0", "drop per-session CompressMode-consistency guard"),
    # --- zlib (DEFLATE) whole-image decompression at BMP-load time ---
    # Inject the glue blob into the dead set_aging_test_info tail, then redirect
    # the one BMP-loader call FUN_0050164a(state, reconBuf, len) in FUN_004ae69c
    # to load_image_z, which inflates a zlib stream (sent as a CompressMode=0
    # image) into a scratch BMP before loading. Raw BMPs pass straight through.
    (g2f(0x491400), "ab f7 21 fd", ZLIB_GLUE.hex(), "zlib glue (zwrap_alloc/free + load_image_z)"),
    (g2f(0x4ae9cc), "52 f0 3d fe", "e2 f7 25 fd", "bl load_image_z (decompress at BMP load)"),
    # --- CFW capability advertisement on the sid=0x09 settings READ response ---
    # Inject settings_send_wrapper into the dead tail (after the zlib glue), then
    # redirect the settings responder's `bl FUN_0047398c` (aa21 send) to it so it
    # appends protobuf field 100 ("EVENCFW/1 img576 imgz xordelta stereo") before
    # framing. Unknown high field tag -> stock app/bridge ignore it; CFW-aware
    # apps read it to detect the firmware and gate features.
    (g2f(0x4917f8), "34 6a 79 e0", SETTINGS_EXT.hex(), "settings_send_wrapper (CFW caps field)"),
    (g2f(0x4b43c4), "bf f7 e2 fa", "dd f7 18 fa", "bl settings_send_wrapper (append caps field 100)"),
]

def hx(s):
    return bytes.fromhex(s.replace(" ", ""))

def crc32c_msb(buf, _t=[]):
    if not _t:
        for b in range(256):
            c = b << 24
            for _ in range(8):
                c = ((c << 1) ^ 0x1edc6f41) & 0xffffffff if c & 0x80000000 else (c << 1) & 0xffffffff
            _t.append(c)
    crc = 0
    for byte in buf:
        crc = ((crc << 8) & 0xffffffff) ^ _t[((crc >> 24) ^ byte) & 0xff]
    return crc

def fixup_checksums(data):
    n = struct.unpack_from('<I', data, 8)[0]
    for i in range(n):
        eid, off, size, _ = struct.unpack_from('<IIII', data, 0x40 + i * 16)
        ps = struct.unpack_from('<I', data, off + 8)[0]
        name = bytes(data[off + 48:off + 128]).split(b'\0')[0].decode('latin1')
        if name.endswith('s200_firmware_ota.bin'):
            pre = zlib.crc32(bytes(data[off + 128 + 8:off + 128 + ps])) & 0xffffffff
            struct.pack_into('<I', data, off + 128 + 4, pre)
        crc = crc32c_msb(bytes(data[off + 128:off + 128 + ps]))
        struct.pack_into('<I', data, 0x40 + i * 16 + 12, crc)
        struct.pack_into('<I', data, off + 12, crc)
        extra = f", preamble crc32={pre:08x}" if name.endswith('s200_firmware_ota.bin') else ""
        print(f"  [{i}] {name}: component crc32c={crc:08x}{extra}")

def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "g2_2.2.4.34.bin"
    dst = sys.argv[2] if len(sys.argv) > 2 else "g2_2.2.4.34_cfw.bin"
    data = bytearray(open(src, "rb").read())
    for off, orig, new, desc in PATCHES:
        o, n = hx(orig), hx(new)  # orig is a prefix sanity-check; new may be longer (code blob)
        if bytes(data[off:off + len(n)]) == n:
            print(f"  {off:#x}: already patched ({desc})")
            continue
        cur = bytes(data[off:off + len(o)])
        assert cur == o, f"{off:#x} ({desc}): expected {o.hex()} got {cur.hex()}"
        data[off:off + len(n)] = n
        print(f"  {off:#x}: {desc} ({len(n)} B)")
    print("recomputing checksums:")
    fixup_checksums(data)
    open(dst, "wb").write(data)
    print(f"wrote {dst} ({len(data)} bytes)")

if __name__ == "__main__":
    main()
