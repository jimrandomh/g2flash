#!/usr/bin/env python3
"""
Build a CFW image for g2_2.2.4.34 with:
  (1) the 576x288 image-container size lift (same 3 edits as
      patches/patch_img_container_576.py),
  (2) 1bpp->4bpp image decompression on ImageRawDataUpdate.CompressMode, the zlib
      image glue (multi-mode load_image_z, incl. keepalive kick + buzzer), and
  (3) a CFW capability-advertisement field (protobuf field 100, a feature-token
      string) appended to the sid=0x09 settings READ response, so a connected
      app can detect this firmware and which extensions it supports.

The injected machine code is NOT pasted in as hex here: each blob is compiled
on demand by build.py (patches/{decompress,zlib_glue,settings_ext}.c) and pulled
in as bytes via its --json output. The three blobs live in a reclaimed dead
region (the production-test handler set_aging_test_info + the dead
FUN_00491570/FUN_004919ec bodies); make_patches() places them at fixed offsets,
enforces that none exceeds its slot (max-length checks against the next blob /
the first live function after the region), and computes every `bl` that targets
injected code from (call-site, placement) so the redirects can never drift out
of sync with the blob sizes.

Length-preserving; recomputes the EVENOTA component CRC32C (TOC + sub-header echo)
and the mainApp preamble zlib-CRC32, exactly like patch_img_container_576.py.
"""
import sys, os, struct, zlib, json, subprocess

DELTA = 0x39E680  # file_off = ghidra_addr - DELTA  (OTA mainApp component)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def g2f(addr):
    return addr - DELTA

# ---- injectable dead region (ghidra addresses) -----------------------------
# set_aging_test_info (production test, zero xrefs) + the dead FUN_00491570 /
# FUN_004919ec bodies. FRAG/GLUE/SETTINGS are placed at fixed starts; the first
# LIVE function after the region is FUN_00491cd0, which nothing may reach.
FRAG_ADDR      = 0x491340   # frag_write() 1bpp->4bpp fragment shim
GLUE_ADDR      = 0x491400   # zlib glue: zwrap_alloc/free + load_image_z + helpers
SETTINGS_ADDR  = 0x491ab0   # settings_send_wrapper (after the glue, 64 B headroom)
INJECT_CEILING = 0x491cd0   # FUN_00491cd0 = first live code after the dead region

# Max length for each compiled blob = the space up to the next blob / the ceiling.
# If a blob outgrows its slot, the assert in make_patches() fires with a clear
# message instead of silently clobbering the following blob or live firmware.
FRAG_MAX     = GLUE_ADDR - FRAG_ADDR          # 0x0c0 = 192 B
GLUE_MAX     = SETTINGS_ADDR - GLUE_ADDR      # 0x572 = 1394 B (currently full)
SETTINGS_MAX = INJECT_CEILING - SETTINGS_ADDR # 0x35e = 862 B

# ---- call-site redirects (ghidra addr -> stock bytes we expect there) -------
# The three per-fragment memcpy (FUN_00439be4) calls in the ImageRawDataUpdate
# handler FUN_004ff8fc, retargeted to frag_write.
FRAG_BL_SITES = {
    0x500984: "39 f7 2e f9",   # first fragment
    0x500b60: "39 f7 40 f8",   # new-stream restart
    0x500d7c: "38 f7 32 ff",   # append fragment
}
LOADBMP_BL_SITE  = (0x4ae9cc, "52 f0 3d fe")  # bl FUN_0050164a -> load_image_z
SETTINGS_BL_SITE = (0x4b43c4, "bf f7 e2 fa")  # bl FUN_0047398c (aa21 send) -> wrapper

def enc_bl(pc, target):
    """Encode a Thumb-2 BL (T1) from instruction address `pc` to `target`."""
    off = target - (pc + 4)
    assert off % 2 == 0, f"BL target {target:#x} not halfword-aligned from {pc:#x}"
    assert -(1 << 24) <= off < (1 << 24), f"BL {pc:#x}->{target:#x} out of +-16MB range"
    imm = (off >> 1) & 0xFFFFFF
    S = (imm >> 23) & 1
    i1 = (imm >> 22) & 1
    i2 = (imm >> 21) & 1
    imm10 = (imm >> 11) & 0x3FF
    imm11 = imm & 0x7FF
    j1 = (~(i1 ^ S)) & 1
    j2 = (~(i2 ^ S)) & 1
    hw1 = 0xF000 | (S << 10) | imm10
    hw2 = 0xD000 | (j1 << 13) | (j2 << 11) | imm11
    return bytes([hw1 & 0xFF, hw1 >> 8, hw2 & 0xFF, hw2 >> 8]).hex()

def build_blob(src, defines=()):
    """Compile patches/<src> via build.py --json and return the parsed dict
    ({text, text_len, functions:[{name,offset,size,bytes}]})."""
    cmd = ["python3", os.path.join(SCRIPT_DIR, "build.py"),
           os.path.join(SCRIPT_DIR, src), "--json", *defines]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"build.py failed for {src}:\n{r.stderr or r.stdout}")
    return json.loads(r.stdout)

def _fn(blob, name):
    for f in blob["functions"]:
        if f["name"] == name:
            return f
    raise SystemExit(f"{blob['src']}: function {name!r} not found")

def make_patches():
    """Compile the three injected blobs, sanity-check their sizes against their
    slots, and return the full (file_offset, expected_original, new_bytes, desc)
    patch list -- injected bytes and all `bl` redirects sourced from the build."""
    # --- frag_write: pull just that function out of decompress.c ---
    dec = build_blob("decompress.c")
    frag = bytes.fromhex(_fn(dec, "frag_write")["bytes"])

    # --- zlib glue: 2-pass. Pass 1 fixes the (placement-stable) zwrap_alloc/free
    #     offsets; pass 2 bakes their absolute Thumb addresses into z_stream so the
    #     zalloc/zfree fn-ptrs are relocation-free constants. ---
    p1 = build_blob("zlib_glue.c")
    alloc_addr = GLUE_ADDR + _fn(p1, "zwrap_alloc")["offset"] + 1  # +1 = Thumb bit
    free_addr  = GLUE_ADDR + _fn(p1, "zwrap_free")["offset"] + 1
    p2 = build_blob("zlib_glue.c", [f"-DZWRAP_ALLOC_ADDR=0x{alloc_addr:x}",
                                    f"-DZWRAP_FREE_ADDR=0x{free_addr:x}"])
    glue = bytes.fromhex(p2["text"])
    loadz_addr = GLUE_ADDR + _fn(p2, "load_image_z")["offset"]     # bl target (even)

    # --- settings wrapper: whole .text ---
    se = build_blob("settings_ext.c")
    settings = bytes.fromhex(se["text"])

    # --- max-length checks on all compiled outputs ---
    for name, blob, base, cap in (
        ("frag_write",            frag,     FRAG_ADDR,     FRAG_MAX),
        ("zlib glue",             glue,     GLUE_ADDR,     GLUE_MAX),
        ("settings_send_wrapper", settings, SETTINGS_ADDR, SETTINGS_MAX),
    ):
        end = base + len(blob)
        assert len(blob) <= cap, (
            f"{name} is {len(blob)} B but its slot at {base:#x} holds only {cap} B "
            f"(ends {end:#x}, next region starts {base + cap:#x}); relocate it / the "
            f"following blob and update the *_ADDR constants.")
        print(f"  layout: {name:<22} {base:#x} + {len(blob):>4} B "
              f"({len(blob)}/{cap}) -> {end:#x}")
    assert SETTINGS_ADDR + len(settings) <= INJECT_CEILING, "injection past ceiling"

    frag_site_patches = [
        (g2f(site), orig, enc_bl(site, FRAG_ADDR), f"bl frag_write @ {site:#x}")
        for site, orig in FRAG_BL_SITES.items()
    ]

    return [
        # --- 576x288 image-container size lift ---
        (g2f(0x501062), "bd f8 2c 10", "40 f2 41 20", "container width  <= 576"),
        (g2f(0x50112a), "bd f8 2e 00", "40 f2 21 11", "container height movw #0x121"),
        (g2f(0x50112e), "91 28",       "88 42",       "container height cmp r0,r1"),
        # --- inject frag_write over set_aging_test_info (production-test, unused) ---
        (g2f(FRAG_ADDR), "ab f7 97 fe", frag.hex(), "frag_write() decompress shim"),
        # --- retarget the 3 per-fragment memcpy calls -> frag_write ---
        *frag_site_patches,
        # --- allow per-fragment CompressMode to vary within one image session ---
        # The append path stashes fragment 0's CompressMode and rejects any later
        # fragment whose CompressMode differs (cmp at 0x500c0e). Make that branch
        # unconditional (beq->b) so a sender can mix a verbatim CompressMode=0 BMP
        # header fragment with CompressMode=1 1bpp pixel fragments in one stream.
        (g2f(0x500c10), "3b d0", "3b e0", "drop per-session CompressMode-consistency guard"),
        # --- zlib (DEFLATE) whole-image decompression at BMP-load time ---
        # Inject the glue blob, then redirect the one BMP-loader call
        # FUN_0050164a(state, reconBuf, len) in FUN_004ae69c to load_image_z, which
        # dispatches on the buffer's first byte ('B'/1/6 -> fast 4bpp BMP/frame;
        # 2 -> 8bpp full, 3 -> 4bpp bounding-box update, 4 -> stereo; 5 -> buzzer
        # sound) and kicks the EvenHub
        # keepalive on every image message. Raw BMPs still pass straight through.
        (g2f(GLUE_ADDR), "ab f7 21 fd", glue.hex(), "zlib glue (zwrap_alloc/free + load_image_z)"),
        (g2f(LOADBMP_BL_SITE[0]), LOADBMP_BL_SITE[1], enc_bl(LOADBMP_BL_SITE[0], loadz_addr),
         "bl load_image_z (decompress at BMP load)"),
        # --- CFW capability advertisement on the sid=0x09 settings READ response ---
        # Inject settings_send_wrapper into the dead tail (after the zlib glue), then
        # redirect the settings responder's `bl FUN_0047398c` (aa21 send) to it so it
        # appends protobuf field 100 ("EVENCFW/1 img576 imgz xordelta stereo") before
        # framing. Unknown high field tag -> stock app/bridge ignore it; CFW-aware
        # apps read it to detect the firmware and gate features.
        (g2f(SETTINGS_ADDR), "7e 49 03 20", settings.hex(), "settings_send_wrapper (CFW caps field)"),
        (g2f(SETTINGS_BL_SITE[0]), SETTINGS_BL_SITE[1], enc_bl(SETTINGS_BL_SITE[0], SETTINGS_ADDR),
         "bl settings_send_wrapper (append caps field 100)"),
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
    print("compiling injected blobs (build.py):")
    patches = make_patches()
    data = bytearray(open(src, "rb").read())
    for off, orig, new, desc in patches:
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
