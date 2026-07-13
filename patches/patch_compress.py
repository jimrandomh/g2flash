#!/usr/bin/env python3
"""
Build a CFW image for g2_2.2.4.34 with:
  (1) the 576x288 image-container size lift (same 3 edits as
      patches/patch_img_container_576.py),
  (2) 1bpp->4bpp image decompression on ImageRawDataUpdate.CompressMode, the zlib
      image glue (multi-mode load_image_z, incl. keepalive kick + buzzer),
  (3) a CFW capability-advertisement field (protobuf field 100) appended to the
      sid=0x09 settings READ response, and
  (4) EvenHub long-press + ring release-long-press forwarding.

PLACEMENT MODEL — APPEND, don't overwrite. The injected code blobs
(frag_write, zlib glue, settings wrapper, gesture_fwd) are APPENDED to
the tail of the main-app component (ota/s200_firmware_ota.bin) rather than being
squeezed into a reclaimed dead function. The bootloader XIP-programs the whole
main-app payload to 0x00438000, so a byte at payload offset K lands at MRAM
0x438000 + K - 0x20; appended blobs therefore load into MRAM immediately after the
current app image (~0x0078f188), with hundreds of KB of headroom before the OTA
flag at 0x007fe000. This removes the old ~2 KB dead-region ceiling.

Appending changes the image size, so this script fixes up every size/offset field
the container + bootloader read: the component's subheader payload size (ps), its
TOC entry size (ps + 128), the main-app preamble length field (preamble[0] low
24 bits — what the bootloader actually erases/programs), and then the checksums
(component CRC32C in the TOC + subheader echo, and the preamble zlib-CRC32). The
main app is the LAST component so appending shifts no downstream offsets.

Every `bl` that targets injected code is computed from (call-site, appended
address) so redirects can never drift; the zwrap_alloc/free fn-ptrs baked into
z_stream use absolute addresses filled in on a 2nd build pass. A hard MRAM-ceiling
check (duplicating g2flash.py's check_mainapp_fits_mram) refuses an oversized image.
"""
import sys, os, struct, zlib, json, subprocess

DELTA = 0x39E680  # file_off = ghidra_addr - DELTA  (OTA mainApp component)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

def g2f(addr):
    return addr - DELTA

# ---- main-app MRAM placement (mirrors g2flash.py check_mainapp_fits_mram) ----
MAINAPP       = "ota/s200_firmware_ota.bin"
APP_LOAD_ADDR = 0x00438000   # bootloader XIP-programs the main app here
APP_PREAMBLE  = 0x20         # it programs payload[0x20:], so payload[k] -> 0x438000 + k - 0x20
OTA_FLAG_ADDR = 0x007FE000   # OTA magic word (last 8 KB of MRAM)
MRAM_END      = 0x00800000
APP_MAX_END   = 0x007F0000   # conservative ceiling: leave the top ~56 KB for NV + flag
BLOB_ALIGN    = 4            # 4-byte-align each appended blob (Thumb literal pools)

def mram_addr(payload_off):
    """MRAM XIP address of the byte at this main-app payload offset, once flashed."""
    return APP_LOAD_ADDR + payload_off - APP_PREAMBLE

def align_up(x, a):
    return (x + a - 1) & ~(a - 1)

# ---- call-site redirects (ghidra addr -> stock bytes we expect there) --------
# The three per-fragment memcpy (FUN_00439be4) calls in the ImageRawDataUpdate
# handler FUN_004ff8fc, retargeted to frag_write.
FRAG_BL_SITES = {
    0x500984: "39 f7 2e f9",   # first fragment
    0x500b60: "39 f7 40 f8",   # new-stream restart
    0x500d7c: "38 f7 32 ff",   # append fragment
}
LOADBMP_BL_SITE        = (0x4ae9cc, "52 f0 3d fe")  # bl FUN_0050164a (deferred consumer) -> image_deferred
# The two both-lens `bl FUN_0045a8ec` (lens-identity check) sites at image-
# reconstruction-complete in FUN_004ff8fc (single- and multi-fragment). Redirected to
# snapshot_side, which copies the fresh recon buffer into a per-state FIFO (both
# lenses) then tail-calls the real FUN_0045a8ec so the RIGHT gate still works. This +
# image_deferred consuming the FIFO fixes the producer/consumer race on the shared
# recon buffer. See the snapshot/restore note in zlib_glue.c.
SNAPSHOT_BL_SITES      = {   # both decode to `bl 0x45a8ec` (verified)
    0x500a04: "59 f7 72 ff",   # single-fragment complete
    0x500df8: "59 f7 78 fd",   # multi-fragment last-fragment complete
}
SETTINGS_BL_SITE       = (0x4b43c4, "bf f7 e2 fa")  # bl FUN_0047398c (aa21 send) -> wrapper
GESTURE_LONGPRESS_SITE = (0x4425ae, "28 f0 49 f8")  # bl FUN_0046a644 -> evenhub_longpress
GESTURE_RELEASE_SITE   = (0x4428de, "1d f0 cf f9")  # bl FUN_0045fc80 -> ring_release

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
    raise SystemExit(f"{blob.get('src', '?')}: function {name!r} not found")

def find_mainapp(img):
    """Return (index, component_off, ps) for the ota/s200_firmware_ota.bin component."""
    n = struct.unpack_from('<I', img, 8)[0]
    for i in range(n):
        _eid, off, _size, _crc = struct.unpack_from('<IIII', img, 0x40 + i * 16)
        name = bytes(img[off + 48:off + 128]).split(b'\0')[0].decode('latin1')
        if name.endswith('s200_firmware_ota.bin'):
            ps = struct.unpack_from('<I', img, off + 8)[0]
            return i, off, ps
    raise SystemExit("main-app component (ota/s200_firmware_ota.bin) not found")

def layout(img):
    """Compile the injected blobs and lay them out at the tail of the main-app
    payload. Returns (append_bytes, in_place_patches, mainapp=(idx,off,old_ps)).
    Enforces the MRAM ceiling (duplicate of g2flash.check_mainapp_fits_mram)."""
    idx, comp_off, old_ps = find_mainapp(img)

    pieces = []          # (payload_off, blob_bytes) in append order
    cursor = align_up(old_ps, BLOB_ALIGN)

    def place(blob):
        nonlocal cursor
        off = align_up(cursor, BLOB_ALIGN)
        pieces.append((off, blob))
        cursor = off + len(blob)
        return off

    # 1) frag_write (extracted from decompress.c; entry at offset 0 of its bytes).
    # This is the one place we append a SINGLE function's bytes rather than the whole
    # blob, so it must carry no rodata -- a string literal here would be laid out
    # after .text by build.py and dropped by this per-function slice, leaving its
    # PC-relative refs dangling. Assert that; if decompress.c ever needs strings,
    # switch this to append the whole `dec["text"]` blob like the others.
    dec = build_blob("decompress.c")
    assert dec.get("rodata_len", 0) == 0, \
        "decompress.c now emits rodata; frag_write can't be extracted alone (see note)"
    frag = bytes.fromhex(_fn(dec, "frag_write")["bytes"])
    frag_off = place(frag)
    frag_addr = mram_addr(frag_off)

    # 2) zlib glue (2-pass: stable zwrap_alloc/free addrs baked into z_stream)
    glue_off = align_up(cursor, BLOB_ALIGN)
    glue_base = mram_addr(glue_off)
    p1 = build_blob("zlib_glue.c")
    alloc_addr = glue_base + _fn(p1, "zwrap_alloc")["offset"] + 1   # +1 = Thumb bit (blx via fn-ptr)
    free_addr  = glue_base + _fn(p1, "zwrap_free")["offset"] + 1
    seq_tick_addr = glue_base + _fn(p1, "seq_tick")["offset"] + 1   # buzzer-sequence osTimer callback
    p2 = build_blob("zlib_glue.c", [
        f"-DZWRAP_ALLOC_ADDR=0x{alloc_addr:x}",
        f"-DZWRAP_FREE_ADDR=0x{free_addr:x}",
        f"-DSEQ_TICK_ADDR=0x{seq_tick_addr:x}",
    ])
    glue = bytes.fromhex(p2["text"])
    assert place(glue) == glue_off, "glue placement drifted between size calc and layout"
    assert glue_base + _fn(p2, "seq_tick")["offset"] + 1 == seq_tick_addr, \
        "seq_tick offset moved between build passes (reordered?); baked SEQ_TICK_ADDR would be wrong"
    snapshot_addr = glue_base + _fn(p2, "snapshot_side")["offset"]   # both-lens snapshot hook
    deferred_addr = glue_base + _fn(p2, "image_deferred")["offset"]  # deferred consumer (FIFO restore)

    # 3) settings_send_wrapper (whole .text; entry at offset 0)
    se = build_blob("settings_ext.c")
    settings = bytes.fromhex(se["text"])
    settings_off = place(settings)
    settings_addr = mram_addr(settings_off)

    # 4) gesture_fwd (whole .text; two entry points)
    gf = build_blob("gesture_fwd.c")
    gesture = bytes.fromhex(gf["text"])
    gesture_off = place(gesture)
    gesture_base = mram_addr(gesture_off)
    longpress_addr = gesture_base + _fn(gf, "evenhub_longpress")["offset"]
    release_addr   = gesture_base + _fn(gf, "ring_release")["offset"]

    # --- assemble the appended payload bytes (old_ps .. cursor) ---
    end_off = cursor
    append = bytearray(end_off - old_ps)
    for off, blob in pieces:
        append[off - old_ps:off - old_ps + len(blob)] = blob

    # --- MRAM ceiling check (duplicate of g2flash.check_mainapp_fits_mram) ---
    prog_end = mram_addr(end_off)   # exclusive MRAM end once flashed
    print("  append layout:")
    for name, off, sz in (("frag_write", frag_off, len(frag)),
                          ("zlib glue", glue_off, len(glue)), ("settings", settings_off, len(settings)),
                          ("gesture_fwd", gesture_off, len(gesture))):
        print(f"    {name:<12} @ MRAM 0x{mram_addr(off):08x}  +{sz} B")
    if prog_end > APP_MAX_END:
        over = prog_end - APP_MAX_END
        raise SystemExit(
            f"appended image is too large: programmed region ends at 0x{prog_end:08x}, "
            f"{over} B ({over / 1024:.1f} KB) past the safe ceiling 0x{APP_MAX_END:08x}. "
            f"MRAM app window is 0x{APP_LOAD_ADDR:08x}..0x{OTA_FLAG_ADDR:08x} (OTA flag); "
            f"end of MRAM is 0x{MRAM_END:08x}. The bootloader does NOT bounds-check this, "
            "so flashing would risk clobbering the OTA flag / NV or bricking the lens "
            "(SWD-only recovery). Reduce the injected code.")
    print(f"    appended {len(append)} B -> payload end MRAM 0x{prog_end:08x} "
          f"({(APP_MAX_END - prog_end) // 1024} KB under 0x{APP_MAX_END:08x})")

    # --- in-place live-code edits + bl retargets (targets are the appended addrs) ---
    in_place = [
        # 576x288 image-container size lift
        (g2f(0x501062), "bd f8 2c 10", "40 f2 41 20", "container width  <= 576"),
        (g2f(0x50112a), "bd f8 2e 00", "40 f2 21 11", "container height movw #0x121"),
        (g2f(0x50112e), "91 28",       "88 42",       "container height cmp r0,r1"),
        # retarget the 3 per-fragment memcpy calls -> frag_write
        *[(g2f(site), orig, enc_bl(site, frag_addr), f"bl frag_write @ {site:#x}")
          for site, orig in FRAG_BL_SITES.items()],
        # allow per-fragment CompressMode to vary within one image session
        (g2f(0x500c10), "3b d0", "3b e0", "drop per-session CompressMode-consistency guard"),
        # Snapshot/restore (fixes the shared-recon-buffer producer/consumer race): at the
        # both-lens completion, redirect `bl FUN_0045a8ec` -> snapshot_side (copies the fresh
        # message into a per-state FIFO, then returns the lens id); the deferred consumer
        # `bl FUN_0050164a` -> image_deferred (pops the FIFO and runs the worker on the
        # snapshot, ignoring the possibly-overwritten live buffer).
        *[(g2f(site), orig, enc_bl(site, snapshot_addr), f"bl snapshot_side @ {site:#x}")
          for site, orig in SNAPSHOT_BL_SITES.items()],
        (g2f(LOADBMP_BL_SITE[0]), LOADBMP_BL_SITE[1], enc_bl(LOADBMP_BL_SITE[0], deferred_addr),
         "bl image_deferred (deferred consumer -> FIFO restore + worker, both lenses)"),
        # redirect the settings responder send -> settings_send_wrapper (caps field 100)
        (g2f(SETTINGS_BL_SITE[0]), SETTINGS_BL_SITE[1], enc_bl(SETTINGS_BL_SITE[0], settings_addr),
         "bl settings_send_wrapper (append caps field 100)"),
        # EvenHub long-press + ring release-long-press forwarding
        (g2f(GESTURE_LONGPRESS_SITE[0]), GESTURE_LONGPRESS_SITE[1],
         enc_bl(GESTURE_LONGPRESS_SITE[0], longpress_addr), "bl evenhub_longpress (replaces force-quit dialog)"),
        (g2f(GESTURE_RELEASE_SITE[0]), GESTURE_RELEASE_SITE[1],
         enc_bl(GESTURE_RELEASE_SITE[0], release_addr), "bl ring_release (forward ring release-long-press)"),
    ]
    return bytes(append), in_place, (idx, comp_off, old_ps)

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

def build_patch_ops(img):
    """Compile the injected blobs (needs clang) and return (patched_data, ops).

    `ops` is the clang-free description of the whole transform: a list of
    {offset, old (hex), new (hex), desc} entries that, applied to the stock
    image, reproduce `patched_data` byte-for-byte. `old` records the stock bytes
    at each site (empty for the tail append) so the applier can sanity-check it
    is operating on the right base. This list is what gen_patches.py serializes
    to patches/cfw_patches.json for apply_patches.py to consume without clang.

    Only offsets whose bytes actually change are recorded, so the per-component
    checksum fixups collapse to just the (changed) main-app component."""
    append, in_place, (idx, comp_off, old_ps) = layout(img)

    data = bytearray(img)
    ops = []

    def record(off, newb, desc):
        """Stage a write of `newb` at `off`, recording the ORIGINAL bytes as the
        expected-old. Skips no-op writes (new == already-there) so unchanged
        checksums don't clutter the patch set. All recorded sites live in the
        image header/code, untouched by the append, so img[off] == data[off]."""
        newb = bytes(newb)
        old = bytes(img[off:off + len(newb)])
        if newb == old:
            return
        ops.append({"offset": off, "old": old.hex(), "new": newb.hex(), "desc": desc})
        data[off:off + len(newb)] = newb

    # 1) live-code edits + bl retargets. `orig` is a stock-bytes sanity prefix.
    print("applying in-place edits:")
    for off, orig, new, desc in in_place:
        o, n = hx(orig), hx(new)
        cur = bytes(data[off:off + len(o)])
        assert cur == o, f"{off:#x} ({desc}): expected {o.hex()} got {cur.hex()} (run against the STOCK image)"
        record(off, n, desc)
        print(f"  {off:#x}: {desc} ({len(n)} B)")

    # 2) append the injected blobs to the main-app payload. The main app is the
    #    last component, so its payload ends at EOF and appending shifts nothing.
    payload_end = comp_off + 128 + old_ps
    assert payload_end == len(data), (
        f"main-app payload ends at 0x{payload_end:x} but file is 0x{len(data):x}; the append "
        "model assumes ota/s200_firmware_ota.bin is the last component")
    ops.append({"offset": payload_end, "old": "", "new": bytes(append).hex(),
                "desc": "append injected blobs to main-app payload"})
    data.extend(append)
    new_ps = old_ps + len(append)

    # 3) fix up the size/offset metadata the container + bootloader read
    record(comp_off + 8, struct.pack('<I', new_ps), "main-app subheader payload size (ps)")
    record(0x40 + idx * 16 + 8, struct.pack('<I', new_ps + 128), "main-app TOC entry size (ps + 128)")
    pre0 = struct.unpack_from('<I', data, comp_off + 128)[0]
    record(comp_off + 128,                                             # preamble length (low 24 bits)
           struct.pack('<I', (pre0 & 0xff000000) | (new_ps & 0xffffff)),
           "main-app preamble length (low 24 bits)")
    print(f"  appended {len(append)} B: ps {old_ps} -> {new_ps}, "
          f"preamble len -> 0x{new_ps & 0xffffff:x}, load addr 0x{APP_LOAD_ADDR:08x}")

    # 4) recompute checksums over the new payload (preamble crc32 first, then crc32c)
    print("recomputing checksums:")
    n = struct.unpack_from('<I', data, 8)[0]
    for i in range(n):
        eid, off, size, _ = struct.unpack_from('<IIII', data, 0x40 + i * 16)
        ps = struct.unpack_from('<I', data, off + 8)[0]
        name = bytes(data[off + 48:off + 128]).split(b'\0')[0].decode('latin1')
        pre = None
        if name.endswith('s200_firmware_ota.bin'):
            pre = zlib.crc32(bytes(data[off + 128 + 8:off + 128 + ps])) & 0xffffffff
            record(off + 128 + 4, struct.pack('<I', pre), f"[{i}] {name} preamble crc32")
        crc = crc32c_msb(bytes(data[off + 128:off + 128 + ps]))
        record(0x40 + i * 16 + 12, struct.pack('<I', crc), f"[{i}] {name} component crc32c (TOC)")
        record(off + 12, struct.pack('<I', crc), f"[{i}] {name} component crc32c (subheader)")
        if pre is not None or crc32c_msb(bytes(img[off + 128:off + 128 + ps])) != crc:
            extra = f", preamble crc32={pre:08x}" if pre is not None else ""
            print(f"  [{i}] {name}: component crc32c={crc:08x}{extra}")

    return bytes(data), ops

def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "g2_2.2.4.34.bin"
    dst = sys.argv[2] if len(sys.argv) > 2 else "g2_2.2.4.34_cfw.bin"
    print("compiling injected blobs (build.py):")
    img = open(src, "rb").read()
    data, ops = build_patch_ops(img)

    # Prove the clang-free op list reproduces the compiled image exactly, so the
    # patches/cfw_patches.json that gen_patches.py emits from `ops` is faithful.
    from apply_patches import apply_ops
    assert apply_ops(img, ops) == data, "op list does not reproduce the compiled image"

    open(dst, "wb").write(data)
    print(f"wrote {dst} ({len(data)} bytes)")

if __name__ == "__main__":
    sys.path.insert(0, SCRIPT_DIR)
    main()
