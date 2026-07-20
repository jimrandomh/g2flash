#!/usr/bin/env python3
"""
Build a CFW image for g2_2.2.6.10 with:
  (1) the 576x288 image-container size lift (the same 3 edits that
      patches/patch_img_container_576.py makes, though that standalone script still
      targets the old 2.2.4.34 base and is not part of this build),
  (2) the zlib image glue (multi-mode load_image_z, incl. keepalive kick + buzzer),
      entered at image_deferred,
  (3) a CFW capability-advertisement field (protobuf field 100) appended to the
      sid=0x09 settings READ response, and
  (4) EvenHub long-press + ring release-long-press forwarding.

REBASED 2.2.4.34 -> 2.2.6.10 (2026-07-16). Every address below was re-derived and
cross-checked; see notes/fw-2.2.6.10-cfw-rebase.md for the full table and the evidence
for each. Two things bit us and are worth remembering if this is ever rebased again:
  * a patch site's offset within its host function is NOT stable -- Even inserts code, so
    each site was located by instruction-window match (firmware/find_site.py) and then
    confirmed by decoding its `bl` target, not by extrapolating from the function entry;
  * hardcoded RAM addresses all moved, with several DIFFERENT deltas, and some old
    addresses still exist in the new image as unrelated variables. They were re-derived
    through the instruction that loads them (firmware/map_ram.py).

The old CompressMode-based per-fragment expander (frag_write, patches/decompress.c) is
GONE as of this rebase: stock 2.2.6.10 defines CompressMode 1=RLE / 2=LZ4 for its own
image compression, which collided with our use of that field. Their implementation
benchmarks ~10 fps vs our zlib path's ~23 fps, so we ignore it and keep image_deferred,
which dispatches on the image's own leading bytes ('BM' vs a small u8 mode) and runs at a
later stage. See notes/fw-2.2.6.10-lz4-images.md.

PLACEMENT MODEL — APPEND, don't overwrite. The injected code blobs
(zlib glue, settings wrapper, gesture_fwd) are APPENDED to
the tail of the main-app component (ota/s200_firmware_ota.bin) rather than being
squeezed into a reclaimed dead function. The bootloader XIP-programs the whole
main-app payload to 0x00438000, so a byte at payload offset K lands at MRAM
0x438000 + K - 0x20; appended blobs therefore load into MRAM immediately after the
current app image (~0x00794324 on 2.2.6.10), with hundreds of KB of headroom before the
OTA flag at 0x007fe000. This removes the old ~2 KB dead-region ceiling.

Appending changes the image size, so this script fixes up every size/offset field
the container + bootloader read: the component's subheader payload size (ps), its
TOC entry size (ps + 128), the main-app preamble length field (preamble[0] low
24 bits — what the bootloader actually erases/programs), and then the checksums
(component CRC32C in the TOC + subheader echo, and the preamble zlib-CRC32). The
main app is the LAST component so appending shifts no downstream offsets.

Every `bl` that targets injected code is computed from (call-site, appended
address) so redirects can never drift; the injected code itself is fully position-
independent (see build.py) and needs no load address at build time, so it compiles
in a single pass. A hard MRAM-ceiling check (duplicating g2flash.py's
check_mainapp_fits_mram) refuses an oversized image.
"""
import sys, os, struct, zlib, json, subprocess

DELTA = 0x37A179  # file_off = ghidra_addr - DELTA  (OTA mainApp component, 2.2.6.10)
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
# All 2.2.6.10 addresses. Each was found with firmware/find_site.py (normalized
# instruction-window match, unique across the image) and then confirmed by decoding the
# `bl` at the new address and checking it lands on the expected callee -- the bytes below
# are the stock encodings read straight out of the image, so apply_patches' old-byte
# check is a third, independent guard.
#
# bl FUN_004dc5ae (set_image_data) in evenhub_ui_reflash_event_handler -> image_deferred.
# NOTE: in 2.2.6.10 this same function is where Even's own RLE/LZ4 decompression was
# inserted, immediately BEFORE this call. That is why the site moved by a different delta
# than the rest of the function. It is harmless for us: with CompressMode=0 (what we
# send) their block is a no-op passthrough, and the ABI here is unchanged
# (r0=obj, r1=data, r2=len; obj+0xc = data, obj+0x20 = len).
LOADBMP_BL_SITE        = (0x496a0e, "45 f0 ce fd")
# The two both-lens `bl FUN_0045a568` (lens-identity check) sites at image-
# reconstruction-complete in the EvenHub data parser (single- and multi-fragment).
# Redirected to snapshot_side, which copies the fresh recon buffer into a per-state FIFO
# (both lenses) then tail-calls the real lens-side fn so the RIGHT gate still works. This
# + image_deferred consuming the FIFO fixes the producer/consumer race on the shared
# recon buffer. See the snapshot/restore note in zlib_glue.c.
SNAPSHOT_BL_SITES      = {   # both decode to `bl 0x45a568` (verified)
    0x4db968: "7e f7 fe fd",   # single-fragment complete
    0x4dbd5c: "7e f7 04 fc",   # multi-fragment last-fragment complete
}
SETTINGS_BL_SITE       = (0x49bb68, "d9 f7 d4 ff")  # bl FUN_00475b14 (aa21 send) -> wrapper
GESTURE_LONGPRESS_SITE = (0x442e92, "28 f0 03 f8")  # bl FUN_0046ae9c -> evenhub_longpress
GESTURE_RELEASE_SITE   = (0x4431c2, "1c f0 9b fb")  # bl FUN_0045f8fc -> ring_release
# Wakeword ("Hey Even") capture. The GX8002 voice codec raises cmd_id 0x0c, which the
# audio thread (2.2.4.34 audio_recv_voice_event @ 0x54e47e) turns into the local Even AI
# wake action. That action does TWO independent things:
#   1. posts EvenAIDataPackage{command_id=CTRL, ctrl.status=EVEN_AI_WAKE_UP} to service 7,
#      which the phone sees as an `aa 21` notify on sid 0x07, and
#   2. calls even_ai_display_ctrl(START), which foregrounds the stock Even AI app over
#      whatever EvenHub app is running and draws the "Even AI listening" modal (then, with
#      no official phone app answering, errors out a few seconds later).
# We want (1) without (2), so faceclaw can own the interaction. This flips the op==START
# dispatch test at the head of even_ai_display_ctrl from `bne` to an unconditional `b`,
# so START falls through to the op==UPDATE/op==STOP compares (which fail for op 0) and the
# function just returns. UPDATE and STOP keep working, so the phone-driven exit path and
# the stock timeout teardown are both untouched. The notify in (1) is emitted earlier in
# the chain (2.2.4.34: FUN_004b0b54 -> FUN_005086e0) and is not affected.
#
# Both encodings are 2-byte Thumb to the same target (imm8 0x34 / imm11 0x034), so this is
# a pure in-place edit with no size change.
# Site found with firmware/find_site.py from the 2.2.4.34 address 0x506f44; the matcher
# reported two candidates and this one was confirmed by its `bl 0x45a568` (the
# independently-derived 2.2.6.10 lens_side), by the surrounding window matching
# 2.2.4.34's even_ai_display_ctrl head instruction-for-instruction, and by the following
# `movs r0,#0x67` being the same source line number (103) in both versions.
EVENAI_START_SITE      = (0x4e1fec, "34 d1")        # bne 0x4e2058 (op != START)
EVENAI_START_NEW       = "34 e0"                    # b   0x4e2058 (always skip START)

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

def build_blob(src):
    """Compile patches/<src> via build.py --json and return the parsed dict
    ({text, text_len, functions:[{name,offset,size,bytes}]})."""
    cmd = ["python3", os.path.join(SCRIPT_DIR, "build.py"),
           os.path.join(SCRIPT_DIR, src), "--json"]
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
    """Compile the single injected code blob (patches_main.c, which #includes every
    patch source) and append it at the tail of the main-app payload. Returns
    (append_bytes, in_place_patches, mainapp=(idx,off,old_ps)). Enforces the MRAM
    ceiling (duplicate of g2flash.check_mainapp_fits_mram)."""
    idx, comp_off, old_ps = find_mainapp(img)

    # Single combined blob: patches_main.c #includes all four patch sources, so build.py
    # emits ONE relocatable blob (its mini-linker resolves cross-file calls) that we
    # append once at the tail of the main-app payload. The blob needs no knowledge of its
    # own load address: injected code that takes the address of its own functions (the
    # z_stream zalloc/zfree pair, the seq_tick osTimer callback) does so with plain `&fn`,
    # which -fropi compiles to a PC-relative, relocation-free sequence. So we compile once
    # and each entry address here is just base + the function's offset in the one blob.
    blob_off = align_up(old_ps, BLOB_ALIGN)
    base = mram_addr(blob_off)
    built = build_blob("patches_main.c")
    blob = bytes.fromhex(built["text"])

    # injected entry points, resolved from the single blob's function table. These are all
    # `bl` targets, so they stay even -- a bl keeps the core in Thumb state and needs no
    # Thumb bit (unlike a fn-ptr consumed by blx, which the C code forms via `&fn`).
    snapshot_addr  = base + _fn(built, "snapshot_side")["offset"]
    deferred_addr  = base + _fn(built, "image_deferred")["offset"]
    settings_addr  = base + _fn(built, "settings_send_wrapper")["offset"]
    longpress_addr = base + _fn(built, "evenhub_longpress")["offset"]
    release_addr   = base + _fn(built, "ring_release")["offset"]

    # --- assemble the appended payload bytes (old_ps .. end) ---
    pad = blob_off - old_ps                     # alignment gap before the blob
    end_off = blob_off + len(blob)
    append = bytearray(end_off - old_ps)
    append[pad:pad + len(blob)] = blob

    # --- MRAM ceiling check (duplicate of g2flash.check_mainapp_fits_mram) ---
    prog_end = mram_addr(end_off)   # exclusive MRAM end once flashed
    rodata = built.get("rodata_len", 0)
    print(f"  combined blob @ MRAM 0x{base:08x}  +{len(blob)} B "
          f"(.text {built['text_len'] - rodata} + rodata {rodata})")
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
        # 576x288 image-container size lift, in common_image_create. Even did NOT raise
        # this cap in 2.2.6.10 (its clamp strings are byte-identical and the limit is
        # still parameterized), so the lift is still needed. These three sites are
        # byte-for-byte the same instructions as on 2.2.4.34, just relocated.
        (g2f(0x4dbfc6), "bd f8 2c 10", "40 f2 41 20", "container width  <= 576"),
        (g2f(0x4dc08e), "bd f8 2e 00", "40 f2 21 11", "container height movw #0x121"),
        (g2f(0x4dc092), "91 28",       "88 42",       "container height cmp r0,r1"),
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
        (g2f(EVENAI_START_SITE[0]), EVENAI_START_SITE[1], EVENAI_START_NEW,
         "even_ai_display_ctrl: skip Even AI app START (wakeword -> phone)"),
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
