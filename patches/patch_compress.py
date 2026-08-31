#!/usr/bin/env python3
"""
Build a CFW image for g2_2.2.9.22 with:
  (1) the 576x288 image-container size lift (the same 3 edits that
      the earlier standalone image-container patch used),
  (2) the zlib image glue (multi-mode load_image_z, incl. keepalive kick + buzzer),
      entered at image_deferred,
  (3) a CFW capability-advertisement field (protobuf field 100) plus a private,
      fail-open Faceclaw wake-ownership lease on sid=0x09,
  (4) conditional idle-double-tap dashboard deferral and conditional stock
      Even-AI suppression while that lease is valid, and
  (5) Faceclaw-lease-gated EvenHub long-press + ring release-long-press forwarding, and
  (6) a full-panel 640x480 packed-4bpp shadow copied directly into the physical
      framebuffer, and
  (7) stock wear-state notifications outside onboarding plus a current-state query, and
  (8) Faceclaw compass forwarding from the global sensor display event to the stock
      navigation BLE notifier while image-handler mode 10 is enabled, and
  (9) a lease-scoped 64 KiB texture cache plus cached-image/cached-string drawing
      through image-handler modes 12, 13, and 14, and built-in-font mode 15.

REBASED 2.2.6.10 -> 2.2.9.22 (2026-08-22). Every address below was re-derived with
normalized function/site matching and checked against the 2.2.9.22 disassembly. Two
changed hosts needed semantic rebases: image completion moved into a shared helper, and
plain long-press no longer calls the old force-quit dialog. Two things are worth
remembering if this is ever rebased again:
  * a patch site's offset within its host function is NOT stable -- Even inserts code, so
    each site was located by instruction-window match (firmware/find_site.py) and then
    confirmed by decoding its `bl` target, not by extrapolating from the function entry;
  * hardcoded RAM addresses all moved, with several DIFFERENT deltas, and some old
    addresses still exist in the new image as unrelated variables. They were re-derived
    through the instruction that loads them (firmware/map_ram.py).

PLACEMENT MODEL — APPEND, don't overwrite. The injected code blobs
(zlib glue, settings wrapper, gesture_fwd) are APPENDED to
the tail of the main-app component (ota/s200_firmware_ota.bin) rather than being
squeezed into a reclaimed dead function. The bootloader XIP-programs the whole
main-app payload to 0x00438000, so a byte at payload offset K lands at MRAM
0x438000 + K - 0x20; appended blobs therefore load into MRAM immediately after the
current app image (~0x007bea64 on 2.2.9.22), with hundreds of KB of headroom before the
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

DELTA = 0x379BFE  # file_off = ghidra_addr - DELTA  (OTA mainApp component, 2.2.9.22)
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

# Reserve the final 1 KiB of the stock primary TLSF arena for CFW-owned fixed
# state. Stock initializes [0x202728a8,0x2029f8a8) with size 0x2d000 at
# 0x0048c350. 0x2cc00 is the closest smaller value encodable by the existing
# four-byte Thumb modified-immediate instruction, leaving
# [0x2029f4a8,0x2029f8a8) outside the allocator. The next stock object starts at
# exactly 0x2029f8a8. CFW_CTX_SLOT in cfw_context.h uses the first reserved word.
PRIMARY_TLSF_SIZE_SITE = (0x48c350, "5f f4 34 32")  # movs.w r2,#0x2d000
PRIMARY_TLSF_CFW_SIZE  = "5f f4 33 32"              # movs.w r2,#0x2cc00
PRIMARY_TLSF_ARENA     = 0x202728a8
PRIMARY_TLSF_STOCK_LEN = 0x2d000
PRIMARY_TLSF_CFW_LEN   = 0x2cc00
CFW_RESERVED_BASE      = PRIMARY_TLSF_ARENA + PRIMARY_TLSF_CFW_LEN
CFW_RESERVED_END       = PRIMARY_TLSF_ARENA + PRIMARY_TLSF_STOCK_LEN

# A bytewise scan for absolute pointers into the reserved tail also interprets every
# four-byte Thumb instruction as a little-endian integer. In 2.2.9.22 these reviewed
# instruction sites happen to spell values in [CFW_RESERVED_BASE, CFW_RESERVED_END):
# mostly `ldr/str ..., [rN, r9, lsl #2]` or `[rN,#0x29]`. Keep the conservative
# every-alignment scan, but exempt only these exact decoded instruction bytes/sites.
TAIL_REF_FALSE_POSITIVES = {
    0x5200bc: "81 f8 29 20", 0x5201e4: "81 f8 29 20", 0x52037a: "81 f8 29 20",
    0x547cc2: "94 f8 29 20", 0x547d0e: "94 f8 29 20", 0x562b44: "40 f6 29 20",
    0x580c98: "80 f8 29 20", 0x58199a: "90 f8 29 20", 0x5c2f04: "56 f8 29 20",
    0x5dd848: "50 f8 29 20", 0x60ccda: "58 f8 29 20", 0x60ccfc: "56 f8 29 20",
    0x60cd04: "56 f8 29 20", 0x60cd42: "56 f8 29 20", 0x60cd68: "52 f8 29 20",
    0x60cd82: "52 f8 29 20", 0x60cd8a: "56 f8 29 20", 0x60cda4: "57 f8 29 20",
    0x60cdb2: "52 f8 29 20", 0x612e1c: "51 f8 29 20", 0x612e22: "41 f8 29 20",
}

def mram_addr(payload_off):
    """MRAM XIP address of the byte at this main-app payload offset, once flashed."""
    return APP_LOAD_ADDR + payload_off - APP_PREAMBLE

def align_up(x, a):
    return (x + a - 1) & ~(a - 1)

# ---- call-site redirects (ghidra addr -> stock bytes we expect there) --------
# All 2.2.9.22 addresses. Unchanged hosts/sites were found with normalized
# instruction-window match, unique across the image) and then confirmed by decoding the
# `bl` at the new address and checking it lands on the expected callee -- the bytes below
# are the stock encodings read straight out of the image, so apply_patches' old-byte
# check is a third, independent guard.
#
# bl FUN_004ee3ba (set_image_data) in evenhub_ui_reflash_event_handler -> image_deferred.
# NOTE: this same function is where Even's own RLE/LZ4 decompression runs
# inserted, immediately BEFORE this call. That is why the site moved by a different delta
# than the rest of the function. image_deferred dispatches CompressMode=0 through the CFW
# snapshot FIFO, but sends every nonzero CompressMode through the decompressed `r1,r2`
# buffer and exact stock loader. The ABI here is unchanged
# (r0=obj, r1=data, r2=len; obj+0xc = compressed data, obj+0x20 = compressed len).
LOADBMP_BL_SITE        = (0x4a4402, "49 f0 da ff")
# 2.2.9.22 funnels single- and multi-fragment image completion through one shared helper
# (FUN_004ec088). Redirect its `bl FUN_0045cfdc` lens-identity check to snapshot_side;
# r4 is the reconstruction state and r6 the container id at this site. The wrapper copies
# the fresh message into a per-state FIFO, then tail-calls the real lens-side function so
# the RIGHT gate still works. This + image_deferred consuming the FIFO fixes the live-
# recon-buffer producer/consumer race for both completion paths with one patch.
SNAPSHOT_BL_SITE       = (0x4ec0ee, "70 f7 75 ff")
SETTINGS_BL_SITE       = (0x4a90e4, "d4 f7 90 fb")  # bl FUN_0047d808 (aa21 send) -> wrapper
# nanopb decode in pb_service_setting's inbound parser. The wrapper scans raw
# unknown field 101 before the stock decoder discards it, then tail-calls decode.
SETTINGS_DECODE_BL_SITE = (0x4a87e4, "f5 f7 10 f9") # bl FUN_0049da08 -> settings_decode_wrapper
# The two REQUEST_DISPLAY_START_UP(1) sites reached by the local and mirrored
# idle double-tap paths. Both must defer or the peer lens can still flash.
DISPLAY_START_BL_SITES = {
    0x45f146: "0b f0 2a f9",
    0x45f206: "0b f0 ca f8",
}
# 2.2.9.22 changed the Menu interaction to tap-then-long-press and plain subtype 3 now
# posts UI event 8 rather than directly opening the force-quit dialog. Wrap that stock
# post, passing all non-Faceclaw/non-ring cases through unchanged. Subtype 0xe release
# still posts UI event 0x4a. r6 holds the current input record at both call sites.
GESTURE_PRESS_SITE     = (0x444a3a, "1d f0 57 fc")  # bl FUN_004622ec -> ring_press
GESTURE_RELEASE_SITE   = (0x444d36, "1d f0 d9 fa")  # bl FUN_004622ec -> ring_release
# Wakeword ("Hey Even") capture. The old patch unconditionally changed the
# op==START branch in even_ai_display_ctrl, which also broke the official Even
# app. Replace the first four bytes with a B.W trampoline: the injected entry
# reproduces the overwritten push/mov and suppresses START only under Faceclaw's
# volatile lease; with no lease it resumes at 0x4f515a byte-for-byte stock.
EVENAI_ENTRY_SITE      = (0x4f5156, "7f b5 06 00")
# The display task copies the composed 576x288 A4 buffer into the physical
# 640x480 framebuffer at two switch cases. Redirect both calls through
# display_copy_hook: ordinary refreshes pass through, while a pending Faceclaw
# shadow replaces the stock compositor copy immediately before panel refresh.
DISPLAY_COPY_BL_SITES = {
    0x4798f2: "f6 f7 ed ff",   # queue message type 3 -> bl FUN_004708d0
    0x479a2e: "f6 f7 4f ff",   # queue message type 6 -> bl FUN_004708d0
}
# The stock wear handler calls its onboarding-only transmitter in both branches.
# Redirect those calls to our lifecycle-independent sender instead.
WEAR_NOTIFY_BL_SITES = {
    0x4ac3ea: "d9 f7 58 ff",  # ON_HEAD:  bl 0x48629e
    0x4ac44e: "d9 f7 26 ff",  # OFF_HEAD: bl 0x48629e
}
# Global display-thread routing of IMU sensor event 9 as UI event 0x41. Navigation's
# UI handler normally receives this and calls the BLE compass notifier; Faceclaw has
# EvenHub active instead, so redirect through a wrapper that preserves the stock call
# and additionally invokes that notifier while mode 10 owns the compass.
COMPASS_EVENT_BL_SITE = (0x444dfc, "1d f0 76 fa")  # bl FUN_004622ec(display,0x41,&heading)

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

def enc_bw(pc, target):
    """Encode an unconditional Thumb-2 B.W (T4)."""
    off = target - (pc + 4)
    assert off % 2 == 0, f"B.W target {target:#x} not halfword-aligned from {pc:#x}"
    assert -(1 << 24) <= off < (1 << 24), f"B.W {pc:#x}->{target:#x} out of +-16MB range"
    imm = (off >> 1) & 0xFFFFFF
    S = (imm >> 23) & 1
    i1 = (imm >> 22) & 1
    i2 = (imm >> 21) & 1
    imm10 = (imm >> 11) & 0x3FF
    imm11 = imm & 0x7FF
    j1 = (~(i1 ^ S)) & 1
    j2 = (~(i2 ^ S)) & 1
    hw1 = 0xF000 | (S << 10) | imm10
    hw2 = 0x9000 | (j1 << 13) | (j2 << 11) | imm11
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

    # This reservation is safe only if the stock image has no absolute pointer
    # into the removed tail. Scan every byte alignment because the OTA container's
    # file-to-MRAM bias is not word-aligned. The allocator's original exclusive
    # end (0x2029f8a8) is intentionally outside the rejected interval and is the
    # base of the next stock object.
    tail_refs = []
    false_hits = set()
    for off in range(len(img) - 3):
        if not CFW_RESERVED_BASE <= struct.unpack_from('<I', img, off)[0] < CFW_RESERVED_END:
            continue
        site = off + DELTA
        expected = TAIL_REF_FALSE_POSITIVES.get(site)
        if expected is not None and bytes(img[off:off + 4]) == bytes.fromhex(expected):
            false_hits.add(site)
        else:
            tail_refs.append(off)
    assert false_hits == set(TAIL_REF_FALSE_POSITIVES), (
        "reviewed TLSF-tail false-positive instruction set changed: "
        f"missing {[hex(x) for x in set(TAIL_REF_FALSE_POSITIVES) - false_hits]}"
    )
    assert not tail_refs, (
        "stock image contains absolute references into the proposed CFW-reserved "
        f"TLSF tail: {[hex(off) for off in tail_refs]}"
    )

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
    settings_decode_addr = base + _fn(built, "settings_decode_wrapper")["offset"]
    display_start_addr = base + _fn(built, "faceclaw_display_start")["offset"]
    evenai_entry_addr = base + _fn(built, "faceclaw_evenai_display_entry")["offset"]
    press_addr     = base + _fn(built, "ring_press")["offset"]
    release_addr   = base + _fn(built, "ring_release")["offset"]
    display_copy_addr = base + _fn(built, "display_copy_hook")["offset"]
    wear_notify_addr = base + _fn(built, "faceclaw_send_wear_event")["offset"]
    compass_event_addr = base + _fn(built, "compass_event_forward")["offset"]

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
        (g2f(PRIMARY_TLSF_SIZE_SITE[0]), PRIMARY_TLSF_SIZE_SITE[1],
         PRIMARY_TLSF_CFW_SIZE,
         "reserve final 1 KiB of primary TLSF arena for CFW context anchor"),
        # 576x288 image-container size lift, in common_image_create. Even did NOT raise
        # this cap in 2.2.9.22 (its clamp strings are byte-identical and the limit is
        # still parameterized), so the lift is still needed. These three sites are
        # byte-for-byte the same instructions as on 2.2.4.34, just relocated.
        (g2f(0x4eddd2), "bd f8 2c 10", "40 f2 41 20", "container width  <= 576"),
        (g2f(0x4ede9a), "bd f8 2e 00", "40 f2 21 11", "container height movw #0x121"),
        (g2f(0x4ede9e), "91 28",       "88 42",       "container height cmp r0,r1"),
        # Snapshot/restore (fixes the shared-recon-buffer producer/consumer race): at the
        # both-lens completion, redirect `bl FUN_0045cfdc` -> snapshot_side (copies the
        # fresh message into a FIFO in the recon-buffer tail, then returns the lens id);
        # the deferred consumer `bl FUN_004ee3ba` -> image_deferred (pops the FIFO and
        # runs the worker on the snapshot, ignoring the possibly-overwritten live buffer).
        (g2f(SNAPSHOT_BL_SITE[0]), SNAPSHOT_BL_SITE[1],
         enc_bl(SNAPSHOT_BL_SITE[0], snapshot_addr),
         f"bl snapshot_side @ {SNAPSHOT_BL_SITE[0]:#x} (shared image-complete helper)"),
        (g2f(LOADBMP_BL_SITE[0]), LOADBMP_BL_SITE[1], enc_bl(LOADBMP_BL_SITE[0], deferred_addr),
         "bl image_deferred (deferred consumer -> FIFO restore + worker, both lenses)"),
        # redirect the settings responder send -> settings_send_wrapper (caps field 100)
        (g2f(SETTINGS_BL_SITE[0]), SETTINGS_BL_SITE[1], enc_bl(SETTINGS_BL_SITE[0], settings_addr),
         "bl settings_send_wrapper (append caps field 100)"),
        (g2f(SETTINGS_DECODE_BL_SITE[0]), SETTINGS_DECODE_BL_SITE[1],
         enc_bl(SETTINGS_DECODE_BL_SITE[0], settings_decode_addr),
         "bl settings_decode_wrapper (Faceclaw lease field 101)"),
        *[(g2f(site), orig, enc_bl(site, display_start_addr),
           f"bl faceclaw_display_start @ {site:#x} (fail-open double-tap takeover)")
          for site, orig in DISPLAY_START_BL_SITES.items()],
        # R1 long-press + release-long-press forwarding
        (g2f(GESTURE_PRESS_SITE[0]), GESTURE_PRESS_SITE[1],
         enc_bl(GESTURE_PRESS_SITE[0], press_addr),
         "bl ring_press (Faceclaw lease gates custom event vs stock UI event 8)"),
        (g2f(GESTURE_RELEASE_SITE[0]), GESTURE_RELEASE_SITE[1],
         enc_bl(GESTURE_RELEASE_SITE[0], release_addr), "bl ring_release (forward ring release-long-press)"),
        (g2f(EVENAI_ENTRY_SITE[0]), EVENAI_ENTRY_SITE[1],
         enc_bw(EVENAI_ENTRY_SITE[0], evenai_entry_addr),
         "even_ai_display_ctrl entry -> conditional Faceclaw lease trampoline"),
        *[(g2f(site), orig, enc_bl(site, display_copy_addr),
           f"bl display_copy_hook @ {site:#x} (640x480 direct framebuffer)")
          for site, orig in DISPLAY_COPY_BL_SITES.items()],
        *[(g2f(site), orig, enc_bl(site, wear_notify_addr),
           f"bl faceclaw_send_wear_event @ {site:#x} (outside onboarding)")
          for site, orig in WEAR_NOTIFY_BL_SITES.items()],
        (g2f(COMPASS_EVENT_BL_SITE[0]), COMPASS_EVENT_BL_SITE[1],
         enc_bl(COMPASS_EVENT_BL_SITE[0], compass_event_addr),
         "bl compass_event_forward (global IMU heading -> stock nav BLE notifier)"),
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
    src = sys.argv[1] if len(sys.argv) > 1 else "g2_2.2.9.22.bin"
    dst = sys.argv[2] if len(sys.argv) > 2 else "g2_2.2.9.22_cfw.bin"
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
