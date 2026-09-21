#!/usr/bin/env python3
"""
Build a CFW image for g2_2.3.0.24 with:
  (1) private SID-f0 message reconstruction and ACKs, and
  (2) custom image/control dispatch (zlib+RLE, keepalive kick + buzzer), and
  (3) a CFW capability-advertisement field (protobuf field 100) plus a private,
      fail-open Faceclaw wake-ownership lease on sid=0x09,
  (4) conditional idle-double-tap dashboard deferral and conditional stock
      Even-AI suppression while that lease is valid, and
  (5) Faceclaw-lease-gated source-qualified long-press, release, and 2.2.9
      tap-then-long-press forwarding, and
  (6) a full-panel 640x480 packed-4bpp shadow copied directly into the physical
      framebuffer, and
  (7) stock wear-state notifications outside onboarding plus a current-state query, and
  (8) Faceclaw compass heading + sample diagnostics from the sensor hub while
      image-handler mode 10 is enabled, with magnetic calibration retained
      across IMU reconfiguration under the framebuffer lease, and
  (9) a lease-scoped 256 KiB resource cache plus cached-image/cached-string drawing
      through image-handler modes 18, 19, and 20,
      and built-in-font mode 15, and
  (10) a phone-controlled microphone configuration + multi-channel audio streaming
      channel (settings fields 103/104 + the 'SM' stream frame) riding the
      already-hooked sid-0x09 settings seams -- no new patch sites; see
      mic_control.c for the contract and its hardware validation gate, and
  (11) LE 2M support, a 7.5 ms / latency-0 fast connection profile, and persistent
      fast-mode requests so the stock 60-second slow-mode timer cannot throttle
      custom image traffic.

REBASED 2.2.9.22 -> 2.3.0.24 (2026-09-19). See docs/firmware-rebase-2.3.0.md
and stock_abi_230.json for the complete site inventory and behavior review.
Every executable absolute dependency and assembly continuation was audited.
Two things to remember for future rebases:
  * a patch site's offset within its host function is NOT stable -- Even inserts code, so
    sites were located using instruction windows and manual control-flow review,
    including decoding displaced `bl` targets and reviewing their calling conventions;
  * hardcoded RAM addresses moved with several DIFFERENT deltas, and some old
    addresses still exist in the new image as unrelated variables. They were re-derived
    through the instructions that load them; witnesses are in stock_abi_230.json.

PLACEMENT MODEL — APPEND, don't overwrite. The injected code blobs
(zlib glue, settings wrapper, gesture_fwd) are APPENDED to
the tail of the main-app component (ota/s200_firmware_ota.bin) rather than being
squeezed into a reclaimed dead function. The bootloader XIP-programs the whole
main-app payload to 0x00438000, so a byte at payload offset K lands at MRAM
0x438000 + K - 0x20; appended blobs therefore load into MRAM immediately after the
current app image (0x007cda60 on 2.3.0.24), with bounded headroom before the
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
import sys, os, struct, zlib, json, subprocess, hashlib
from audit_stock import validate_stock, validate_footprint

DELTA = 0x379BF5  # file_off = ghidra_addr - DELTA  (OTA mainApp component, 2.3.0.24)
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

# BLE policy was hardware-tested on 2.2.9 with sustained 2,000-byte/window-3
# transfers (~41 KiB/s) and a day of battery use. 2.3.0 needs hardware validation. These are Apollo host changes, not EM9305 ROM edits.
# Startup Set Local Feature (vendor opcode 0xfff2): byte 1 bit 0 is LE 2M.
BLE_2M_SITE = (0x4c9b3c, "7c205070")  # movs r0,#0x7c; strb r0,[r2,#1]
# Fast profile min/max intervals (1.25 ms units), then latency/timeout/retries.
BLE_FAST_INTERVAL_SITE = (0x7bcba0, "0c0018000000580205000000")
# _connectParamReq_impl saves mode in r5 and the new generation argument in r7.
# Force 0xa3 (fast), including when the delayed 0xa4 (slow) request arrives.
# Keep the normal connection validation, already-fast check and deferral logic.
# This intentionally applies while idle too; Android can still negotiate another
# interval, and the phone must request 2M PHY to use the newly exposed feature.
BLE_FORCE_FAST_SITE = (0x47bdb4, "05000f00f9f722fb")

# Reserve the final 1 KiB of the stock primary TLSF arena for CFW-owned fixed
# state. Stock initializes [0x2027299c,0x2029f99c) with size 0x2d000 at
# 0x0048dae8. 0x2cc00 is the closest smaller value encodable by the existing
# four-byte Thumb modified-immediate instruction, leaving
# [0x2029f59c,0x2029f99c) outside the allocator. The next stock object starts at
# exactly 0x2029f99c. CFW_CTX_SLOT in cfw_context.h uses the first reserved word.
PRIMARY_TLSF_SIZE_SITE = (0x48dae8, "5ff43432")  # movs.w r2,#0x2d000
PRIMARY_TLSF_CFW_SIZE  = "5f f4 33 32"              # movs.w r2,#0x2cc00
PRIMARY_TLSF_ARENA     = 0x2027299c
PRIMARY_TLSF_STOCK_LEN = 0x2d000
PRIMARY_TLSF_CFW_LEN   = 0x2cc00
CFW_RESERVED_BASE      = PRIMARY_TLSF_ARENA + PRIMARY_TLSF_CFW_LEN
CFW_RESERVED_END       = PRIMARY_TLSF_ARENA + PRIMARY_TLSF_STOCK_LEN

# A bytewise scan for absolute pointers into the reserved tail also interprets every
# four-byte Thumb instruction as a little-endian integer. In 2.3.0.24 these reviewed
# instruction sites happen to spell values in [CFW_RESERVED_BASE, CFW_RESERVED_END):
# mostly `ldr/str ..., [rN, r9, lsl #2]` or `[rN,#0x29]`. The odd hit at
# 0x509fd1 spans ldr.w r2,[pc,#0x9f8] / movs r1,#0x20 (see ABI guard).
# Keep the conservative
# every-alignment scan, but exempt only these exact decoded instruction bytes/sites.
TAIL_REF_FALSE_POSITIVES = {
    0x509fd1: "f8f82920",
    0x526240: "81f82920",
    0x526368: "81f82920",
    0x5264fe: "81f82920",
    0x54d8fe: "94f82920",
    0x54d94a: "94f82920",
    0x586d90: "80f82920",
    0x587a92: "90f82920",
    0x5c863c: "56f82920",
    0x5e36b8: "50f82920",
    0x613f72: "58f82920",
    0x613f94: "56f82920",
    0x613f9c: "56f82920",
    0x613fda: "56f82920",
    0x614000: "52f82920",
    0x61401a: "52f82920",
    0x614022: "56f82920",
    0x61403c: "57f82920",
    0x61404a: "52f82920",
    0x61a0b4: "51f82920",
    0x61a0ba: "41f82920",
}

def mram_addr(payload_off):
    """MRAM XIP address of the byte at this main-app payload offset, once flashed."""
    return APP_LOAD_ADDR + payload_off - APP_PREAMBLE

def align_up(x, a):
    return (x + a - 1) & ~(a - 1)

# ---- call-site redirects (ghidra addr -> stock bytes we expect there) --------
# fff2 ATT write callback, before any TPL reconstruction or SID dispatch.
# r0=pipe(0), r1=borrowed ATT value, r2=uint16 length; r0 returns status.
MESSAGE_RX_BL_SITE = (0x4d6994, "f8f76cfd")  # bl TPL_ReceivePacket
MESSAGE_BRIDGE_BL_SITES = (
    (0x466ede, "fff717f8"),
    (0x46703a, "fef769ff"),
    (0x468e70, "fdf74ef8"),
)

# All 2.3.0.24 addresses. Unchanged hosts/sites were found with normalized
# instruction-window match, unique across the image) and then confirmed by decoding the
# `bl` at the new address and checking it lands on the expected callee -- the bytes below
# are the stock encodings read straight out of the image, so apply_patches' old-byte
# check is a third, independent guard.
#
SETTINGS_BL_SITE       = (0x4aa8dc, "d4f712fb")  # bl FUN_0047ef04 (aa21 send) -> wrapper
# nanopb decode in pb_service_setting's inbound parser. The wrapper scans raw
# unknown field 101 before the stock decoder discards it, then tail-calls decode.
SETTINGS_DECODE_BL_SITE = (0x4a9fdc, "f5f7e2f8") # bl FUN_0049f1a4 -> settings_decode_wrapper
# The two REQUEST_DISPLAY_START_UP(1) sites in the display thread's idle touch
# policy: sub-event 1 (double tap) and sub-event 6 (IMU head-up). Both must defer
# or the peer lens can still flash; each reports its own field-102 event code.
DISPLAY_START_BL_SITES = {
    0x467d68: ("02f083fc", "faceclaw_display_start"),
    0x467e28: ("02f023fc", "faceclaw_display_start_headup"),
}
# The stock idle gate call in that same touch branch. The dispatcher frees the
# message when the gate fails -- which it does while an EvenHub page is on
# screen, so a soft-sleeping phone never hears about a head-up. Wrap the gate
# call: the stock result is passed through unchanged and a head-up is also
# forwarded as EvenHub sys event 12 under the Faceclaw framebuffer lease.
HEADUP_GATE_BL_SITE = (0x467cf4, "07f085fb")  # bl FUN_0046f402 -> headup_gate
# 2.3.0 removed the separate idle-mode call; this same hook now forwards idle
# tap / long-press / release when the stock gate returns 1.
# Menu remains tap-then-long-press. Plain subtype 3 posts UI event 8;
# subtype 0xe posts release event 0x4a; the new subtype 0x11 starts at 0x44499c.
# r6 holds the raw input record at all three sites. Hook the state getter before
# the new press/release menu-off guards, preserving the complete stock path when
# unowned and exiting through the stock success epilogue when owned.
GESTURE_PRESS_SITE      = (0x444940, "1ff071ff") # menu state getter -> gesture_press
GESTURE_SHORT_LONG_SITE = (0x44499c, "1ff043ff") # bl FUN_00464826 -> gesture_short_long
GESTURE_RELEASE_SITE    = (0x444d08, "1ff08dfd") # menu state getter -> gesture_release
GESTURE_RING_PRESS_SITE = (0x44484a, "19f03efb") # mode lookup, r6=input -> ring press observer
# Wakeword ("Hey Even") capture. The old patch unconditionally changed the
# op==START branch in even_ai_display_ctrl, which also broke the official Even
# app. Replace the first four bytes with a B.W trampoline: the injected entry
# reproduces the overwritten push/mov and suppresses START only under Faceclaw's
# volatile lease; with no lease it resumes at 0x4f919a byte-for-byte stock.
EVENAI_ENTRY_SITE      = (0x4f9196, "7fb50600")
# The display task copies the composed 576x288 A4 buffer into the physical
# 640x480 framebuffer at two switch cases. Redirect both calls through
# display_copy_hook: ordinary refreshes pass through, while a pending Faceclaw
# shadow replaces the stock compositor copy immediately before panel refresh.
DISPLAY_COPY_BL_SITES = {
    0x47a78e: "f6f787f9",   # queue message type 3 -> bl FUN_00470aa0
    0x47a8ca: "f6f7e9f8",   # queue message type 6 -> bl FUN_00470aa0
}
# The stock wear handler calls its onboarding-only transmitter in both branches.
# Redirect those calls to our lifecycle-independent sender instead.
WEAR_NOTIFY_BL_SITES = {
    0x4ade92: "d9f7d0fd",  # ON_HEAD:  bl 0x487a36
    0x4adef6: "d9f79efd",  # OFF_HEAD: bl 0x487a36
}
# Capture the selected GAF source before the parser clears it, then attach the
# matching record diagnostics at the sensor-hub heading report call.
COMPASS_DECODE_BL_SITE = (0x4b96d2, "67f02dff")  # bl GAF decode, before output is cleared
COMPASS_REPORT_BL_SITE = (0x4b90ce, "fff7c9fb")  # bl DRV_IMUSendUIEvent(9,heading)
# Inline accuracy reset immediately before the stock cached-mag-bias setter.
# The helper returns the accuracy pointer in r0; the following ldrb supplies r2.
COMPASS_ACCURACY_RESET_SITE = (0x4b75be, "00210170")

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

def validate_ring_battery_stock(img):
    """Pin the read-only stock ABI used by ring_battery.c (2.3.0.24 only)."""
    for address, expected, description in (
        (0x517b48, "0200d2b2652a00db6420384a1070c9b2002901d0012000e0002050707047334890f90000c0b27047304840787047", "cache setter and accessors"),
        (0x517c34, "7e830720", "cache address literal"),
        (0x4806c8, "80b535f0f7f8002808d0fff77eff002801d0012000e00020c0b207e0fff77bff002801d0012000e00020c0b202bd", "dashboard connection predicate"),
        (0x4805d2, "1a480078c0f30010c0b2704717480078c0f34010c0b27047", "connection-bit getters"),
        (0x48063c, "04850720", "connection-bit address literal"),
        (0x4ab556, "80b56cf005fb02bd", "dashboard battery getter"),
    ):
        expected = bytes.fromhex(expected)
        if bytes(img[g2f(address):g2f(address) + len(expected)]) != expected:
            raise ValueError(f"ring battery stock ABI mismatch: {description} at {address:#x}")

def validate_message_transport_stock(img):
    """Pin ingress/bridge call sites, fallback entries and copying TX APIs."""
    for address, expected in (
        (0x4d6938, "1fb5040040f6470089b2814222d166f702ff"),
        (0x4cf470, "2de9f04385b007000d00002d"),
        (0x466ed2, "a388e28814f10801bfb23800fff717f8"),
        (0x46702e, "a388e28814f10801bfb23800fef769ff"),
        (0x468e62, "2569a96848888b88ca88083180b2fdf74ef8"),
        (0x465f10, "2de9f04385b006000f00150098462800f2f71dfb"),
        (0x46a860, "2de9f84388b005000e0090461f00dff800452068002823d1"),
        (0x46aa10, "039988681ffa88f810f10805424631002800cff7dff80120"),
        (0x47ee0a, "feb504000d0016001f00cbf7feff002823d0"),
        (0x47ee64, "bfb2019700962b00dbb22200d2b200210020fff7f6fd"),
        (0x47ec2c, "04980772049880f80980049880f80a90049810f10b051ffa8bfb5a4621002800baf7caff"),
        (0x465d4c, "dff8100c00787047"),
        (0x442ef6, "70b505000026fff761fc0028"),
        (0x442f90, "f8b506000c0075086d0016f0"),
        (0x442ff6, "70b505006c08640015f00105"),
        (0x443048, "38b5040064086400fff7b7fb"),

    ):
        expected = bytes.fromhex(expected)
        if bytes(img[g2f(address):g2f(address) + len(expected)]) != expected:
            raise ValueError(f"message transport stock ABI mismatch at {address:#x}")


def validate_compass_calibration_stock(img):
    """Pin the inline hook ABI, cached bias/accuracy, and vendor restore code."""
    for address, expected in (
        # Prologue saves LR and keeps SP 8-byte aligned at the injected call.
        (0x4b6d98, "2de9f04fcdb0"),
        (0x4b75ba, "dff8fc0a002101700278dff8f41a280069f052fc04430df18d03"),
        (0x4b80b8, "758407205c720720"),
    ):
        expected = bytes.fromhex(expected)
        if bytes(img[g2f(address):g2f(address) + len(expected)]) != expected:
            raise ValueError(f"compass calibration stock ABI mismatch at {address:#x}")
    # The complete vendor setter restores bias, accuracy, covariance, and
    # scaled internal bias. The FIFO block updates the cached bias/accuracy
    # together and still owns anomaly/ready handling. Neither is patched.
    for address, size, digest in (
        (0x520e72, 190, "ddcdc7b2c82e92c65bc200de218dfea0fb94bd5398f3b34353c957d6fd331428"),
        (0x4b96e8, 110, "7cf6f5e136d46e64e6f85c3117ab4bec0eed627983cf603cfb5baf93e66715e3"),
    ):
        if hashlib.sha256(img[g2f(address):g2f(address) + size]).hexdigest() != digest:
            raise ValueError(f"compass calibration stock ABI mismatch at {address:#x}")


def layout(img):
    """Compile the single injected code blob (patches_main.c, which #includes every
    patch source) and append it at the tail of the main-app payload. Returns
    (append_bytes, in_place_patches, mainapp=(idx,off,old_ps)). Enforces the MRAM
    ceiling (duplicate of g2flash.check_mainapp_fits_mram)."""
    audit = validate_stock(img)
    validate_ring_battery_stock(img)
    validate_message_transport_stock(img)
    validate_compass_calibration_stock(img)
    idx, comp_off, old_ps = find_mainapp(img)
    if APP_LOAD_ADDR - (comp_off + 128 + APP_PREAMBLE) != DELTA:
        raise ValueError("main-app file-to-MRAM mapping differs from audited layout")

    # This reservation is safe only if the stock image has no absolute pointer
    # into the removed tail. Scan every byte alignment because the OTA container's
    # file-to-MRAM bias is not word-aligned. The allocator's original exclusive
    # end (0x2029f99c) is intentionally outside the rejected interval and is the
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
    if false_hits != set(TAIL_REF_FALSE_POSITIVES):
        raise ValueError("reviewed TLSF-tail false-positive instruction set changed")
    if tail_refs:
        raise ValueError("stock image references the proposed CFW-reserved TLSF tail: "
                         f"{[hex(off) for off in tail_refs]}")

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
    message_rx_addr = base + _fn(built, "cfw_receive_packet")["offset"]
    message_bridge_addr = base + _fn(built, "faceclaw_input_bridge_received")["offset"]
    settings_addr  = base + _fn(built, "settings_send_wrapper")["offset"]
    settings_decode_addr = base + _fn(built, "settings_decode_wrapper")["offset"]
    display_start_addrs = {name: base + _fn(built, name)["offset"]
                           for _, name in DISPLAY_START_BL_SITES.values()}
    headup_gate_addr = base + _fn(built, "headup_gate")["offset"]
    evenai_entry_addr = base + _fn(built, "faceclaw_evenai_display_entry")["offset"]
    press_addr     = base + _fn(built, "gesture_press")["offset"]
    short_long_addr = base + _fn(built, "gesture_short_long")["offset"]
    release_addr   = base + _fn(built, "gesture_release")["offset"]
    ring_press_addr = base + _fn(built, "gesture_ring_press_mode")["offset"]
    display_copy_addr = base + _fn(built, "display_copy_hook")["offset"]
    wear_notify_addr = base + _fn(built, "faceclaw_send_wear_event")["offset"]
    compass_decode_addr = base + _fn(built, "compass_decode_capture")["offset"]
    compass_report_addr = base + _fn(built, "compass_report_event")["offset"]
    compass_accuracy_addr = base + _fn(built, "compass_preserve_accuracy")["offset"]

    # --- assemble the appended payload bytes (old_ps .. end) ---
    pad = blob_off - old_ps                     # alignment gap before the blob
    # Read-only constants can end on an odd byte. Keep the programmed main-app
    # payload word-aligned, just like the donor, with explicit zero padding.
    end_off = align_up(blob_off + len(blob), BLOB_ALIGN)
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
        (g2f(0x47911a), "38b584b0",
         enc_bw(0x47911a, base + _fn(built, "faceclaw_ring_receive")["offset"]),
         "ring receiver entry: timestamped unfiltered reports under framebuffer lease"),
        # Stock profile_ancc_process_msg branches both WRITE_RSP (9) and
        # WRITE_CMD_RSP (10) to 0x4d8cde, which formerly skipped the gate.
        # Redirect that two-byte branch to the existing r0=event / BL gate path.
        (g2f(0x4d8cde), "15e0", "fae7", "route ANCS write completions through relay event hook"),
        *[(g2f(site), old, enc_bl(site, base + _fn(built, name)["offset"]), name)
          for site, old, name in [
            (0x4d8cd8, "fef788ff", "cfw_ancs_event"),
            (0x4d8cf4, "fef7e8fb", "cfw_ancs_open"),
            (0x4d8d06, "fef7e7fb", "cfw_ancs_close")]],
        (g2f(0x4ccb58), "39694d00",
         struct.pack("<I", base + _fn(built, "cfw_ancs_write")["offset"] + 1).hex(),
         "connection-bound ANCS EUS write callback"),
        *[(g2f(site), old, enc_bl(site, message_bridge_addr),
           "bl faceclaw_input_bridge_received (ring relay and ordered private transport before worker pool)")
          for site, old in MESSAGE_BRIDGE_BL_SITES],
        (g2f(MESSAGE_RX_BL_SITE[0]), MESSAGE_RX_BL_SITE[1],
         enc_bl(MESSAGE_RX_BL_SITE[0], message_rx_addr),
         "bl cfw_receive_packet (private SID-f0 probe before TPL reassembly)"),
        (g2f(BLE_2M_SITE[0]), BLE_2M_SITE[1], "7d 20",
         "Set Local Feature: enable LE 2M bit 8"),
        (g2f(BLE_FAST_INTERVAL_SITE[0]), BLE_FAST_INTERVAL_SITE[1], "06 00 06 00",
         "fast connection interval min=max=7.5 ms; latency remains 0"),
        (g2f(BLE_FORCE_FAST_SITE[0]), BLE_FORCE_FAST_SITE[1], "a3 25",
         "_connectParamReq_impl: force requested mode to fast (0xa3); disables idle slow requests"),
        (g2f(PRIMARY_TLSF_SIZE_SITE[0]), PRIMARY_TLSF_SIZE_SITE[1],
         PRIMARY_TLSF_CFW_SIZE,
         "reserve final 1 KiB of primary TLSF arena for CFW context anchor"),
        # redirect the settings responder send -> settings_send_wrapper (caps field 100)
        (g2f(SETTINGS_BL_SITE[0]), SETTINGS_BL_SITE[1], enc_bl(SETTINGS_BL_SITE[0], settings_addr),
         "bl settings_send_wrapper (append caps field 100)"),
        (g2f(SETTINGS_DECODE_BL_SITE[0]), SETTINGS_DECODE_BL_SITE[1],
         enc_bl(SETTINGS_DECODE_BL_SITE[0], settings_decode_addr),
         "bl settings_decode_wrapper (Faceclaw lease field 101)"),
        *[(g2f(site), orig, enc_bl(site, display_start_addrs[name]),
           f"bl {name} @ {site:#x} (fail-open wake takeover)")
          for site, (orig, name) in DISPLAY_START_BL_SITES.items()],
        (g2f(HEADUP_GATE_BL_SITE[0]), HEADUP_GATE_BL_SITE[1],
         enc_bl(HEADUP_GATE_BL_SITE[0], headup_gate_addr),
         "bl headup_gate (head-up sensor event -> EvenHub sys event 12 under lease)"),
        # Source-qualified long-press, tap-then-long-press, and release forwarding.
        (g2f(GESTURE_RING_PRESS_SITE[0]), GESTURE_RING_PRESS_SITE[1],
         enc_bl(GESTURE_RING_PRESS_SITE[0], ring_press_addr),
         "bl gesture_ring_press_mode (observe ring touch-down, preserve stock dispatch)"),
        (g2f(GESTURE_PRESS_SITE[0]), GESTURE_PRESS_SITE[1],
         enc_bl(GESTURE_PRESS_SITE[0], press_addr),
         "bl gesture_press (owned event 9 before stock menu gate)"),
        (g2f(GESTURE_SHORT_LONG_SITE[0]), GESTURE_SHORT_LONG_SITE[1],
         enc_bl(GESTURE_SHORT_LONG_SITE[0], short_long_addr),
         "bl gesture_short_long (Faceclaw lease gates event 11 vs stock Menu path)"),
        (g2f(GESTURE_RELEASE_SITE[0]), GESTURE_RELEASE_SITE[1],
         enc_bl(GESTURE_RELEASE_SITE[0], release_addr),
         "bl gesture_release (owned event 10 before stock menu gate)"),
        (g2f(EVENAI_ENTRY_SITE[0]), EVENAI_ENTRY_SITE[1],
         enc_bw(EVENAI_ENTRY_SITE[0], evenai_entry_addr),
         "even_ai_display_ctrl entry -> conditional Faceclaw lease trampoline"),
        *[(g2f(site), orig, enc_bl(site, display_copy_addr),
           f"bl display_copy_hook @ {site:#x} (640x480 direct framebuffer)")
          for site, orig in DISPLAY_COPY_BL_SITES.items()],
        *[(g2f(site), orig, enc_bl(site, wear_notify_addr),
           f"bl faceclaw_send_wear_event @ {site:#x} (outside onboarding)")
          for site, orig in WEAR_NOTIFY_BL_SITES.items()],
        (g2f(COMPASS_DECODE_BL_SITE[0]), COMPASS_DECODE_BL_SITE[1],
         enc_bl(COMPASS_DECODE_BL_SITE[0], compass_decode_addr),
         "bl compass_decode_capture (sample-matched GAF diagnostics)"),
        (g2f(COMPASS_REPORT_BL_SITE[0]), COMPASS_REPORT_BL_SITE[1],
         enc_bl(COMPASS_REPORT_BL_SITE[0], compass_report_addr),
         "bl compass_report_event (stock UI + heading with diagnostics over BLE)"),
        (g2f(COMPASS_ACCURACY_RESET_SITE[0]), COMPASS_ACCURACY_RESET_SITE[1],
         enc_bl(COMPASS_ACCURACY_RESET_SITE[0], compass_accuracy_addr),
         "bl compass_preserve_accuracy (retain magnetic calibration under Faceclaw framebuffer lease)"),
    ]
    validate_footprint(img, in_place, audit)
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
    # ANCS uses fixed stock SRAM and Cordio ABIs as well as patched call sites.
    # Authenticate the complete base, not just four-byte hook instructions.
    if hashlib.sha256(img).hexdigest() != "187ccf2bcc5c17a212106e8a376745511e8289c4232b634a7ea94b9bf25a0979":
        raise ValueError("ANCS relay requires the exact audited G2 2.3.0.24 image")
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
        if cur != o:
            raise ValueError(f"{off:#x} ({desc}): expected {o.hex()} got {cur.hex()}")
        record(off, n, desc)
        print(f"  {off:#x}: {desc} ({len(n)} B)")

    # 2) append the injected blobs to the main-app payload. The main app is the
    #    last component, so its payload ends at EOF and appending shifts nothing.
    payload_end = comp_off + 128 + old_ps
    if payload_end != len(data):
        raise ValueError("main-app must be the last component and end exactly at EOF")
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
    src = sys.argv[1] if len(sys.argv) > 1 else "g2_2.3.0.24.bin"
    dst = sys.argv[2] if len(sys.argv) > 2 else "g2_2.3.0.24_cfw.bin"
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
