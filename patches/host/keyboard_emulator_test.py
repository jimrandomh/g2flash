#!/usr/bin/env python3
"""Execute the committed keyboard ARM code with g2-firmware-emulator.

Requires its Python package; no Renode, physical BLE, or vendor image download.
Stock imports are explicit deterministic shims for the 2.3.0.24 ABI. Unknown
imports fail closed. See docs/keyboard-emulator.md for scope and invocation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys

from g2emu.cfw_symbols import decode_thumb_branch, load_cfw_layout
from g2emu.unicorn_harness import (
    CfwUnicornHarness, HarnessAllocator, ImportOutcome, RETURN_SENTINEL, STACK_TOP,
)
from unicorn.arm_const import (
    UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3,
    UC_ARM_REG_R4, UC_ARM_REG_R5, UC_ARM_REG_R6, UC_ARM_REG_R12, UC_ARM_REG_XPSR,
)
from unicorn import UC_HOOK_CODE
import unicorn

ROOT = Path(__file__).resolve().parents[2]
EMULATOR_COMMIT = "f30527c0b7e6fa6e9be06e7067e49468197fa258"


class KeyboardMachine:
    def __init__(self, executable, layout, symbols):
        self.h = CfwUnicornHarness(executable, layout)
        self.symbols = symbols
        self.alloc = HarnessAllocator()
        self.messages, self.packets, self.requests, self.closes = [], [], [], []
        self.timers, self.master, self.security, self.confirms = [], [], [], []
        self.scans, self.opens, self.auths, self.compares = [], [], [], []
        self.side, self.fail_wsf, self.fail_att, self.fail_timer = 1, False, False, False
        self.db_counts, self.bond = {0: 1, 1: 1}, 0
        self.stock_dispatches = 0
        self.timer_active = self.in_timer = False
        hook = self.h.hook_import
        hook(0x45855f, lambda h, c: self.alloc.allocate(h, c.args[0], "malloc"))
        hook(0x4585a3, lambda h, c: self.alloc.free(h, c.args[0]))
        hook(0x465d4d, lambda h, c: self.side)
        hook(0x442b65, self.timer_new)
        hook(0x442c4d, self.timer_start)
        hook(0x442cf3, lambda h, c: 0)
        hook(0x4d8e77, lambda h, c: 0 if self.fail_wsf else self.buf(bytes(c.args[0])))
        hook(0x4d8e93, self.post)
        hook(0x47f025, self.notify)
        hook(0x4843a9, lambda h, c: self.bond)
        hook(0x483c2d, lambda h, c: self.db_counts[c.args[0]])
        hook(0x51d249, self.open)
        hook(0x4ce3f5, lambda h, c: self.closes.append(c.args[:3]))
        hook(0x51c7e5, lambda h, c: self.security.append(c.args[:3]))
        hook(0x51d0c3, lambda h, c: self.master.append(self.read(c.args[0], 4)))
        hook(0x51d163, lambda h, c: self.master.append(self.read(c.args[0], 4)))
        hook(0x4e6685, lambda h, c: self.auths.append((c.args[:2], self.read(c.args[2], 3))))
        hook(0x550239, lambda h, c: self.compares.append(c.args[:2]))
        hook(0x579c13, lambda h, c: self.scans.append((c.args[:2], self.read(c.args[2], 1), c.args[3], c.stack_words[:2])))
        hook(0x4c9fcd, lambda h, c: 0 if self.fail_att else self.buf(bytes(c.args[0])))
        hook(0x4ca561, self.request)
        hook(0x4ca759, lambda h, c: self.confirms.append(c.args[0]))
        hook(0x4cc1f1, self.stock_dispatch)
        self.h.uc.mem_write(0x20073eba, b"\x07")

    def read(self, ptr, size):
        return bytes(self.h.uc.mem_read(ptr, size))

    def buf(self, data):
        ptr = self.h.allocate_scratch(len(data))
        self.h.uc.mem_write(ptr, data)
        return ptr

    def call(self, name, *args, **kwargs):
        return self.h.call_address(self.symbols[name], *args, **kwargs)

    def timer_new(self, h, c):
        assert c.args[1] == 0 and c.args[3] == 0
        self.timers.append(c.args)
        return 0 if self.fail_timer else len(self.timers)

    def timer_start(self, h, c):
        assert c.args[:2] == (1, 20)
        self.timer_active = True
        return 0

    def post(self, h, c):
        assert c.args[0] == 7
        self.messages.append(c.args[1])

    def notify(self, h, c):
        assert self.in_timer, "phone sender must never run on the WSF task"
        assert c.args[:2] == (1, 9)
        p = self.read(c.args[2], c.args[3])
        assert p[:6] == bytes.fromhex("080310009a08")
        length = p[6] & 127
        start = 7
        if p[6] & 128:
            length |= p[7] << 7
            start += 1
        assert len(p) == start + length
        self.packets.append(p[start:])
        return 0

    def open(self, h, c):
        self.opens.append((c.args[:2], self.read(c.args[2], 6), c.args[3]))
        return 3  # synthetic phone=1, ring=2, keyboard=3

    def request(self, h, c):
        assert c.stack_words[0] == 0
        size = struct.unpack("<H", self.read(c.args[3], 2))[0]
        self.requests.append((c.args[:3], self.read(c.args[3], size + 8)))

    def stock_dispatch(self, h, c):
        self.stock_dispatches += 1
        assert c.stack_pointer == STACK_TOP - 32
        saved = struct.unpack("<IIII", self.read(c.stack_pointer + 16, 16))
        assert saved[-1] == RETURN_SENTINEL | 1
        return ImportOutcome(resume_address=RETURN_SENTINEL | 1, stack_pointer=STACK_TOP)

    def pump(self):
        while self.messages:
            self.call("keyboard_dispatch_entry", 0, self.messages.pop(0))

    def tick(self):
        assert len(self.timers) == 1
        if not self.timer_active:
            return
        self.timer_active = False
        cb, _, state, _ = self.timers[0]
        assert cb == self.symbols["kb_tick"] | 1
        self.in_timer = True
        try:
            self.h.call_address(cb, state)
        finally:
            self.in_timer = False
        self.pump()

    def drain(self):
        # At most 32 fragments; do not advance time/create heartbeat traffic.
        for _ in range(9):
            self.tick()

    def control(self, op, body=b"", epoch=0, request=1):
        p = b"KB\x01" + bytes([op]) + struct.pack("<HH", request, epoch) + body
        self.call("keyboard_apply_control", self.buf(p), len(p))
        self.pump()

    def event(self, event, conn=3, status=0, body=bytes(32)):
        ptr = self.buf(struct.pack("<HBB", conn, event, status) + body)
        self.call("keyboard_dispatch_entry", 0, ptr)
        return ptr

    def att(self, event=13, value=b"\x00\x00\x04\x00\x00\x00\x00\x00", conn=3, handle=0x25, status=0):
        p = struct.pack("<HBBIHHBBH", conn, event, status, self.buf(value), len(value), handle, 0, 0, 23)
        ptr = self.buf(p)
        self.call("keyboard_dispatch_entry", 0, ptr)
        return ptr

    def connected(self, encrypted=True):
        self.control(1)
        # Two occupied CCBs leave the third slot for the keyboard.
        for i in range(2):
            self.h.uc.mem_write(0x20073c30 + i * 0x30 + 0x16, b"\x01")
        self.control(4, b"\x01\x06\x05\x04\x03\x02\x01")
        assert self.opens == [((1, 1), bytes.fromhex("060504030201"), 0)]
        self.event(0x27)
        if encrypted:
            self.event(0x2c)
        self.drain()
        self.packets.clear()

    def watch(self):
        self.control(12, bytes.fromhex("012500"), epoch=1)
        self.drain()
        self.packets.clear()


def dormant(m):
    ptr = m.event(0x27, conn=2)
    assert m.stock_dispatches == 1 and not m.alloc.attempts and not m.timers
    for name, reg in [("keyboard_dm_alloc_entry", UC_ARM_REG_R6), ("keyboard_att_alloc_entry", UC_ARM_REG_R5)]:
        m.fail_wsf = True
        assert m.call(name, 32, initial_registers={reg: ptr}).return_value == 0
    assert not m.alloc.attempts and not m.timers
    m.control(1, b"bad")
    m.side = 2
    m.control(1)
    assert not m.alloc.attempts and not m.timers and not m.messages


def pairing_and_routing(m):
    m.connected(encrypted=False)
    m.control(6, epoch=1)
    assert m.security == [(3, 1, 0x20073990)]
    m.event(0x2e, body=b"\x00\x01" + bytes(30))
    m.control(7, (123456).to_bytes(3, "little"), epoch=1)
    assert m.auths == [((3, 3), (123456).to_bytes(3, "little"))]
    m.event(0x35, body=bytes(12) + (1123456).to_bytes(4, "big"))
    m.control(8, b"\x01", epoch=1)
    assert m.compares == [(3, 1)]
    m.drain()
    assert any(p[3] == 4 and p[12:14] == b"\x35\x00" and p[24:] == struct.pack("<I", 123456) for p in m.packets)
    m.control(12, bytes.fromhex("012500"), epoch=1)
    m.drain()
    assert m.packets[-1][12:14] == b"\x07\x00"  # encryption required
    m.event(0x2c)
    m.watch()
    m.att(conn=1)
    m.att(conn=2)
    m.event(0x27, conn=2)
    m.event(0x34)  # global ECC event must reach stock
    assert m.stock_dispatches == 4
    m.att()
    m.drain()
    inputs = [p for p in m.packets if p[3] == 3]
    assert len(inputs) == 1 and inputs[0][24:] == bytes.fromhex("0000040000000000")
    assert inputs[0][8:10] == b"\x01\x00" and inputs[0][14:16] == b"\x25\x00"


def io_capability(m):
    cfg = m.buf(bytes(4) + b"\x03")
    cfg_ptr = m.buf(struct.pack("<I", cfg))
    def check(conn, expected):
        ccb = m.buf(bytes(0x3d) + bytes([conn]))
        preserved = {UC_ARM_REG_R4: 0x44444444, UC_ARM_REG_R5: ccb,
                     UC_ARM_REG_R6: 0x66666666, UC_ARM_REG_R12: 0x12121212}
        # call_address initializes XPSR, so seed APSR at the first instruction.
        entry = m.symbols["keyboard_io_cap_entry"]
        flags = 0xa8000000  # N, C and sticky Q
        hook = m.h.uc.hook_add(UC_HOOK_CODE, lambda uc, a, size, data:
                              uc.reg_write(UC_ARM_REG_XPSR, flags | (1 << 24)),
                              begin=entry, end=entry)
        try:
            m.call("keyboard_io_cap_entry", 0x11111111, cfg_ptr, 99, 0x33333333, initial_registers=preserved)
        finally:
            m.h.uc.hook_del(hook)
        assert m.h.uc.reg_read(UC_ARM_REG_XPSR) & 0xf8000000 == flags
        assert m.h.uc.reg_read(UC_ARM_REG_R2) == expected
        for reg, val in {UC_ARM_REG_R0: 0x11111111, UC_ARM_REG_R1: cfg_ptr,
                         UC_ARM_REG_R3: 0x33333333, **preserved}.items():
            assert m.h.uc.reg_read(reg) == val
    check(3, 3)
    m.connected()
    for conn, expected in [(1, 3), (2, 3), (3, 4)]:
        check(conn, expected)
    m.control(2, epoch=1)
    check(3, 3)
    assert m.read(cfg + 4, 1) == b"\x03"


def att_requests(m):
    m.connected()
    cases = [
        (9, "0100ffff", 2, "05000100ffff00000400000000"),
        (10, "25000000", 5, "03000000000000000a2500"),
        (10, "25001600", 6, "05001600000000000c25001600"),
        (11, "260000020100", 9, "05000000000000001226000100"),
        (11, "260001020000", 10, "05000000000000005226000000"),
    ]
    for op, body, event, packet in cases:
        m.control(op, bytes.fromhex(body), epoch=1, request=77)
        args, actual = m.requests[-1]
        assert args == (3, int.from_bytes(bytes.fromhex(body)[:2], "little"), event)
        assert actual.hex() == packet, (actual.hex(), packet)
        m.att(event=event, value=b"\x01\x02", handle=args[1])
        m.drain()
        assert any(p[3] == 2 and p[10:12] == b"M\x00" and p[24:] == b"\x01\x02" for p in m.packets)
        m.packets.clear()
    m.fail_att = True
    m.control(10, bytes.fromhex("25000000"), epoch=1)
    m.drain()
    assert m.packets[-1][12:14] == b"\x09\x00"


def fragments_overflow_and_oom(m):
    m.connected()
    m.watch()
    value = bytes(range(256)) * 2
    m.att(value=value)
    m.drain()
    parts = [p for p in m.packets if p[3] == 3]
    assert len(parts) == 6 and b"".join(p[24:] for p in parts) == value
    assert len({p[4:8] for p in parts}) == 1
    assert [int.from_bytes(p[18:20], "little") for p in parts] == [0, 96, 192, 288, 384, 480]
    m.packets.clear()
    for _ in range(33):
        m.att()
    m.drain()
    inputs = [p for p in m.packets if p[3] == 3]
    assert len(inputs) == 32
    previous = int.from_bytes(inputs[-1][4:8], "little")
    m.h.write_u32(0x20077e4c, 1000)
    m.drain()
    heartbeat = m.packets[-1]
    assert heartbeat[3] == 1 and int.from_bytes(heartbeat[30:34], "little") == 1
    assert int.from_bytes(heartbeat[4:8], "little") == previous + 2
    ptr = m.att(value=bytes(8))
    m.fail_wsf = True
    assert m.call("keyboard_att_alloc_entry", 24, initial_registers={UC_ARM_REG_R5: ptr}).return_value == 0
    close = m.buf(struct.pack("<HBB", 3, 0x28, 0) + bytes(32))
    assert m.call("keyboard_dm_alloc_entry", 40, initial_registers={UC_ARM_REG_R6: close}).return_value == 0
    m.fail_wsf = False
    m.drain()
    status = [p for p in m.packets if p[3] == 1][-1]
    assert status[25:28] == bytes(3) and int.from_bytes(status[30:34], "little") == 3
    m.att(conn=3)  # closed connId is no longer keyboard-owned
    assert m.stock_dispatches == 1


def timeout_and_disable(m):
    m.connected()
    m.watch()
    m.control(10, bytes.fromhex("25000000"), epoch=1)
    m.h.write_u32(0x20077e4c, 30001)
    m.drain()
    assert m.closes == [(3, 3, 0x13)]
    assert any(p[3] == 1 and p[12:14] == b"\x08\x00" for p in m.packets)
    m.packets.clear()
    m.att(event=14)
    m.drain()
    assert m.confirms == [3] and not any(p[3] == 3 for p in m.packets)
    m.control(2, epoch=1)
    m.event(0x28)
    m.drain()
    assert not m.timer_active
    count = len(m.packets)
    m.h.write_u32(0x20077e4c, 40001)
    m.drain()
    assert len(m.packets) == count


def scan_and_capacity(m):
    m.control(1)
    m.h.uc.mem_write(0x200769cc, b"\x01")
    m.control(3)
    assert not m.scans
    m.h.uc.mem_write(0x200769cc, b"\x00")
    m.control(3)
    assert m.scans == [((1, 0), b"\x01", 1, (5000, 0))]
    advert = b"\x02\x01\x06"
    body = struct.pack("<IBbBB6s", m.buf(advert), len(advert), -45, 0, 1, bytes.fromhex("060504030201"))
    m.event(0x26, conn=0, body=body)
    m.event(0x25, conn=0)
    m.drain()
    assert any(p[3] == 5 and p[24:] == bytes.fromhex("01060504030201d30003") + advert for p in m.packets)
    m.db_counts[1] = 5
    m.control(4, b"\x01" + bytes(6))
    m.drain()
    assert not m.opens and m.packets[-1][12:14] == b"\x05\x00"


def allocation_failure(m):
    m.alloc.fail_after = 0
    m.control(1)
    assert not m.timers and not m.messages and m.h.read_u32(0x2029f59c) == 0
    m.alloc.fail_after = None
    m.fail_timer = True
    m.control(1)
    assert len(m.alloc.live) == 1 and not m.alloc.invalid_frees and not m.messages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--image", required=True, type=Path)
    parser.add_argument("--build-json", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    layout = load_cfw_layout(ROOT / "patches/cfw_patches.json")
    image = args.image.read_bytes()
    assert hashlib.sha256(image).hexdigest() == layout.output_sha256
    build = json.loads(args.build_json.read_text())
    compiled = bytes.fromhex(build["text"])
    assert layout.appended_bytes[:len(compiled)] == compiled, "build symbols do not match committed image"
    symbols = {f["name"]: layout.injection_address + f["offset"] for f in build["functions"]}
    executable = image[layout.executable_container_offset:layout.injection_container_offset + len(layout.appended_bytes)]
    # Verify all five actual patch sites, including B.W entries which the
    # upstream canonical manifest symbol-name parser does not recognize.
    sites = {0x4caa60: "keyboard_dm_alloc_entry", 0x4caab0: "keyboard_att_alloc_entry",
             0x60093e: "keyboard_io_cap_entry", 0x601298: "keyboard_io_cap_entry",
             0x4cc1ec: "keyboard_dispatch_entry"}
    for site, name in sites.items():
        branch = decode_thumb_branch(site, executable[site - 0x438000:site - 0x438000 + 4])
        assert branch.target == symbols[name]
        assert branch.kind == ("b.w" if name == "keyboard_dispatch_entry" else "bl")
    import g2emu
    emulator_root = Path(g2emu.__file__).resolve().parents[2]
    commit = subprocess.check_output(["git", "-C", str(emulator_root), "rev-parse", "HEAD"], text=True).strip()
    assert commit == EMULATOR_COMMIT, "use the documented pinned emulator revision"
    emulator_sources = {}
    for name in ["cfw_symbols.py", "unicorn_harness.py", "firmware.py"]:
        relative = f"src/g2emu/{name}"
        source = (emulator_root / relative).read_bytes()
        committed = subprocess.check_output(["git", "-C", str(emulator_root), "show", f"HEAD:{relative}"])
        assert source == committed, f"modified emulator source: {relative}"
        emulator_sources[relative] = hashlib.sha256(source).hexdigest()
    results = []
    for test in [dormant, pairing_and_routing, io_capability, att_requests,
                 fragments_overflow_and_oom, timeout_and_disable, scan_and_capacity, allocation_failure]:
        m = KeyboardMachine(executable, layout, symbols)
        test(m)
        assert not m.alloc.invalid_frees
        results.append({"scenario": test.__name__, "passed": True,
                        "executed_instruction_addresses": len(m.h.covered_addresses),
                        "stock_import_calls": len(m.h.import_calls)})
        print(f"PASS {test.__name__}", flush=True)
    report = {"emulator_commit": commit, "image_sha256": layout.output_sha256,
              "emulator_sources": emulator_sources, "unicorn_version": unicorn.__version__,
              "python_version": sys.version.split()[0],
              "injected_sha256": layout.appended_sha256, "hook_sites_verified": len(sites),
              "scope": "isolated ARM execution with explicit synthetic 2.3.0.24 stock imports; not boot/radio/iOS qualification",
              "results": results}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")


if __name__ == "__main__":
    main()
