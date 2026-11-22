#!/usr/bin/env python3
"""Boot a built image in g2-firmware-emulator and check the heaps and the MPU.

The image's application is started by its own bootloader on one emulated
temple, with no phone. After the given number of virtual seconds this checks
that nothing faulted, that the Bluetooth controller is advertising, that the
three heaps have the bounds in ram_layout.h with well-formed TLSF block chains,
that the MPU's non-cacheable region ends with the LVGL arena, and that the RAM
reserved for the screen buffer and resource cache is still all zero.

It does not send the firmware any message, so no injected code beyond the
patched boot path runs: the buffers are never claimed and nothing is allocated
for the message transport.

Run it with the emulator's Python, from the emulator checkout, so that g2emu
and its Renode build are found:

    cd ../g2-firmware-emulator
    .venv/bin/python ../g2flash/patches/host/emulator_boot_check.py \\
        ../g2flash/g2_2.3.0.24_cfw.bin generated/validation/boot-check-1

The output directory must not exist. It receives report.json, the firmware's
UART log, the Renode transcript and a dump of RAM. Pass --stock-heaps to check
an image built before the layout change (LVGL 0xcd000, EvenHub 0x70800) as a
control. Exit status is 0 only if every check passes.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import struct
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import patch_compress as layout

RAM_BASE, RAM_SIZE = 0x20000000, 0x380000
TLSF_CONTROL_BYTES = 0xc74


def expected_heaps(stock):
    if stock:
        lvgl = (layout.LVGL_ARENA, layout.LVGL_STOCK_LEN)
        evenhub = (layout.EVENHUB_STOCK_ARENA, layout.EVENHUB_STOCK_LEN)
    else:
        lvgl = (layout.LVGL_ARENA, layout.LVGL_CFW_LEN)
        evenhub = (layout.EVENHUB_CFW_ARENA, layout.EVENHUB_CFW_LEN)
    return {"lvgl": lvgl, "evenhub": evenhub,
            "primary": (layout.PRIMARY_TLSF_ARENA, layout.PRIMARY_TLSF_CFW_LEN)}


def words(console, commands):
    result = console.command("\n".join(commands))
    values = re.findall(r"(?m)^0x([0-9a-fA-F]+)\s*$", result.replace("\r", ""))
    if len(values) != len(commands):
        raise RuntimeError("incomplete monitor reply: " + result)
    return [int(value, 16) for value in values]


def read32(console, addresses):
    return words(console, [f"sysbus ReadDoubleWord 0x{address:08X}" for address in addresses])


def boot(image, output, seconds):
    from g2emu.atomic_renode_console import AtomicRenodeConsole
    from g2emu.nor_profile import monitor_path
    from g2emu.ota_boot_profile import prepare_boot_board
    from g2emu.ota_preflight import preflight_image
    from g2emu.ota_runtime import ROOT

    raw = image.read_bytes()
    preflight = preflight_image(raw)
    report = {"image": str(image), "sha256": hashlib.sha256(raw).hexdigest(), "virtual_seconds": seconds}
    board = prepare_boot_board(image, output / "board", expected_sha256=preflight["container_sha256"],
                               expected_bootloader_container_sha256=preflight["container_sha256"])
    started = time.monotonic()
    with AtomicRenodeConsole(ROOT, output / "renode", configuration=ROOT / "renode/ota-automation.config") as console:
        console.command(f"include {monitor_path(Path(board['profiles']['boot']))}")
        console.command('mach set "g2"')
        console.command('logLevel "Warning"')
        console.command(f'uart1 CreateFileBackend {monitor_path(output / "uart1.bin")} true')
        report["progress"] = []
        for second in range(1, seconds + 1):
            console.command('emulation RunFor "1"', timeout=1800)
            pc, cfsr, hfsr = words(console, ["cpu PC", "sysbus ReadDoubleWord 0xE000ED28",
                                             "sysbus ReadDoubleWord 0xE000ED2C"])
            report["progress"].append({"virtual_seconds": second, "pc": pc, "cfsr": cfsr, "hfsr": hfsr})
        report["advertising"] = bool(re.search(r"(?m)^True\s*$", console.command("iom6.em9305 AdvertisingEnabled")))
        fields = ("mutex", "tlsf", "current_bytes", "peak_bytes", "size", "base", "policy")
        report["descriptors"] = {
            name: dict(zip(fields, read32(console, range(address, address + 28, 4))))
            for name, address in (("primary", 0x2000033C), ("lvgl", 0x20000358))}
        report["evenhub_tlsf_pointer"], = read32(console, [0x20077ED0])
        ccr, control, selected, mair0 = read32(console, [0xE000ED14, 0xE000ED94, 0xE000ED98, 0xE000EDC0])
        report["scb"] = {"ccr": ccr, "mpu_ctrl": control, "mair0": mair0}
        report["mpu_regions"] = []
        for number in range(16):  # selecting a region changes guest state; restored below
            console.command(f"sysbus WriteDoubleWord 0xE000ED98 {number}")
            rbar, rlar = read32(console, [0xE000ED9C, 0xE000EDA0])
            report["mpu_regions"].append({"number": number, "enabled": bool(rlar & 1), "base": rbar & ~31,
                                          "limit": rlar | 31, "attribute": rlar >> 1 & 7})
        console.command(f"sysbus WriteDoubleWord 0xE000ED98 {selected}")
        console.command(f'dump_file 0x{RAM_BASE:x} 0x{RAM_SIZE:x} {monitor_path(output / "ram.bin")}', timeout=600)
    report["wall_seconds"] = round(time.monotonic() - started, 1)
    return report


def walk_tlsf(ram, arena, size):
    """Follow the physical block chain: (well formed, used, free, largest free, blocks)."""
    end, word, used, free, largest, blocks = arena + size, arena + TLSF_CONTROL_BYTES, 0, 0, 0, 0
    while True:
        header, = struct.unpack_from("<I", ram, word - RAM_BASE)
        block = header & ~3
        if block == 0:
            return word == end - 4, used, free, largest, blocks
        if block < 12 or word + 4 + block > end - 4:
            return False, used, free, largest, blocks
        if header & 1:
            free, largest = free + block, max(largest, block)
        else:
            used += block
        blocks, word = blocks + 1, word + block + 4


def judge(report, ram, stock):
    heaps, results = expected_heaps(stock), []

    def check(passed, text):
        results.append((bool(passed), text))

    progress = report["progress"]
    check(all(row["cfsr"] == 0 and row["hfsr"] == 0 for row in progress), "no fault status in any second")
    check(report["advertising"], "Bluetooth controller is advertising")
    for name in ("lvgl", "primary"):
        base, size = heaps[name]
        d = report["descriptors"][name]
        check(d["mutex"] and d["tlsf"] == base and d["base"] == base and d["size"] == size,
              f"{name} heap descriptor: base {d['base']:#x}, size {d['size']:#x}, "
              f"peak use {d['peak_bytes'] / 1024:.1f} KiB")
    check(report["evenhub_tlsf_pointer"] == heaps["evenhub"][0],
          f"EvenHub heap created at {report['evenhub_tlsf_pointer']:#x}")
    scb = report["scb"]
    check(scb["ccr"] >> 16 & 1, "data cache enabled")
    check(scb["mpu_ctrl"] & 1 and scb["mair0"] & 0xff == 0x44, "MPU enabled, attribute 0 non-cacheable")
    region = report["mpu_regions"][6]
    lvgl_end = sum(heaps["lvgl"])
    check(region["enabled"] and region["attribute"] == 0 and region["base"] == heaps["lvgl"][0] & ~31
          and region["limit"] == (lvgl_end - 1) | 31,
          f"non-cacheable region 6 is [{region['base']:#x}, {region['limit']:#x}], ending with the LVGL arena")
    for name, (arena, size) in heaps.items():
        valid, used, free, largest, blocks = walk_tlsf(ram, arena, size)
        check(valid, f"{name} heap block chain reaches {arena + size:#x}: {blocks} blocks, "
                     f"{used / 1024:.1f} KiB used, {free / 1024:.1f} KiB free")
    if not stock:
        check(region["limit"] < layout.SCREEN_BUFFER_ADDR, "screen buffer and resource cache are in cached RAM")
        gap = ram[lvgl_end - RAM_BASE:heaps["evenhub"][0] - RAM_BASE]
        check(not any(gap), f"the {len(gap):#x} bytes between the LVGL and EvenHub heaps are still zero")
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("image", type=Path)
    parser.add_argument("output", type=Path, help="fresh directory for the report, logs and RAM dump")
    parser.add_argument("--seconds", type=int, default=10, help="virtual seconds to run (default 10)")
    parser.add_argument("--stock-heaps", action="store_true",
                        help="expect the stock LVGL and EvenHub bounds (an image from before the layout change)")
    arguments = parser.parse_args()
    output = arguments.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    report = boot(arguments.image.resolve(), output, arguments.seconds)
    results = judge(report, (output / "ram.bin").read_bytes(), arguments.stock_heaps)
    report["checks"] = [{"passed": passed, "check": text} for passed, text in results]
    report["passed"] = all(passed for passed, _ in results)
    (output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"{report['sha256']}  {arguments.seconds} virtual s in {report['wall_seconds']} s")
    print("pc each second: " + " ".join(f"{row['pc']:08x}" for row in report["progress"]))
    for passed, text in results:
        print(("ok    " if passed else "FAIL  ") + text)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
