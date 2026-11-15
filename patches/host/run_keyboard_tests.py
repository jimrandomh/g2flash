#!/usr/bin/env python3
"""Offline checks for the BLE keyboard bridge.

python3 patches/host/run_keyboard_tests.py [--out DIR] [--no-sanitize]

Builds keyboard_host_test.c for the host (the production state machine from
keyboard_core.h plus the timer functions lifted out of keyboard.c) and runs it
under ASan/UBSan, then runs the reference client's tests and typecheck in
demos/. No Bluetooth connection and no firmware flashing. Needs cc and, for the
client half, bun with the demos dependencies installed.
"""
import argparse
import re
import shutil
import subprocess
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def run(*args, cwd=ROOT):
    print("+", " ".join(map(str, args)), flush=True)
    subprocess.run(list(map(str, args)), cwd=cwd, check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--no-sanitize", action="store_true")
    opts = parser.parse_args()
    out = opts.out.resolve() if opts.out else Path(tempfile.mkdtemp(prefix="g2-keyboard-"))
    out.mkdir(parents=True, exist_ok=True)
    flags = [] if opts.no_sanitize else ["-fsanitize=address,undefined", "-fno-sanitize-recover=all"]

    # The host test supplies its own platform shims but runs the real timer
    # functions, which live next to the Thumb-only trampolines in keyboard.c.
    keyboard = (ROOT / "patches/keyboard.c").read_text()
    timers = []
    for name in ("kb_kick", "kb_schedule", "kb_tick"):
        match = re.search(r"(?:__attribute__\(\(used\)\) )?static void " + name + r"\([^;{]*\) \{\n.*?^\}",
                          keyboard, re.M | re.S)
        if not match:
            raise RuntimeError(f"Cannot find keyboard timer function {name}")
        timers.append(match[0])
    (out / "keyboard_timer_functions.inc").write_text("\n\n".join(timers) + "\n")

    binary = out / "keyboard_host_test"
    run("cc", "-std=c11", "-O1", "-g", "-Wall", "-Wextra", "-Wno-unused-function", *flags,
        "-Ipatches", "-I" + str(out), "-o", binary, "patches/host/keyboard_host_test.c")
    run(binary)

    demos = ROOT / "demos"
    if not shutil.which("bun") or not (demos / "node_modules").is_dir():
        print("skipping the reference client tests: need bun and `bun install` in demos/")
        return
    run("bun", "test", "keyboard.test.ts", cwd=demos)
    run("bun", "node_modules/typescript/bin/tsc", "-p", "tsconfig.keyboard.json", cwd=demos)


if __name__ == "__main__":
    main()
