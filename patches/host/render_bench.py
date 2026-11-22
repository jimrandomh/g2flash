#!/usr/bin/env python3
"""Replay Faceclaw screen recordings through the firmware's renderer and count
the Thumb instructions it takes.

  venv/bin/python patches/host/render_bench.py [--baseline DIR] [--profile]
      [--no-inline] RECORDING.fcrec.jsonl ...

A recording is what Faceclaw's "Record screen" writes (faceclaw
tools/screen-recording-visualizer/FORMAT.md): the glasses' display state, then
every Bluetooth message. This takes the starting state and the renderer's
messages (resource upload/evict/create, draw calls, set root, present),
compiles the renderer from patches/ exactly as the firmware build does
(render_bench.c, build.py's flags) and runs it under Unicorn for each lens. An
animating root list is redrawn every 45 ms until the next message, as the
firmware's timer would.

Per recording and lens it prints the instructions executed for draw-call
messages, set-root messages, presents and animation redraws, and a digest of
every composed frame, the final screen buffer and the final resources.

--baseline DIR  also run the patch sources in DIR (for example the patches/ of
                a `git worktree` at an older commit), require identical frames,
                and print both instruction counts.
--profile       list the functions the instructions were spent in.
--no-inline     compile with -fno-inline, which makes --profile attribute work
                to the small helpers that are otherwise inlined (the totals are
                then not the real build's).

Instruction counts are a proxy for time: no cache or pipeline is modelled, and
a divide counts as one instruction (the number executed is printed separately).
Needs clang, and unicorn and capstone in the Python environment. Nothing here
talks to glasses.
"""
import argparse
import base64
import collections
import hashlib
import json
import struct
import sys
import tempfile
from pathlib import Path

import capstone
import unicorn as uc
from unicorn import arm_const as arm

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'patches'))
import build  # noqa: E402

# Keep in sync with render_bench.c.
CODE, STOP = 0x00800000, 0x00900000
RAM, RAM_SIZE = 0x20000000, 0x800000
RIGHT_LENS, SCREEN, FRAMEBUFFER, MESSAGE, STACK_TOP = 0x20110000, 0x20300000, 0x20340000, 0x20400000, 0x20700000
FRAME_BYTES = 640 * 480 // 2

# Private-stream message types (cfw_message_type in zlib_glue.c).
UPLOAD, EVICT, DRAW_CALLS, SET_ROOT, PRESENT, CREATE_SURFACE = 21, 22, 26, 27, 28, 29
REDRAW = 'redraw'  # an animation frame: not a message
RENDERER_MESSAGES = (UPLOAD, EVICT, DRAW_CALLS, SET_ROOT, PRESENT, CREATE_SURFACE)
COLUMNS = ((DRAW_CALLS, 'draw'), (SET_ROOT, 'set-root'), (PRESENT, 'present'), (REDRAW, 'redraw'))
ANIMATION_INTERVAL_MS = 45
DRAW_OP_STOCK_FONT_STRING = 3
RESOURCE_TYPE_MASK, RESOURCE_TYPE_DISPLAY_LIST = 3, 2
NO_ROOT = 65535
MAX_UPLOAD_CHUNK = 60000


class Unsupported(Exception):
    pass


def draw_ops(sequence):
    """Opcodes of a [count16]{[length16][call]} sequence."""
    count, = struct.unpack_from('<H', sequence, 0)
    position = 2
    for _ in range(count):
        length, = struct.unpack_from('<H', sequence, position)
        yield sequence[position + 2]
        position += 2 + length


def load(path):
    """A recording as [(kind, time_ms, payload)]: kind 'screen' or 'message'."""
    records = []
    ops = set()

    def upload(resource_id, data, time):
        for offset in range(0, len(data), MAX_UPLOAD_CHUNK):
            chunk = data[offset:offset + MAX_UPLOAD_CHUNK]
            header = struct.pack('<BHHIHH', UPLOAD, 1, resource_id, len(data), offset, len(chunk))
            records.append(('message', time, header + chunk))

    with open(path) as lines:
        for line in lines:
            record = json.loads(line)
            kind = record['type']
            if kind == 'header':
                if record.get('glasses') != 'g2' or (record.get('width'), record.get('height')) not in ((640, 480), (0, 0)):
                    raise Unsupported('not a 640x480 G2 recording')
            elif kind == 'state':
                time = record['t']
                if 'screen' in record:
                    records.append(('screen', time, base64.b64decode(record['screen'])))
                for resource in record.get('resources', []):
                    data = base64.b64decode(resource['data'])
                    if data[0] & RESOURCE_TYPE_MASK == RESOURCE_TYPE_DISPLAY_LIST:
                        ops.update(draw_ops(data[1:]))
                    upload(resource['id'], data, time)
                if record.get('root', NO_ROOT) != NO_ROOT:
                    records.append(('message', time, struct.pack('<BH', SET_ROOT, record['root'])))
            elif kind == 'msg' and record.get('sid') == 0xf0 and 'payload' in record and not record.get('baseline'):
                payload = base64.b64decode(record['payload'])
                if payload[0] & 127 in RENDERER_MESSAGES:
                    if payload[0] & 127 == DRAW_CALLS:
                        ops.update(draw_ops(payload[1:]))
                    records.append(('message', record['t'], payload))
    if DRAW_OP_STOCK_FONT_STRING in ops:
        raise Unsupported('draws with the stock font, which needs the stock firmware image')
    return records


class Renderer:
    """The renderer from one patches directory, as a Thumb blob in Unicorn."""

    def __init__(self, tree, objects, inline=True):
        extra = ['-I', str(tree)]
        if 'rle_state' in (Path(tree) / 'rle.h').read_text():
            extra.append('-DRENDER_BENCH_OLD_RLE')
        if not inline:
            extra.append('-fno-inline')
        build.OBJ_DIR = str(objects)
        blob, functions, _ = build.compile_text(str(Path(__file__).with_name('render_bench.c')), extra)
        self.functions = sorted((offset, size, name) for name, offset, size in functions)
        self.address = {name: CODE + offset for name, offset, size in functions}
        self.disassembler = capstone.Cs(capstone.CS_ARCH_ARM, capstone.CS_MODE_THUMB | capstone.CS_MODE_MCLASS)
        self.block_info = {}
        self.blocks = collections.Counter()
        self.machine = machine = uc.Uc(uc.UC_ARCH_ARM, uc.UC_MODE_THUMB | uc.UC_MODE_MCLASS)
        machine.mem_map(CODE, 0x100000)
        machine.mem_map(STOP, 0x1000)
        machine.mem_map(RAM, RAM_SIZE)
        machine.mem_write(CODE, bytes(blob))
        machine.hook_add(uc.UC_HOOK_BLOCK, self.on_block)

    def on_block(self, machine, address, size, user):
        self.blocks[(address, size)] += 1

    def reset(self, right_lens):
        self.machine.mem_write(RAM, bytes(RAM_SIZE))
        self.machine.mem_write(RIGHT_LENS, struct.pack('<I', right_lens))
        self.blocks.clear()

    def call(self, name, *arguments):
        machine = self.machine
        for index, value in enumerate(arguments):
            machine.reg_write(arm.UC_ARM_REG_R0 + index, value & 0xffffffff)
        machine.reg_write(arm.UC_ARM_REG_SP, STACK_TOP)
        machine.reg_write(arm.UC_ARM_REG_LR, STOP | 1)
        machine.emu_start(self.address[name] | 1, STOP)
        value = machine.reg_read(arm.UC_ARM_REG_R0)
        return value - (1 << 32) if value & 0x80000000 else value

    def describe(self, block):
        """(instructions, divides, function) of one translated block."""
        if block not in self.block_info:
            address, size = block
            instructions = divides = 0
            for instruction in self.disassembler.disasm(bytes(self.machine.mem_read(address, size)), address):
                instructions += 1
                divides += instruction.mnemonic.startswith(('udiv', 'sdiv'))
            name = '?'
            for offset, length, candidate in self.functions:
                if offset <= address - CODE < offset + max(length, 1):
                    name = candidate
            self.block_info[block] = (instructions, divides, name)
        return self.block_info[block]

    def take(self):
        """Instructions, divides and per-function instructions since the last take."""
        instructions = divides = 0
        by_function = collections.Counter()
        for block, times in self.blocks.items():
            count, divide_count, name = self.describe(block)
            instructions += count * times
            divides += divide_count * times
            by_function[name] += count * times
        self.blocks.clear()
        return instructions, divides, by_function


class Result:
    def __init__(self):
        self.instructions = collections.Counter()
        self.divides = 0
        self.functions = collections.Counter()
        self.failures = self.frames = 0
        self.digest = hashlib.blake2b(digest_size=8)

    def add(self, other):
        self.instructions.update(other.instructions)
        self.divides += other.divides
        self.functions.update(other.functions)
        self.failures += other.failures
        self.frames += other.frames

    def drawing(self):
        return sum(self.instructions[kind] for kind, _ in COLUMNS)

    def columns(self):
        return ' '.join(f'{label}={self.instructions[kind]}' for kind, label in COLUMNS)


def replay(renderer, records, right_lens):
    result = Result()
    renderer.reset(right_lens)

    def account(kind):
        instructions, divides, by_function = renderer.take()
        result.instructions[kind] += instructions
        if kind in (DRAW_CALLS, SET_ROOT, PRESENT, REDRAW):
            result.divides += divides
            result.functions.update(by_function)

    def frame():
        result.digest.update(renderer.machine.mem_read(FRAMEBUFFER, FRAME_BYTES))
        result.frames += 1

    for index, (kind, time, payload) in enumerate(records):
        if kind == 'screen':
            renderer.machine.mem_write(SCREEN, payload)
            continue
        mode = payload[0] & 127
        renderer.machine.mem_write(MESSAGE, payload)
        status = renderer.call('bench_message', MESSAGE, len(payload))
        account(mode)
        result.failures += status < 0
        if mode != PRESENT:
            continue
        frame()
        until_next = records[index + 1][1] - time if index + 1 < len(records) else 2000
        elapsed = ANIMATION_INTERVAL_MS
        while status == 1 and elapsed < min(until_next, 2000):
            status = renderer.call('bench_present', elapsed)
            account(REDRAW)
            frame()
            elapsed += ANIMATION_INTERVAL_MS
    result.digest.update(renderer.machine.mem_read(SCREEN, FRAME_BYTES))
    result.digest.update(struct.pack('<I', renderer.call('bench_resource_hash') & 0xffffffff))
    renderer.take()
    return result


def print_profile(label, total):
    print(f'{label}: instructions by function')
    for name, instructions in total.functions.most_common(16):
        print(f'  {instructions:13d} {100 * instructions / max(1, total.drawing()):5.1f}%  {name}')


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--baseline', type=Path, help='a second patches directory to compare with')
    parser.add_argument('--profile', action='store_true')
    parser.add_argument('--no-inline', action='store_true')
    parser.add_argument('recordings', nargs='+', type=Path)
    options = parser.parse_args()

    with tempfile.TemporaryDirectory() as objects:
        current = Renderer(ROOT / 'patches', objects, not options.no_inline)
        baseline = Renderer(options.baseline, objects, not options.no_inline) if options.baseline else None
        totals, baseline_totals = Result(), Result()
        mismatched = False
        for path in options.recordings:
            try:
                records = load(path)
            except Unsupported as reason:
                print(f'{path.name}: skipped, {reason}')
                continue
            for right_lens in (0, 1):
                result = replay(current, records, right_lens)
                totals.add(result)
                line = (f"{path.name} {'right' if right_lens else 'left'}: {result.columns()} "
                        f"total={result.drawing()} frames={result.frames} failed={result.failures} "
                        f"digest={result.digest.hexdigest()}")
                if baseline:
                    before = replay(baseline, records, right_lens)
                    baseline_totals.add(before)
                    same = (before.digest.digest(), before.frames, before.failures) == \
                        (result.digest.digest(), result.frames, result.failures)
                    mismatched |= not same
                    line += f" baseline={before.drawing()} {'same output' if same else 'OUTPUT DIFFERS'}"
                print(line, flush=True)
        print(f'all: {totals.columns()} total={totals.drawing()} divides={totals.divides}')
        if baseline:
            print(f'baseline: {baseline_totals.columns()} total={baseline_totals.drawing()} divides={baseline_totals.divides}')
            for kind, label in COLUMNS:
                if totals.instructions[kind]:
                    print(f'  {label}: {baseline_totals.instructions[kind] / totals.instructions[kind]:.2f}x fewer instructions')
            print(f'  total: {baseline_totals.drawing() / max(1, totals.drawing()):.2f}x fewer instructions')
        if options.profile:
            print_profile('working tree', totals)
            if baseline:
                print_profile('baseline', baseline_totals)
        return 1 if mismatched or totals.failures else 0


if __name__ == '__main__':
    sys.exit(main())
