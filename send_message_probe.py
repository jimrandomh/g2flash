#!/usr/bin/env python3
"""Send a private CFW message through one lens and verify each target lens ACK.

Requires Faceclaw/10 on both lenses; the default payload is a mode-7 no-op. See docs/message-transport.md.
"""
import argparse
import math
import os
import queue
from pathlib import Path
import secrets
import sys
import time
import zlib

from g2flash import (Bridge, CTRL, DroidBridgeTransport, LocalBleTransport,
                     crc16, parse_connection_string)


LENS_BITS = {"left": 1, "right": 2, "both": 3}


def make_packet(chunk, sequence=7, options=3, *, reset=False, end=False):
    """Wrap stream bytes; a packet boundary need not be a message boundary."""
    if not 0 <= sequence <= 255:
        raise ValueError("sequence must be 0..255")
    if not 0 <= options <= 3:
        raise ValueError("options must contain only the left/right bits (0..3)")
    if len(chunk) > 252:
        raise ValueError("a packet holds at most 252 stream bytes")
    body = bytes((options | (0x80 if reset else 0) | (0x40 if end else 0),)) + chunk
    return (bytes((0xaa, 0x21, sequence, len(body) + 2, 1, 1, 0xf0, 0))
            + body + crc16(body))


def make_packets(messages, mtu=23, sequence=7, options=3, *, max_write=None):
    if not 23 <= mtu <= 517:
        raise ValueError("MTU must be 23..517")
    if not 1 <= len(messages) <= 65536:
        raise ValueError("send 1..65536 messages per stream")
    if any(len(message) > 65535 for message in messages):
        raise ValueError("each message must fit its two-byte length tag (0..65535 bytes)")
    compressor = zlib.compressobj()
    records = []
    reset_context = True
    for message in messages:
        compressed = compressor.compress(message) + compressor.flush(zlib.Z_SYNC_FLUSH)
        flags = options | 4 | (8 if reset_context else 0)
        reset_context = False
        if len(compressed) > 65535:
            compressed = message
            flags = options | 8
            compressor = zlib.compressobj()
            reset_context = True
        records.append(bytes((flags,)) + len(compressed).to_bytes(2, 'little') + crc16(message) + compressed)
    stream = b''.join(records)
    write_limit = mtu - 3 if max_write is None else min(mtu - 3, max_write)
    capacity = min(252, write_limit - 11)
    if capacity < 1:
        raise ValueError("ATT write limit must allow at least one stream byte (12 bytes total)")
    # Validate even when the first packet's sequence would otherwise wrap.
    make_packet(b'', sequence, options)
    return [make_packet(stream[offset:offset + capacity], (sequence + index) & 255, options,
                        reset=offset == 0, end=offset + capacity >= len(stream))
            for index, offset in enumerate(range(0, len(stream), capacity))]


def parse_ack(frame):
    """Return (stream ID, message ordinal, lens bit, size, payload CRC), or None."""
    if (len(frame) != 19 or frame[:2] != b'\xaa\x12' or frame[3] != 11
            or frame[4:8] != bytes((1, 1, 0xf0, 0))):
        return None
    body = frame[8:-2]
    if crc16(body) != frame[-2:] or body[0] not in (1, 3) or body[4] not in (1, 2):
        return None
    return (body[1], int.from_bytes(body[2:4], 'little'), body[4],
            int.from_bytes(body[5:7], 'little'), int.from_bytes(body[7:9], 'little'))


def send_probe(transport, messages, mtu=23, ack_timeout=10, sequence=7, options=3):
    transport.connect()
    if not transport.discover():
        raise RuntimeError("service discovery failed")
    if isinstance(transport, LocalBleTransport):
        characteristic = transport.client.services.get_characteristic(CTRL[1])
        if characteristic is None:
            raise RuntimeError(f"G2 command characteristic {CTRL[1]} not found")
        if 'write-without-response' not in characteristic.properties:
            raise RuntimeError("command characteristic does not support write without response")
        max_write = characteristic.max_write_without_response_size
    else:
        services = transport.br.services(transport.address).get('services', [])
        found = any(s['uuid'].lower() == CTRL[0] and any(
            c['uuid'].lower() == CTRL[1] and c['properties'] & 0x04
            for c in s.get('characteristics', [])) for s in services)
        if not found:
            raise RuntimeError("G2 command characteristic with write-without-response support not found")
        # DroidBridge does not expose the negotiated MTU in its services API.
        # Default to ATT's minimum; allow a user-supplied known MTU for bigger probes.
        max_write = mtu - 3
    packets = make_packets(messages, mtu, sequence, options, max_write=max_write)
    transport.set_notify(CTRL[0], CTRL[2], True)
    if not isinstance(transport, LocalBleTransport):
        # DroidBridge /notify returns before the CCCD descriptor write completes
        # and exposes no completion event. Match the flasher's one-time setup
        # grace period; this delay is outside message/ACK timing.
        time.sleep(2.5)
    # Discard notifications predating this request; sequence and metadata checks
    # below also reject delayed unrelated replies. No automatic retry/dedup yet.
    while True:
        try:
            transport.notes.get_nowait()
        except queue.Empty:
            break
    pending = {(index, bit): (len(message), int.from_bytes(crc16(message), 'little'))
               for index, message in enumerate(messages) for bit in (1, 2) if options & bit}
    start = time.monotonic()

    def receive_ack(characteristic, frame):
        if characteristic.lower() != CTRL[2]:
            return
        ack = parse_ack(frame)
        if ack is None:
            return
        stream_id, message_id, lens, size, checksum = ack
        if frame[8] == 3 and stream_id == sequence and (message_id, lens) in pending:
            raise RuntimeError(f"NACK: stream {stream_id}, message {message_id}, lens {lens}; reset context before retry")
        key = (message_id, lens)
        if stream_id != sequence or pending.get(key) != (size, checksum):
            return
        del pending[key]
        name = 'left' if lens == 1 else 'right'
        print(f"  {name} ACK: stream {stream_id}, message {message_id}, size {size}, "
              f"CRC {checksum:04X}, {(time.monotonic() - start) * 1000:.1f} ms")

    for packet in packets:
        transport.write(CTRL[0], CTRL[1], packet.hex(), 1)
        # Replies can arrive while a later message is still being sent.
        while True:
            try:
                receive_ack(*transport.notes.get_nowait())
            except queue.Empty:
                break
    print(f"  submitted {len(messages)} messages in {len(packets)} packets "
          f"({sum(map(len, packets))} write bytes; write limit {min(max_write, mtu - 3)} bytes)")
    # Large transfers can take longer than the final-ACK grace period.
    deadline = time.monotonic() + ack_timeout
    while pending:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            receive_ack(*transport.notes.get(timeout=remaining))
        except queue.Empty:
            break
    if pending:
        missing = ', '.join(f"message {index}/{'left' if lens == 1 else 'right'}"
                            for index, lens in list(pending)[:8])
        raise TimeoutError(f"missing {len(pending)} ACK(s): {missing} after {ack_timeout:g}s")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-c', '--connection', default=os.environ.get('G2_CONNECTION_STRING'),
                        help="same g2://local or g2://droidbridge URL as g2flash; defaults to G2_CONNECTION_STRING")
    parser.add_argument('--lens', '--via', choices=('left', 'right'), default='left',
                        help="BLE ingress lens (default: left); the peer is reached over the frame bridge")
    parser.add_argument('--targets', choices=('left', 'right', 'both'), default='both',
                        help="lenses that process and ACK the message (default: both)")
    contents = parser.add_mutually_exclusive_group()
    contents.add_argument('--text', action='append', help="UTF-8 message; repeat for multiple messages (default payload: hex 07ff no-op)")
    contents.add_argument('--hex', dest='hex_payload', action='append', help="hex message; repeat for multiple messages; '' sends an empty message")
    contents.add_argument('--file', action='append', help="binary message file; repeat for multiple messages")
    parser.add_argument('--repeat', type=int, default=1, help="repeat the supplied message list (default: 1)")
    parser.add_argument('--seq', type=int, default=None, help="initial packet sequence and stream ID (default: random)")
    parser.add_argument('--ack-timeout', type=float, default=10, help="final ACK timeout after sending all packets, in seconds (default: 10)")
    parser.add_argument('--mtu', '--bridge-mtu', type=int, default=23,
                        help="ATT MTU used for packet splitting; does not negotiate MTU (default: 23); "
                             "local BLE also caps writes at its actual limit")
    parser.add_argument('--dry-run', action='store_true', help="print all split packets and expected final overlay; do not connect")
    args = parser.parse_args(argv)
    try:
        if args.hex_payload is not None:
            messages = [bytes.fromhex(value) for value in args.hex_payload]
        elif args.file is not None:
            messages = [Path(name).read_bytes() for name in args.file]
        elif args.text is not None:
            messages = [value.encode('utf-8') for value in args.text]
        else:
            messages = [bytes([7, 255])]
        if not 1 <= args.repeat <= 65536 or len(messages) * args.repeat > 65536:
            raise ValueError("repeat must produce 1..65536 messages")
        messages *= args.repeat
        if args.seq is None:
            args.seq = secrets.randbelow(256)
        packets = make_packets(messages, args.mtu, args.seq, LENS_BITS[args.targets])
        if not math.isfinite(args.ack_timeout) or args.ack_timeout <= 0:
            raise ValueError("ACK timeout must be finite and positive")
        if not args.dry_run and not args.connection:
            raise ValueError("supply --connection or G2_CONNECTION_STRING (see --help)")
        connection = parse_connection_string(args.connection) if not args.dry_run else None
    except (ValueError, OSError) as error:
        parser.error(str(error))

    print(f"SID 0xf0, {len(messages)} messages, {sum(map(len, messages))} payload bytes, "
          f"stream {args.seq}, via {args.lens}, targets {args.targets}, MTU {args.mtu}")
    if args.dry_run:
        for index, packet in enumerate(packets):
            print(f"packet {index}: {packet.hex(' ')}")
    last = messages[-1]
    print(f"expected final overlay: rx {len(last)} crc {int.from_bytes(crc16(last), 'little'):04X}")
    if args.dry_run:
        return 0

    bridge = None
    try:
        if connection['method'] == 'droidbridge':
            bridge = Bridge(connection['base'], connection['token'])
            bridge.start_ws()
            deadline = time.monotonic() + args.ack_timeout
            while not bridge.ws_open:
                if time.monotonic() >= deadline:
                    raise TimeoutError("DroidBridge notification websocket did not open")
                time.sleep(0.05)
        lens = args.lens
        print(f"{lens}: connecting")
        transport = (DroidBridgeTransport(bridge, connection[lens]) if bridge else
                     LocalBleTransport(connection[lens], connection['address_type'], side=lens))
        try:
            send_probe(transport, messages, args.mtu, args.ack_timeout, args.seq, LENS_BITS[args.targets])
        finally:
            transport.close()
    except Exception as error:
        print(f"probe failed: {error}", file=sys.stderr)
        return 1
    finally:
        if bridge:
            bridge.close()
            if getattr(bridge, 'ws', None) is not None:
                bridge.ws.close()
    print("All selected lenses acknowledged every message.")
    print("Enable the debug overlay and render a normal custom frame to check the size and CRC.")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
