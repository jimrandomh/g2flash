#!/usr/bin/env python3
"""Send a private CFW message through one lens and verify each target lens ACK.

Requires Faceclaw/5 on both lenses. See docs/message-transport.md.
"""
import argparse
import math
import os
import queue
import secrets
import sys
import time

from g2flash import (Bridge, CTRL, DroidBridgeTransport, LocalBleTransport,
                     crc16, parse_connection_string)


LENS_BITS = {"left": 1, "right": 2, "both": 3}


def make_packet(payload, sequence=7, options=3):
    if not 0 <= sequence <= 255:
        raise ValueError("sequence must be 0..255")
    if not 0 <= options <= 3:
        raise ValueError("options must contain only the left/right bits (0..3)")
    if len(payload) > 252:
        raise ValueError("this probe supports at most 252 payload bytes; fragmentation is not implemented")
    body = bytes((options,)) + payload
    return (bytes((0xaa, 0x21, sequence, len(body) + 2, 1, 1, 0xf0, 0))
            + body + crc16(body))


def check_write_size(packet, max_write):
    if len(packet) > max_write:
        raise ValueError(f"{len(packet)}-byte packet exceeds the {max_write}-byte ATT write limit; "
                         f"use at most {max(0, min(252, max_write - 11))} payload bytes")


def parse_ack(frame):
    """Return (request sequence, lens bit, size, payload CRC), or None.

    The outer sequence belongs to stock TPL; the ACK body echoes our request.
    Validate the entire single-packet envelope before trusting its contents.
    """
    if (len(frame) != 17 or frame[:2] != b'\xaa\x12' or frame[3] != 9
            or frame[4:8] != bytes((1, 1, 0xf0, 0))):
        return None
    body = frame[8:-2]
    if crc16(body) != frame[-2:] or body[0] != 1 or body[2] not in (1, 2):
        return None
    return body[1], body[2], int.from_bytes(body[3:5], 'little'), int.from_bytes(body[5:7], 'little')


def send_probe(transport, packet, bridge_mtu=23, ack_timeout=10):
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
        max_write = bridge_mtu - 3
    check_write_size(packet, max_write)
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
    pending = {bit for bit in (1, 2) if packet[8] & bit}
    payload = packet[9:-2]
    expected_crc = int.from_bytes(crc16(payload), 'little')
    start = time.monotonic()
    transport.write(CTRL[0], CTRL[1], packet.hex(), 1)
    print(f"  submitted {len(packet)} bytes (write limit {max_write} bytes)")
    deadline = start + ack_timeout
    while pending:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        try:
            characteristic, frame = transport.notes.get(timeout=remaining)
        except queue.Empty:
            break
        if characteristic.lower() != CTRL[2]:
            continue
        ack = parse_ack(frame)
        if ack is None:
            continue
        sequence, lens, size, checksum = ack
        if (sequence != packet[2] or lens not in pending or size != len(payload)
                or checksum != expected_crc):
            continue
        pending.remove(lens)
        name = 'left' if lens == 1 else 'right'
        print(f"  {name} ACK: seq {sequence}, size {size}, CRC {checksum:04X}, "
              f"{(time.monotonic() - start) * 1000:.1f} ms")
    if pending:
        names = ', '.join(name for name, bit in LENS_BITS.items() if bit in pending)
        raise TimeoutError(f"missing ACK from {names} after {ack_timeout:g}s")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('-c', '--connection', default=os.environ.get('G2_CONNECTION_STRING'),
                        help="same g2://local or g2://droidbridge URL as g2flash; defaults to G2_CONNECTION_STRING")
    parser.add_argument('--lens', '--via', choices=('left', 'right'), default='left',
                        help="BLE ingress lens (default: left); the peer is reached over the frame bridge")
    parser.add_argument('--targets', choices=('left', 'right', 'both'), default='both',
                        help="lenses that process and ACK the message (default: both)")
    contents = parser.add_mutually_exclusive_group()
    contents.add_argument('--text', default=None, help="UTF-8 payload (default: 123456789)")
    contents.add_argument('--hex', dest='hex_payload', help="hex payload, e.g. '00 01 ff'; '' sends an empty payload")
    parser.add_argument('--seq', type=int, default=None, help="request sequence byte (default: random)")
    parser.add_argument('--ack-timeout', type=float, default=10, help="ACK timeout in seconds (default: 10)")
    parser.add_argument('--bridge-mtu', type=int, default=23,
                        help="known negotiated DroidBridge ATT MTU; does not request a new MTU (default: 23)")
    parser.add_argument('--dry-run', action='store_true', help="print packet and expected overlay; do not connect")
    args = parser.parse_args(argv)
    try:
        payload = (bytes.fromhex(args.hex_payload) if args.hex_payload is not None
                   else (args.text if args.text is not None else '123456789').encode('utf-8'))
        if args.seq is None:
            args.seq = secrets.randbelow(256)
        packet = make_packet(payload, args.seq, LENS_BITS[args.targets])
        if not math.isfinite(args.ack_timeout) or args.ack_timeout <= 0:
            raise ValueError("ACK timeout must be finite and positive")
        if not 23 <= args.bridge_mtu <= 517:
            raise ValueError("bridge MTU must be 23..517")
        if not args.dry_run and not args.connection:
            raise ValueError("supply --connection or G2_CONNECTION_STRING (see --help)")
        connection = parse_connection_string(args.connection) if not args.dry_run else None
    except ValueError as error:
        parser.error(str(error))

    print(f"SID 0xf0, {len(payload)} payload bytes, sequence {args.seq}, "
          f"via {args.lens}, targets {args.targets}")
    print(f"packet: {packet.hex(' ')}")
    print(f"expected overlay: rx {len(payload)} crc {int.from_bytes(crc16(payload), 'little'):04X}")
    if args.dry_run:
        return 0

    bridge = None
    try:
        if connection['method'] == 'droidbridge':
            # Reject an oversized packet before changing any connection state.
            check_write_size(packet, args.bridge_mtu - 3)
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
            send_probe(transport, packet, args.bridge_mtu, args.ack_timeout)
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
    print("All selected lenses acknowledged processing the payload.")
    print("Enable the debug overlay and render a normal custom frame to check the size and CRC.")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
