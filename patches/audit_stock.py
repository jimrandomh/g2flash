"""Fail closed if a stock dependency or patch footprint escapes the 2.3.0 audit.

The JSON is reviewed evidence, not an automatically discovered offset map.
Generation intentionally never updates it. Run after editing C sources to catch
new or stale absolute dependencies before compiling a flashable manifest.
"""
from collections import Counter
import hashlib
import json
from pathlib import Path
import re

PATCHES = Path(__file__).resolve().parent
AUDIT = PATCHES / 'stock_abi_230.json'


def source_literals(directory=PATCHES):
    result = {}
    for path in sorted(directory.iterdir()):
        if path.suffix not in ('.c', '.h'):
            continue
        text = re.sub(r'/\*.*?\*/|//[^\n]*', '', path.read_text(), flags=re.S)
        values = Counter(int(s, 16) for s in re.findall(r'\b0x[0-9a-fA-F]+\b',
            # C integer suffixes are not part of the address.
            re.sub(r'(0x[0-9a-fA-F]+)[uUlL]+\b', r'\1', text)))
        values = {f'0x{a:08x}': n for a, n in sorted(values.items())
                  if 0x00438000 <= a < 0x00800000 or 0x20000000 <= a < 0x20400000}
        if values:
            result[path.name] = values
    return result


def validate_stock(img):
    audit = json.loads(AUDIT.read_text())
    if hashlib.sha256(img).hexdigest() != audit['base_sha256']:
        raise ValueError('stock ABI audit requires the exact G2 2.3.0.24 donor')
    if source_literals() != audit['source_literals']:
        raise ValueError('firmware address inventory changed: review stock_abi_230.json before building')
    delta = int(audit['file_delta'], 16)
    for guard in audit['guards']:
        off = int(guard['address'], 16) - delta
        expected = bytes.fromhex(guard['bytes'])
        if off < 0 or img[off:off + len(expected)] != expected:
            raise ValueError(f"stock ABI guard mismatch: {guard['label']}")
    return audit


def validate_footprint(img, in_place, audit):
    """Each live write is reviewed, bounded, disjoint, and has pinned stock bytes."""
    actual = []
    spans = []
    for offset, old, new, description in in_place:
        old, new = bytes.fromhex(old), bytes.fromhex(new)
        if offset < 0 or not new or len(old) < len(new) or offset + len(old) > len(img):
            raise ValueError(f'invalid in-place patch: {description}')
        if img[offset:offset + len(old)] != old:
            raise ValueError(f'stock bytes differ: {description}')
        spans.append((offset, offset + len(new)))
        actual.append({'address': f'0x{offset + int(audit["file_delta"], 16):08x}',
                       'old': old.hex(), 'write_size': len(new)})
    spans.sort()
    if any(a[1] > b[0] for a, b in zip(spans, spans[1:])):
        raise ValueError('overlapping live firmware patches')
    if actual != audit['patch_sites']:
        raise ValueError('live patch footprint changed: review stock_abi_230.json before building')
