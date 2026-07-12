#!/usr/bin/env python3
"""
Apply a committed CFW patch set (patches/cfw_patches.json) to the stock G2
firmware image. This is the clang-free half of the build: it takes the stock
base image plus a JSON list of (offset, expected-old-bytes, new-bytes) patches
and writes the patched image. No compiler, no C sources, nothing but stdlib —
so it runs anywhere Python does, including on a phone.

The patch JSON is produced by patches/gen_patches.py (which DOES need clang) and
committed to git, so the exact reviewed image can be rebuilt on any machine
without reproducing clang's byte-for-byte output.

Usage:
  python3 apply_patches.py <base.bin> <patches.json> <out.bin>

JSON schema (see gen_patches.py):
  {
    "base":          "g2_2.2.4.34.bin",
    "base_sha256":   "<hex>",              # verified against <base.bin> up front
    "output_sha256": "<hex>",              # verified against <out.bin> at the end
    "patches": [
      {"offset": <int>, "old": "<hex>", "new": "<hex>", "desc": "..."},
      ...
      # the final patch is usually an append: "old" is "" and "offset" == len(base)
    ]
  }

Each patch writes `new` at `offset` after checking the bytes already there equal
`old` (a stock-image sanity check + double-apply guard). A patch whose `old` is
empty writes at (or extends) the end of the file — that's how the appended code
blob is represented.
"""
import sys, json, hashlib


class ApplyError(Exception):
    pass


def _sha256(b):
    return hashlib.sha256(b).hexdigest()


def apply_ops(img, ops):
    """Apply the list of {offset, old, new[, desc]} patches to `img` (bytes) and
    return the patched bytes. Raises ApplyError on any expected-bytes mismatch."""
    buf = bytearray(img)
    for i, op in enumerate(ops):
        off = op["offset"]
        old = bytes.fromhex(op.get("old", ""))
        new = bytes.fromhex(op["new"])
        desc = op.get("desc", "")
        tag = f"patch #{i} @ {off:#x}" + (f" ({desc})" if desc else "")

        if old:
            # in-place edit: verify the stock bytes are present, then overwrite.
            cur = bytes(buf[off:off + len(old)])
            if cur == new and cur != old:
                continue  # already applied — idempotent
            if cur != old:
                raise ApplyError(
                    f"{tag}: expected {old.hex()} but found {cur.hex()}.\n"
                    "    The input is not the stock base this patch set targets "
                    "(or it is already partially patched).")
            buf[off:off + len(new)] = new
        else:
            # append / write-at-EOF: `old` is empty, so there's nothing to verify
            # against — file-level base_sha256 is what guards correctness here.
            end = off + len(new)
            if off <= len(buf) and bytes(buf[off:end]) == new and end <= len(buf):
                continue  # already applied — idempotent
            if off != len(buf):
                raise ApplyError(
                    f"{tag}: append expects offset {off:#x} == current length "
                    f"{len(buf):#x}. The input is not the stock base this patch "
                    "set targets.")
            buf.extend(new)
    return bytes(buf)


def main():
    if len(sys.argv) != 4:
        print(__doc__.strip().split("Usage:", 1)[1].strip().splitlines()[0]
              if "Usage:" in __doc__ else "usage: apply_patches.py base json out",
              file=sys.stderr)
        print("usage: apply_patches.py <base.bin> <patches.json> <out.bin>",
              file=sys.stderr)
        sys.exit(2)
    base_path, json_path, out_path = sys.argv[1:4]

    img = open(base_path, "rb").read()
    spec = json.load(open(json_path))
    ops = spec["patches"]

    exp_base = spec.get("base_sha256")
    if exp_base:
        got = _sha256(img)
        if got != exp_base:
            print(f"error: base image hash mismatch\n"
                  f"    expected {exp_base}\n    got      {got}\n"
                  f"    ({base_path} is not the stock image these patches target)",
                  file=sys.stderr)
            sys.exit(1)
        print(f"  base verified ({base_path})")

    try:
        out = apply_ops(img, ops)
    except ApplyError as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)

    exp_out = spec.get("output_sha256")
    if exp_out:
        got = _sha256(out)
        if got != exp_out:
            print(f"error: patched image hash mismatch\n"
                  f"    expected {exp_out}\n    got      {got}",
                  file=sys.stderr)
            sys.exit(1)

    open(out_path, "wb").write(out)
    print(f"  applied {len(ops)} patches -> {out_path} ({len(out)} bytes)")


if __name__ == "__main__":
    main()
