#!/usr/bin/env python3
"""
Generate the committed CFW patch set (patches/cfw_patches.json) from the stock
G2 firmware image. This is the clang half of the build: it compiles the injected
code blobs (via patch_compress.build_patch_ops -> build.py -> clang) and writes a
JSON list of (offset, expected-old-bytes, new-bytes) patches plus the base and
output SHA-256s.

The resulting JSON is committed to git and is the source of truth for the build.
apply_patches.py replays it onto the stock image with no compiler at all, so the
exact reviewed firmware can be rebuilt on any machine — including ones that can't
run a matching clang cross-compiler (a phone, a fresh CI box, etc.). Because the
op list is derived from clang output, a different clang version will produce a
different JSON; that's expected, and it's why the JSON — not the compiler — is the
committed artifact.

Usage:
  python3 gen_patches.py <base.bin> <out.json>

Regenerate the committed patch set after changing any of the injected C sources:
  python3 patches/gen_patches.py g2_2.2.4.34.bin patches/cfw_patches.json
(then update OUT_SHA256 in build_cfw.sh if the output hash changed, and commit).
"""
import sys, os, json, hashlib

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)

from patch_compress import build_patch_ops


def main():
    if len(sys.argv) != 3:
        print("usage: gen_patches.py <base.bin> <out.json>", file=sys.stderr)
        sys.exit(2)
    base_path, out_path = sys.argv[1], sys.argv[2]

    img = open(base_path, "rb").read()
    print("compiling injected blobs (build.py):")
    data, ops = build_patch_ops(img)

    spec = {
        "base": os.path.basename(base_path),
        "base_sha256": hashlib.sha256(img).hexdigest(),
        "output_sha256": hashlib.sha256(data).hexdigest(),
        "patches": ops,
    }
    with open(out_path, "w") as f:
        json.dump(spec, f, indent=2)
        f.write("\n")

    total = sum(len(bytes.fromhex(op["new"])) for op in ops)
    print(f"wrote {out_path}: {len(ops)} patches, {total} bytes changed/appended")
    print(f"  base   sha256 {spec['base_sha256']}")
    print(f"  output sha256 {spec['output_sha256']}")


if __name__ == "__main__":
    main()
