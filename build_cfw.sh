#!/usr/bin/env bash
#
# build_cfw.sh — one-shot setup + custom-firmware build for Even Realities G2.
#
# Does everything needed to go from a fresh clone to a flashable image:
#   1. creates a Python virtualenv (./venv) and installs the flasher's deps
#   2. downloads the stock G2 2.2.4.34 firmware from Even's CDN
#   3. verifies the download hashes as expected (refuses to proceed otherwise)
#   4. applies our patches (patches/patch_compress.py) to produce the CFW image
#   5. verifies the patched image hashes as expected
#
# The stock firmware is Even's, so we don't redistribute it — it's fetched at
# build time and patched locally. The build is fully deterministic: the same
# stock input always yields the same patched output (checked against a pinned
# hash), so a successful run proves you got exactly the reviewed image.
#
# Usage:
#   ./build_cfw.sh                  # full setup + build
#   ./build_cfw.sh --skip-venv      # build only, don't touch the venv
#   ./build_cfw.sh --force-download # re-download the stock image even if cached
#   ./build_cfw.sh --help
#
# WARNING: flashing custom firmware voids your warranty and can brick the
# glasses. This script only BUILDS the image; see README.md for flashing.

set -euo pipefail

# ---- config (pinned) -------------------------------------------------------
FW_URL="https://cdn.evenreal.co/firmware/a6966d807634cc97aec641a0dcca358b.bin"
BASE="g2_2.2.4.34.bin"            # stock image (downloaded)
OUT="g2_2.2.4.34_cfw.bin"         # patched image (produced)
PATCH="patches/patch_compress.py"
BASE_SHA256="f9a93621a7141e0ae54ca6371cd2f1b4afbffa61f302ace096e0656ba25b1754"
OUT_SHA256="3b92551cffe297a75e486e036bc0ecc3723fac377ec16fc1f2c1fa81ff4aae22"

SKIP_VENV=0
FORCE_DOWNLOAD=0

# ---- args ------------------------------------------------------------------
for arg in "$@"; do
  case "$arg" in
    --skip-venv)      SKIP_VENV=1 ;;
    --force-download) FORCE_DOWNLOAD=1 ;;
    -h|--help)
      awk 'NR==1{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "$0"
      exit 0 ;;
    *)
      echo "unknown option: $arg (try --help)" >&2
      exit 2 ;;
  esac
done

cd "$(dirname "$0")"

say()  { printf '\033[1;34m==>\033[0m %s\n' "$*"; }
ok()   { printf '\033[1;32m  ✓\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31merror:\033[0m %s\n' "$*" >&2; exit 1; }

# ---- helpers ---------------------------------------------------------------
sha256() {
  if command -v shasum >/dev/null 2>&1; then
    shasum -a 256 "$1" | awk '{print $1}'
  elif command -v sha256sum >/dev/null 2>&1; then
    sha256sum "$1" | awk '{print $1}'
  else
    die "no sha256 tool found (need 'shasum' or 'sha256sum')"
  fi
}

verify() { # path expected_hash label
  local got; got="$(sha256 "$1")"
  [ "$got" = "$2" ] || die "$3 hash mismatch
    expected $2
    got      $got"
}

download() { # url dest
  if command -v curl >/dev/null 2>&1; then
    curl -fL --retry 3 --progress-bar -o "$2" "$1"
  elif command -v wget >/dev/null 2>&1; then
    wget -O "$2" "$1"
  else
    die "no downloader found (need 'curl' or 'wget')"
  fi
}

# ---- 1. venv ---------------------------------------------------------------
if [ "$SKIP_VENV" -eq 1 ]; then
  say "skipping venv setup (--skip-venv)"
else
  command -v python3 >/dev/null 2>&1 || die "python3 not found"
  if [ ! -d venv ]; then
    say "creating virtualenv (./venv)"
    python3 -m venv venv
  else
    say "reusing existing virtualenv (./venv)"
  fi
  say "installing dependencies (bleak, websocket-client)"
  ./venv/bin/python -m pip install --quiet --upgrade pip
  ./venv/bin/python -m pip install --quiet -r requirements.txt
  ok "venv ready — flash with: ./venv/bin/python g2flash.py -c g2://local -f ..."
fi

# ---- 2/3. download + verify stock image ------------------------------------
if [ "$FORCE_DOWNLOAD" -eq 0 ] && [ -f "$BASE" ] && [ "$(sha256 "$BASE")" = "$BASE_SHA256" ]; then
  say "stock image already present and verified ($BASE)"
else
  say "downloading stock G2 2.2.4.34 firmware from Even's CDN"
  download "$FW_URL" "$BASE"
  verify "$BASE" "$BASE_SHA256" "stock firmware"
  ok "stock image verified ($BASE)"
fi

# ---- 4/5. patch + verify ---------------------------------------------------
say "applying CFW patches -> $OUT"
python3 "$PATCH" "$BASE" "$OUT"
verify "$OUT" "$OUT_SHA256" "patched firmware"
ok "patched image verified ($OUT)"

echo
say "done. Custom firmware: $OUT"
echo "    Flash it (voids warranty, can brick the device) with, e.g.:"
echo "      ./venv/bin/python g2flash.py -c g2://local -f $OUT"
echo "    See README.md for connection options and safety notes."
