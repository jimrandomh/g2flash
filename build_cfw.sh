#!/usr/bin/env bash
#
# build_cfw.sh — one-shot setup + custom-firmware build for Even Realities G2.
#
# Does everything needed to go from a fresh clone to a flashable image:
#   1. creates a Python virtualenv (./venv) and installs the flasher's deps
#   2. downloads the stock G2 2.2.6.10 firmware from Even's CDN
#   3. verifies the download hashes as expected (refuses to proceed otherwise)
#   4. applies the committed patch set (patches/cfw_patches.json) to the stock
#      image to produce the CFW image
#   5. verifies the patched image hashes as expected
#
# The patch set is split into two halves:
#   * patches/gen_patches.py compiles the injected code with clang and emits the
#     committed patches/cfw_patches.json (a list of offset/old/new byte patches).
#   * patches/apply_patches.py replays that JSON onto the stock image with NO
#     compiler — pure Python stdlib, so it runs anywhere (a phone, a fresh box).
# The build always APPLIES the committed JSON, so the pinned output hash is met
# regardless of what clang is (or isn't) installed. When a working clang cross-
# compiler is present we additionally REGENERATE the patch set and check it still
# matches the committed one — a reproducibility check. If clang is missing or
# can't cross-compile, we warn and skip that check rather than failing.
#
# The stock firmware is Even's, so we don't redistribute it — it's fetched at
# build time and patched locally. The build is fully deterministic: the committed
# JSON + the pinned output hash mean a successful run reproduces exactly the
# reviewed image.
#
# Usage:
#   ./build_cfw.sh                  # full setup + build
#   ./build_cfw.sh --skip-venv      # build only, don't touch the venv
#   ./build_cfw.sh --force-download # re-download the stock image even if cached
#   ./build_cfw.sh --update-patches # regenerate & OVERWRITE patches/cfw_patches.json
#                                   #   from clang (use after editing patch sources)
#   ./build_cfw.sh --help
#
# WARNING: flashing custom firmware voids your warranty and can brick the
# glasses. This script only BUILDS the image; see README.md for flashing.

set -euo pipefail

# ---- config (pinned) -------------------------------------------------------
FW_URL="https://cdn.evenreal.co/firmware/e28738432d7b612d625331b00383149b.bin"
BASE="g2_2.2.6.10.bin"            # stock image (downloaded)
OUT="g2_2.2.6.10_cfw.bin"         # patched image (produced)
PATCH_JSON="patches/cfw_patches.json"   # committed patch set (applied to produce OUT)
GEN="patches/gen_patches.py"      # clang: (re)generate the patch set
APPLY="patches/apply_patches.py"  # no clang: replay the patch set onto BASE
BASE_SHA256="f4dfb0b49ad3de3c2daf17f8a27a157c3dc98411d6a0d3ab2cfd0918f41b9afa"
OUT_SHA256="5c1539fd39c599e6035f6a8ec0779ba687c250d342a24c21a39952fed6c56aa0"

SKIP_VENV=0
FORCE_DOWNLOAD=0
UPDATE_PATCHES=0

# ---- args ------------------------------------------------------------------
for arg in "$@"; do
  case "$arg" in
    --skip-venv)      SKIP_VENV=1 ;;
    --force-download) FORCE_DOWNLOAD=1 ;;
    --update-patches) UPDATE_PATCHES=1 ;;
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
warn() { printf '\033[1;33m  !\033[0m %s\n' "$*"; }
die()  { printf '\033[1;31merror:\033[0m %s\n' "$*" >&2; exit 1; }

# Does clang look like a working thumbv7em cross-compiler? (compile a tiny TU)
clang_cross_ok() {
  command -v clang >/dev/null 2>&1 || return 1
  local tmp; tmp="$(mktemp -d)"
  printf 'int f(int x){return x+1;}\n' > "$tmp/probe.c"
  local rc=0
  clang --target=thumbv7em-none-eabi -mthumb -ffreestanding -c \
        "$tmp/probe.c" -o "$tmp/probe.o" >/dev/null 2>&1 || rc=1
  rm -rf "$tmp"
  return $rc
}

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
  say "downloading stock G2 2.2.6.10 firmware from Even's CDN"
  download "$FW_URL" "$BASE"
  verify "$BASE" "$BASE_SHA256" "stock firmware"
  ok "stock image verified ($BASE)"
fi

# ---- 4a. (re)generate patch set with clang, if available -------------------
# The committed patches/cfw_patches.json is the source of truth; clang is only
# needed to (re)generate it. By default we regenerate to a temp file and check it
# still matches the committed set (a reproducibility check). --update-patches
# overwrites the committed file (developer workflow after editing patch sources).
[ -f "$PATCH_JSON" ] || [ "$UPDATE_PATCHES" -eq 1 ] || \
  die "committed patch set missing ($PATCH_JSON) and --update-patches not given"

if [ "$UPDATE_PATCHES" -eq 1 ]; then
  clang_cross_ok || die "--update-patches needs a working clang thumbv7em cross-compiler"
  say "regenerating committed patch set with clang ($PATCH_JSON)"
  python3 "$GEN" "$BASE" "$PATCH_JSON"
  ok "patch set regenerated — review & commit $PATCH_JSON"
elif clang_cross_ok; then
  say "clang found — regenerating patch set to check reproducibility"
  TMP_JSON="$(mktemp)"
  if python3 "$GEN" "$BASE" "$TMP_JSON" >/dev/null 2>&1; then
    if diff -q "$PATCH_JSON" "$TMP_JSON" >/dev/null 2>&1; then
      ok "clang reproduces the committed patch set exactly"
    else
      warn "clang output differs from the committed patch set (likely a clang"
      warn "version difference) — using the committed $PATCH_JSON, which is pinned."
      warn "If you meant to change the patches, run: ./build_cfw.sh --update-patches"
    fi
  else
    warn "clang patch regeneration failed — using the committed $PATCH_JSON"
  fi
  rm -f "$TMP_JSON"
else
  warn "no working clang cross-compiler — skipping regeneration"
  warn "using the committed $PATCH_JSON (no compiler needed to apply it)"
fi

# ---- 4b/5. apply committed patch set + verify ------------------------------
say "applying committed patch set -> $OUT"
python3 "$APPLY" "$BASE" "$PATCH_JSON" "$OUT"
verify "$OUT" "$OUT_SHA256" "patched firmware"
ok "patched image verified ($OUT)"

echo
say "done. Custom firmware: $OUT"
echo "    Flash it (voids warranty, can brick the device) with, e.g.:"
echo "      ./venv/bin/python g2flash.py -c g2://local -f $OUT"
echo "    See README.md for connection options and safety notes."
