# g2flash

`g2flash.py` flashes firmware onto Even Realities G2 smart glasses by
reimplementing the official app's BLE flash protocol. It is the tool used to
push custom firmware (a patched `*_cfw.bin` image) onto the device.

> **WARNING — this voids your warranty and can brick the glasses.**
> Flashing custom firmware over the OTA path carries a real risk of bricking
> the device. The tool makes you type `my warranty is void` at an interactive
> prompt before it will write anything (use `--my-warranty-is-void` to skip the
> prompt for automation). Only proceed if you understand and accept the risk.

## Quick start

```bash
cd g2flash
./build_cfw.sh                       # set up venv, download stock fw, patch, verify
./venv/bin/python g2flash.py -c g2://local -f g2_2.2.6.10_cfw.bin
```

`build_cfw.sh` does the whole build: it creates `./venv` with the flasher's
dependencies, downloads the stock **G2 2.2.6.10** firmware from Even's CDN,
applies the patches in `patches/`, and verifies that both the download and the
patched result match pinned SHA-256 hashes (so a clean run proves you got
exactly the reviewed image). Run `./build_cfw.sh --help` for options
(`--skip-venv`, `--force-download`). Then flash as shown above — see
[Usage](#usage) for connection strings and safety flags.

## What's the custom firmware?

The patches in `patches/` add image/display features on top of stock 2.2.6.10:

- **576×288 image containers** (stock caps at 288×144).
- **zlib-compressed image payloads** and **8bpp XOR-delta** frame updates, for
  much faster image/video streaming.
- **Per-lens stereo image pairs**.
- A **capability-advertisement field** on the settings response, so a connected
  app can detect this firmware and which features it supports.

## What's in this directory

- `g2flash.py` — the flasher.
- `build_cfw.sh` — one-shot venv setup + download + patch + verify (see above).
- `patches/` — the patch sources and tools:
  - `cfw_patches.json` — the **committed patch set**: a list of
    offset/expected-old/new byte patches (plus base + output SHA-256s) that turns
    the stock image into the CFW image. This is the source of truth for the build.
  - `apply_patches.py` — replays `cfw_patches.json` onto the stock image. **No
    compiler** — pure Python stdlib, so it runs anywhere (a phone, a fresh box).
    This is what `build_cfw.sh` uses to produce the image.
  - `gen_patches.py` — compiles the injected code with **clang** and (re)generates
    `cfw_patches.json`. Run it after editing the patch sources:
    `python3 patches/gen_patches.py g2_2.2.6.10.bin patches/cfw_patches.json`
    (or `./build_cfw.sh --update-patches`), then commit the JSON.
  - `patch_compress.py` — the all-in-one patcher (576 lift + image compression
    + stereo + capability field); `gen_patches.py` calls it to build the ops.
    Holds every stock-firmware address the patches depend on; see
    `notes/fw-2.2.6.10-cfw-rebase.md` for how they were derived.
  - `patch_img_container_576.py` — standalone tool for just the 576×288 lift.
    NOTE: still targets the old 2.2.4.34 base and is not part of the build.
  - `build.py`, `*.c` — the C→position-independent-Thumb pipeline and sources
    for the injected firmware code (compiled by `gen_patches.py`; the resulting
    machine code lands in `cfw_patches.json`).
- `requirements.txt` — the flasher's Python dependencies.

Firmware images (`g2_2.2.6.10*.bin`) are **not** checked in — they are Even's
firmware, so you build them locally with `build_cfw.sh`.

## Requirements

- Python 3.x (developed against the Homebrew `python@3.14` build).
- One of two transports to reach the glasses:
  - **local** — this machine's own Bluetooth radio, via the `bleak` package.
  - **droidbridge** — a bonded Android phone running
    [DroidBridge](../droidbridge) that forwards GATT over HTTP/WebSocket; uses
    the `websocket-client` package.

Third-party Python dependencies:

| Package            | Needed for                          | Imported as |
|--------------------|-------------------------------------|-------------|
| `bleak`            | `g2://local` transport              | `bleak`     |
| `websocket-client` | `g2://droidbridge` transport        | `websocket` |

Both are imported lazily, so you only need to install the one for the transport
you actually use. Firmware parsing, validation, and `--recompute-checksums` run
on the standard library alone.

## Setting up the venv

`build_cfw.sh` creates and populates `./venv` for you as part of a normal run.
To set it up by hand instead:

```bash
cd g2flash
python3 -m venv venv
./venv/bin/python -m pip install --upgrade pip
./venv/bin/python -m pip install -r requirements.txt
```

With the venv activated (`source venv/bin/activate`) you can invoke the tool as
`python g2flash.py ...`; otherwise use `./venv/bin/python g2flash.py ...`. Run
`deactivate` to leave the environment.

### macOS note

On macOS, `bleak` talks to CoreBluetooth, which never exposes BLE MAC
addresses — scanned addresses are random per-host UUIDs. `g2flash` works around
this by scanning and matching the last three MAC bytes embedded in the arm's
advertised name (`Even G2_32_L_693CCB`). For a local flash the arm must be
powered on and **not** connected to the phone (quit the Even app / turn off the
phone's Bluetooth) so it advertises for a direct connection. The first time you
run it, macOS will prompt to grant your terminal Bluetooth permission.

## Usage

```
python g2flash.py -c <connection-string> -f <firmware.bin> [options]
```

Connection strings:

```
# direct from this machine's Bluetooth radio (needs bleak)
g2://local?left=<addr>&right=<addr>&addressType=public|random

# through a bonded phone running DroidBridge (needs websocket-client)
g2://droidbridge?phone=<host>&port=<port>&token=<tok>&left=<mac>&right=<mac>
```

`addressType=public` is a normal MAC (`D0:7A:47:82:09:67`); `random` is the
macOS/CoreBluetooth peripheral-UUID style.

Common options:

- `--lens left|right|both` — which arm to flash (default `both`).
- `--stop-before discover|heartbeat|file_check|flash|done` — dry-run gate that
  halts before the named stage; use it to test connectivity without writing.
- `--my-warranty-is-void` — skip the interactive warranty confirmation.
- `--component-retries N` / `--block-nak-retries N` — transfer retry tuning.
- `--debug` — print received BLE frames.

`--recompute-checksums IMAGE` rewrites an image's stored checksums in place
(component CRC32C + mainApp preamble CRC32) to match its current payloads and
exits without connecting. Run it after any length-preserving binary patch —
otherwise the glasses reject the component on END with status 7 (CHECK_FAIL).

### Examples

```bash
# dry run: connect to both arms over the local radio and stop before any write
python g2flash.py \
  -c 'g2://local?left=AA:BB:CC:11:22:33&right=AA:BB:CC:44:55:66&addressType=public' \
  -f g2_2.2.6.10.bin --stop-before flash

# fix checksums after patching, no device needed
python g2flash.py --recompute-checksums g2_2.2.6.10_cfw.bin

# flash the custom firmware to both arms via DroidBridge
python g2flash.py \
  -c 'g2://droidbridge?phone=192.168.1.50&port=8080&token=secret&left=AA:BB:CC:11:22:33&right=AA:BB:CC:44:55:66' \
  -f g2_2.2.6.10_cfw.bin
```

## How it works (brief)

The flasher speaks the same `aa21`-framed envelope protocol as the official
app, validated byte-for-byte against a real flash capture. The firmware image
is an EVENOTA container of five components; each is streamed over the firmware
data service (`...e1001`) as a FILE_CHECK subheader followed by 4 KB blocks,
then an END check the glasses verify against a per-component CRC32C. A heartbeat
on the EvenHub control service (`...e5450`) keeps the session alive during the
transfer. Arms are flashed one at a time. See the module docstring and comments
in `g2flash.py` for the wire-level details and the retry/recovery rationale.
