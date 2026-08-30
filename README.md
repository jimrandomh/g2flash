# g2flash

`g2flash` is a utility for installing firmware on Even Realities G2 smart
glasses, as well as a collection of firmware modifications that add features and
fix limitations of the glasses. The modifications themselves are made with
[Faceclaw](https://github.com/jimrandomh/faceclaw) in mind, but can be used by
any software that communicates with the G2 glasses using their BLE protocol. If
you are using this with Faceclaw, you can either use this tool to apply the
firmware mods, or use the flashing tool built in to Faceclaw's onboarding
process (both versions install the same firmware image). Installing using
Faceclaw is a bit more user friendly than installing using g2flash.

This repository contains patches applied to firmware, but does not contain the
Even Realities firmware itself. The `build_cfw.sh` script will download the base
firmware from Even's CDN, apply patches, and verify that the resulting firmware
has the expected hash for you.

FLASHING A CUSTOM FIRMWARE WILL VOID YOUR WARRANTY. This tool will require you
to acknowledge that you are voiding your warranty when you run it.


## Modifications

This firmware reworks how images and screen updates work in EvenHub. The
intended usage is that you create a layout with a single 576x288 image
container, which is used as a message target (but the EvenHub layout system is
otherwise entirely ignored). Image updates sent to this container are
interpreted as custom messages of new types. Image traffic is compressed with
zlib+RLE. Screen contents can be up to 640x480 (larger than the screen area
supported by the stock firmware), you can update dirty rects rather than
updating the whole screen at once, and you can send messages which perform
rect-to-rect copies for low-bandwidth scroll animations. Because this mode
writes directly to the framebuffer without going through EvenHub's
screen-update functions, you cannot mix this mode with EvenHub list or text or
list containers. A lease-scoped 64 KiB texture cache lets the phone upload RLE
icons and glyphs once, then draw cached images and strings with small update
messages. The cache is allocated and zeroed on its first write and released
when the Faceclaw framebuffer lease ends. Cached draw commands carry an options
byte whose low nibble selects the top output color; bit 4 makes source color 0
transparent, and bit 5 reverses the proportional 16-entry color ramp.
Image-handler mode 15 draws a length-prefixed UTF-8 string with the glasses'
built-in 20 px font chain and its default pair kerning. Its payload after the
mode byte is `[x:u16][y:u16][options:u8][strlen:u8][UTF-8 bytes]`; options match
the cached draw commands, and inline bytes 1–31 adjust x by -10 through 20 just
as they do in cached-font mode 14.

The firmware also adds a microphone control plane (capability tokens `micctl`,
`micmc`, `micraw`). Each temple carries a front + rear microphone pair, and the
stock firmware only ever sends the phone a mono 16 kHz LC3 stream. The mic
extension lets the phone choose, per temple, the capture front end (codec vs
PDM), which of the pair's microphones are used, the codec (LC3 vs raw PCM
passthrough), sample format/rate, and LC3 bitrate, over a private
settings-channel message (sid 0x09 field 103), with live read-back on field 104
so a UI (e.g. SybilSight's glasses -> microphones menu) can display and confirm
the active configuration of both temples. When armed, capture is streamed as
`'SM'` frames carrying the multi-channel samples, a millisecond timestamp, and
the on-device SSR + TDOA angle estimate — everything a phone-side beamformer
needs to do direction-of-arrival processing across the four microphones,
fused with the compass/IMU heading the firmware already forwards. Streaming is
held by a fail-open 90-second renewal lease, so mics can never be left running
when the phone goes away. Bringing up the capture hardware is additionally
gated behind an explicit arm flag because several of the recovered stock audio
entry points are ABI-inferred and must be validated on hardware first; see the
contract comment in `patches/mic_control.c`.

Some other features this has (used by Faceclaw, but the exact API may not be
fully documented):

 * Receive ring and temple-touchpad long-press and long-press-release as regular
   source-qualified gestures, rather than opening a modal offering to quit
 * Play sound effects with the piezo buzzer
 * Receive on-head detection wear/unwear events, to trigger a lock-screen
 * Use the magnetometer as a compass
 * Take over the wakeword ("hey Even") and replace what it opens with a
   different phone-side transcription and AI agent pipeline
 * Take over the screen-wake even on the dashboard, so that you can end the
   EvenHub session (putting the glasses in a low-power mode) and return to
   Faceclaw with a double-tap

Glasses with a custom firmware identify themselves with the version number of
the stock firmware that the modded version is based on, with an extra field in
the settings-response message describing the capabilities added. See
`settings_send_wrapper` in `patches/settings_ext.c`. The format of this
capability string is not yet standardized and is in flux; if writing your own
firmware and your own phone software to go with it, assume that firmware is
probably only compatible if you recognize the exast string.

Custom firmwares in this repository are intended to be backwards-compatible
with the official Even Realities app; ie, if the phone doesn't send any
bluetooth messages that make use of the added features, it will behave the same
way. In practice, this isn't tested, and using a custom firmware with the
official Even app may introduce bugs. EvenHub apps are especially likely to be
broken by custom firmware, since most of the modifications are to
EvenHub-related functionality.

Custom firmwares are currently _not_ expected to be compatible with each other.
When you update Faceclaw, you should update the firmware at the same time. If
you are writing your own software that relies on custom firmware, you probably
want to specify a particular CFW version. (This will probably change in the
future, when there is less new stuff still to be added.)

What _is_ tested for compatibility is the OTA updater. This firmware does not
make any changes to how OTA updates are installed. If you connect the official
Even app to your glasses, and it has a firmware that's newer than the one your
CFW is based on, it will offer to install an update. Installing an OTA update
using the official Even app will fully remove the custom firmware and restore
it to stock behavior. You can also go back to stock firmware by installing an
unmodified firmware image using g2flash, or the "Uninstall firmware" menu item
in Faceclaw.


## Using the Flash Tool

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
  - `patch_compress.py` — the all-in-one patcher (576 carrier lift + image
    compression + direct framebuffer presentation + capability field);
    `gen_patches.py` calls it to build the ops.
    Holds every stock-firmware address the patches depend on; see
    `notes/fw-2.2.6.10-cfw-rebase.md` for how they were derived.
  - `build.py`, `*.c` — the C→position-independent-Thumb pipeline and sources
    for the injected firmware code (compiled by `gen_patches.py`; the resulting
    machine code lands in `cfw_patches.json`).
- `requirements.txt` — the flasher's Python dependencies.

Firmware images (`g2_2.2.6.10*.bin`) are **not** checked in — they are Even's
firmware, so you build them locally with `build_cfw.sh`.

## Requirements

- Python 3.x
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


## Contributing

PRs welcome, especially of firmware mods that unlock functionality that isn't
usable in the stock firmware. Please only submit changes that you've tested on
a real device.


# Acknowledgements

Thanks to kalanihelekunihi for [evenRealities-openCFW](https://github.com/kalanihelekunihi/evenRealities-openCFW/) and Commute773 for [g2-kit-unofficial](https://github.com/Commute773/g2-kit-unofficial/), which were immensely helpful while creating this.
