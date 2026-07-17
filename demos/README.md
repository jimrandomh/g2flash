# g2 CFW demos

Two small [Bun](https://bun.sh) programs that talk to a pair of Even Realities
G2 glasses over Bluetooth and show off the [custom firmware](../) built by
`../build_cfw.sh`:

- **`detect-cfw.ts`** — reads the glasses' settings and reports whether they're
  running the custom firmware, and which extensions it advertises. Works on
  stock firmware too (it just reports "no CFW").
- **`video-bench.ts`** — streams a video (as a GIF) to the lens as fast as it
  acks and benchmarks the achieved framerate / byte count. Streams via the
  CFW's compressed display modes — 8bpp full frames, 8bpp XOR deltas, or 4bpp
  indexed BMP — selectable with `G2_MODE` to compare size and throughput.

They depend on [`g2-kit`](https://github.com/jimrandomh/g2-kit-unofficial) (a
reverse-engineered BLE library for the G2), pulled directly from GitHub — see
`package.json`. Nothing here needs the rest of this repo at runtime; the CFW
just needs to already be flashed for the demos to show anything interesting.

## Setup

```bash
cd demos
bun install
```

> The glasses must be powered on and **not** connected to the phone (quit the
> Even app / turn off the phone's Bluetooth) so they advertise for a direct
> connection. On macOS the first run prompts for Bluetooth permission.

## Detect the firmware

```bash
bun detect-cfw.ts        # or: bun run detect
```

On the custom firmware you'll see something like:

```
firmware: L=2.2.4.34 R=2.2.4.34
CFW detected: EVENCFW/1 img576 imgz xordelta stereo
  contract v1, features: img576, imgz, xordelta, stereo
  img576=yes imgz=yes xordelta=yes stereo=yes
```

On stock firmware it prints `no CFW capability field`.

## Video streaming benchmark

`video-bench.ts` takes a GIF and streams its frames. Make one from any video
with ffmpeg (grayscale, sized to the lens):

```bash
ffmpeg -i input.mp4 -vf "fps=30,scale=288:144:flags=area,format=gray" demo.gif
bun video-bench.ts demo.gif        # or: bun run bench demo.gif
```

It decodes/rescales/compresses every frame up front, then streams them in
order, pacing on the per-fragment acks, and prints framerate + bytes at the end.

Useful environment variables:

| Var | Default | Meaning |
|-----|---------|---------|
| `G2_IMG_W` / `G2_IMG_H` | `288` / `144` | target size (max `576`×`288`) |
| `G2_IMG_THRESHOLD` | `-1` | `>=0` = 1-bit threshold; `-1` = grayscale |
| `G2_MODE` | `full` | `full` = 8bpp full frame (mode 2); `delta` = 4bpp bounding-box update of the changed region (mode 3); `bmp` = 4bpp BMP via stock loader (mode 1); `raw4` = headerless 4bpp via fast expander (mode 6). Modes 3 and 6 run their pixels through RLE before deflate (payload = `zlib(rle(px))`), the others deflate directly. |
| `G2_KEYFRAME_INTERVAL` | `0` | in `delta` mode, force a full frame every N |
| `G2_FRAME_STRIDE` | `1` | use every Nth source frame |
| `G2_MAX_FRAMES` | `0` | cap frame count (`0` = all) |
| `G2_WINDOW` | `2` | image messages in flight at once (`1` = serial) |
| `G2_DRY_RUN` | — | `1` = decode/compress/report only, don't connect |

Sweep `G2_WINDOW` (e.g. `1`, `2`, `4`) to see how much the ack round-trip is
costing — higher windows overlap the next frame's BLE transfer with the current
frame's on-device processing. Typical results on the CFW: ~22 fps at 288×144,
~9 fps at 576×288.

## Requires the custom firmware

`video-bench.ts` uses display modes that only exist in the CFW; against stock
firmware it won't render. Build and flash the firmware first (see the
[top-level README](../README.md)), then confirm with `detect-cfw.ts`.
