#!/usr/bin/env bun
// Stream a video (GIF) to the lens as fast as it acks, and benchmark byte count
// + framerate. Decodes/rescales/grayscales/compresses ALL frames up front, then
// sends them in order, pacing only on the per-fragment acks.
//
// Each frame is zlib-compressed and sent with a leading mode byte that selects
// a CFW display path. The 4bpp modes that go through the firmware's shadow (3 and
// 6) run their pixels through RLE before deflate — their payload is zlib(rle(px)),
// not zlib(px) — since runs of one gray level are what this content is made of.
// G2_MODE picks the encoding (handy for comparing 4bpp-BMP vs 8bpp
// compressibility and throughput):
//   full  (default) 8bpp: every frame -> mode 2 (full 8bpp frame, raw pixels)
//   delta           4bpp: first frame mode 2, rest mode 3 (bounding-box update)
//   bmp             4bpp: every frame -> mode 1 (full 4bpp indexed BMP)
//   raw4            4bpp: every frame -> mode 6 (headerless 4bpp, fast expander)
//   lz4             4bpp: STOCK 2.2.6.10 path — 4bpp BMP, LZ4'd, CompressMode=2
// 8bpp carries a full byte per pixel (the panel requantizes to ~16 levels);
// 4bpp is half the raw size before compression. `bmp` (mode 1) runs through the
// stock BMP loader, which decodes with two function calls per pixel; `raw4`
// (mode 6) sends headerless 4bpp and expands it on-device with a plain nibble
// copy, so it gets the 4bpp airtime without the stock decoder's CPU cost.
// `delta` sends only the bounding box of pixels that changed vs the previous
// frame (as 4bpp), so for mostly-static content like Bad Apple it moves very few
// pixels per frame — the cheapest mode both on wire and on-device CPU, at the
// cost of one full keyframe up front (and every G2_KEYFRAME_INTERVAL frames).
//
// `lz4` is the odd one out: it is the only mode that needs NO custom firmware. Stock
// 2.2.6.10 added Even's own image compression (CompressMode 1=RLE, 2=LZ4), so this mode
// sends a plain 4bpp BMP compressed as an LZ4 *block* with CompressMode=2 and no leading
// mode byte, which the stock firmware inflates in evenhub_ui_reflash_event_handler and
// feeds to its normal BMP path. It exists to benchmark Even's official compression
// against ours on the same content. Notes: the firmware decompresses into a malloc(W*H)
// buffer sized from the container, so the payload must inflate to <= W*H bytes; and a
// CompressMode the firmware doesn't know is silently "treated as raw" (i.e. garbage on
// the lens) rather than rejected, so this mode is only meaningful against a 2.2.6.10+
// base. See notes/fw-2.2.6.10-lz4-images.md.
//
//   bun video-bench.ts path/to/bad_apple.gif
//
// Make a GIF from any video with ffmpeg, e.g.:
//   ffmpeg -i bad_apple.mp4 -vf "fps=30,scale=288:144:flags=area,format=gray" bad_apple.gif
//
// Env:
//   G2_IMG_W / G2_IMG_H   target size (default 288x144)
//   G2_IMG_THRESHOLD      >=0 = 1-bit threshold; -1 = grayscale (default -1)
//   G2_MAX_FRAMES         cap frame count (default 0 = all)
//   G2_FRAME_STRIDE       use every Nth decoded frame (default 1)
//   G2_KEYFRAME_INTERVAL  (delta mode) force a full frame every N (default 0 = only the first)
//   G2_MODE               "full" (default), "delta", "bmp", "raw4", or "lz4" (see above)
//   G2_DRY_RUN=1          decode+compress+report only, don't connect/stream
//   G2_MEASURE_PERSIST=1  also report frame sizes if zlib state persisted across
//                         frames (measurement only; firmware doesn't do this yet)
//   G2_WINDOW             pipelined image messages in flight at once (default 2; 1 = serial)
//
// About G2_MEASURE_PERSIST: today every frame is an independent zlib stream, so a
// frame can only back-reference itself. If the firmware instead kept one inflate
// stream alive across frames, later frames could back-reference the sliding-window
// history of earlier ones (and skip the per-frame zlib header/adler framing). This
// flag feeds every frame's raw bytes through a single deflate stream, flushing
// (Z_SYNC_FLUSH) at each frame boundary, and reports what the per-frame message
// sizes would then be — so we can judge whether the firmware complexity is worth it
// before implementing it. It doesn't change what gets streamed to the device.

import {
  G2Session,
  buildCreateStartUpPageContainer,
  buildImageContainers,
  buildImageRawData,
  buildEvenHubBmp,
  planImageFragments,
  type ImageContainerSpec,
} from "g2-kit/ble";
import { startHeartbeat } from "g2-kit/ui";
import { deflateSync, createDeflate, constants as zconst } from "node:zlib";
import { GifReader } from "omggif";
import { lz4CompressBlock } from "./lz4";

const ACK_MS = 12_000;
const W = Math.max(16, Math.min(576, Number(process.env.G2_IMG_W ?? "288")));
const H = Math.max(16, Math.min(288, Number(process.env.G2_IMG_H ?? "144")));
const THRESHOLD = Math.max(-1, Math.min(255, Number(process.env.G2_IMG_THRESHOLD ?? "-1")));
const MAX_FRAMES = Math.max(0, Number(process.env.G2_MAX_FRAMES ?? "0"));
const STRIDE = Math.max(1, Number(process.env.G2_FRAME_STRIDE ?? "1"));
const KEYFRAME_INTERVAL = Math.max(0, Number(process.env.G2_KEYFRAME_INTERVAL ?? "0"));
const MODE = (process.env.G2_MODE ?? "full").toLowerCase(); // "full"|"delta"|"bmp"|"raw4"|"lz4"
if (!["full", "delta", "bmp", "raw4", "lz4"].includes(MODE)) {
  console.error(`G2_MODE must be one of full|delta|bmp|raw4|lz4 (got "${MODE}")`);
  process.exit(1);
}
const BMP4 = MODE === "bmp";    // mode 1: full 4bpp BMP per frame (stock BMP path)
const RAW4 = MODE === "raw4";   // mode 6: headerless 4bpp, our fast expander
const DELTA = MODE === "delta"; // mode 2 keyframes + mode 3 XOR deltas
const LZ4 = MODE === "lz4";     // stock 2.2.6.10: 4bpp BMP as an LZ4 block, CompressMode=2
// Stock firmware sizes its decode buffer as W*H from the container geometry and hands
// LZ4_decompress_safe that as dstCapacity, so a payload inflating past W*H is refused
// on-device ("decompress failed"). Catch it here instead of on the lens.
const LZ4_DECODE_CAP = W * H;
/** CompressMode (ImgRawDataUpdate field 5) we put on the wire. 0 = uncompressed. */
const COMPRESS_MODE = LZ4 ? 2 : 0;
const DRY_RUN = process.env.G2_DRY_RUN === "1";
const MEASURE_PERSIST = process.env.G2_MEASURE_PERSIST === "1";
const WINDOW = Math.max(1, Number(process.env.G2_WINDOW ?? "2"));
const FRAME_SLEEP = Math.max(0, Number(process.env.G2_FRAME_SLEEP ?? "0"));
const IMAGE_SEND_ARM = "R";

let magic = 100;
const nextMagic = () => (magic = magic >= 255 ? 100 : magic + 1);

const videoPath = process.argv[2] ?? process.env.G2_VIDEO;
if (!videoPath) {
  console.error("usage: bun video-bench.ts <video.gif>  (see header for ffmpeg one-liner)");
  process.exit(1);
}

// ---- decode GIF -> grayscale frames at WxH (nearest-neighbor rescale) ----
const gifBuf = new Uint8Array(await Bun.file(videoPath).arrayBuffer());
const gif = new GifReader(gifBuf);
const W0 = gif.width, H0 = gif.height;
const total = gif.numFrames();
console.log(`[decode] ${videoPath}: ${W0}x${H0}, ${total} frames -> target ${W}x${H}`);

function rescaleGray(rgba: Uint8Array): Uint8Array {
  const out = new Uint8Array(W * H);
  if (THRESHOLD >= 0) {
    for (let y = 0; y < H; y++) {
      const sy = Math.min(H0 - 1, ((y * H0) / H) | 0);
      for (let x = 0; x < W; x++) {
        const sx = Math.min(W0 - 1, ((x * W0) / W) | 0);
        const i = (sy * W0 + sx) * 4;
        out[y * W + x] = (rgba[i]! > THRESHOLD ? 255 : 0);
      }
    }
    return out;
  } else {
    for (let y = 0; y < H; y++) {
      const sy = Math.min(H0 - 1, ((y * H0) / H) | 0);
      for (let x = 0; x < W; x++) {
        const sx = Math.min(W0 - 1, ((x * W0) / W) | 0);
        const i = (sy * W0 + sx) * 4;
        out[y * W + x] = (rgba[i]! * 0.299 + rgba[i + 1]! * 0.587 + rgba[i + 2]! * 0.114) | 0;
      }
    }
    return out;
  }
}

// ---- build per-frame payloads up front ([mode][zlib], modes 3/6 [mode][zlib(rle)]) ----
type FrameStat = { bytes: number; key: boolean; persist: number };
const payloads: Uint8Array[] = [];
const stats: FrameStat[] = [];
const rgba = new Uint8Array(W0 * H0 * 4);
let dispose: { x: number; y: number; w: number; h: number } | null = null;
let prev: Uint8Array | null = null;
let used = 0;
let boxAreaSum = 0; // sum of mode-3 box areas (px), for avg-coverage reporting

// Optional persistent-zlib measurement: one long-lived deflate stream fed every
// frame's raw (pre-compression) bytes in order, flushed at each frame boundary so
// we can read off the per-frame output size. Later frames get to back-reference
// the sliding-window history of earlier ones, which independent per-frame deflate
// throws away. Returns the compressed bytes emitted for the just-pushed frame.
let persistPush: ((raw: Uint8Array) => Promise<number>) | null = null;
if (MEASURE_PERSIST) {
  const def = createDeflate();
  let chunks: Buffer[] = [];
  def.on("data", (c: Buffer) => chunks.push(c));
  persistPush = (raw) =>
    new Promise<number>((resolve, reject) => {
      chunks = [];
      def.write(raw, (e) => { if (e) reject(e); });
      // Z_SYNC_FLUSH keeps the window/dictionary (cross-frame refs survive);
      // Z_FULL_FLUSH would reset it and defeat the whole point of measuring.
      def.flush(zconst.Z_SYNC_FLUSH, () => resolve(chunks.reduce((a, c) => a + c.length, 0)));
    });
}

const t0 = performance.now();
for (let i = 0; i < total; i++) {
  if (dispose) {
    for (let yy = dispose.y; yy < dispose.y + dispose.h; yy++)
      rgba.fill(0, (yy * W0 + dispose.x) * 4, (yy * W0 + dispose.x + dispose.w) * 4);
  }
  const fi = gif.frameInfo(i);
  gif.decodeAndBlitFrameRGBA(i, rgba);
  dispose = fi.disposal === 2 ? { x: fi.x, y: fi.y, w: fi.width, h: fi.height } : null;

  if (i % STRIDE !== 0) continue;
  const gray = rescaleGray(rgba);
  // In delta mode a frame is a keyframe on the first frame or every Nth; full
  // and bmp modes send a whole frame every time.
  //const isKey = !DELTA || prev === null || (KEYFRAME_INTERVAL > 0 && used % KEYFRAME_INTERVAL === 0);
  const isKey = (i==0);
  // Build this frame's payload. Simple modes are [mode][zlib(raw)]; the mode-3
  // box carries a 4-byte uncompressed header before its zlib box pixels. For the
  // persist measurement we also track the pre-compression bytes and the length of
  // the uncompressed prefix (mode byte + any header).
  let payload: Uint8Array, persistRaw: Uint8Array, prefixLen: number;
  if (LZ4) {
    // Stock 2.2.6.10: no mode byte and no zlib — the whole payload IS the LZ4 block,
    // and CompressMode=2 on the message is what tells the firmware to inflate it.
    persistRaw = buildEvenHubBmp(W, H, (x, y) => gray[y * W + x]! >> 4);
    if (persistRaw.length > LZ4_DECODE_CAP) {
      console.error(
        `G2_MODE=lz4: a ${W}x${H} BMP is ${persistRaw.length} B but stock firmware only ` +
        `allocates W*H = ${LZ4_DECODE_CAP} B to decompress into, so the lens would reject ` +
        `it. Use a smaller frame.`);
      process.exit(1);
    }
    payload = lz4CompressBlock(persistRaw); prefixLen = 0;
  } else if (BMP4) {
    // mode 1: full 4bpp indexed BMP (gray 0..255 -> 0..15), zlib-compressed.
    persistRaw = buildEvenHubBmp(W, H, (x, y) => gray[y * W + x]! >> 4);
    payload = pack(1, persistRaw); prefixLen = 1;
  } else if (RAW4) {
    // mode 6: headerless tight 4bpp (gray>>4), RLE'd then deflated, expanded by our
    // fast on-device expander.
    persistRaw = rleEncode(pack4bpp(gray));
    payload = pack(6, persistRaw); prefixLen = 1;
  } else if (isKey && DELTA) {
    // delta keyframe: mode 6 full 4bpp — seeds the firmware's persistent 4bpp
    // shadow (which the box deltas composite onto), unlike an 8bpp mode-2 frame.
    persistRaw = rleEncode(pack4bpp(gray));
    payload = pack(6, persistRaw); prefixLen = 1;
  } else if (isKey) {
    // G2_MODE=full: full 8bpp frame every frame.
    persistRaw = gray;
    payload = pack(2, gray); prefixLen = 1;
  } else {
    // mode 3: bounding-box delta — send only the changed rectangle as 4bpp, RLE'd
    // then deflated.
    const box = computeBox(gray, prev!);
    boxAreaSum += box.w * box.h;
    persistRaw = rleEncode(box.pixels);
    payload = packBox(box, persistRaw, used & 0xffff); prefixLen = 7; // used = this frame's index

  }
  payloads.push(payload);
  const persist = persistPush ? prefixLen + (await persistPush(persistRaw)) : 0;
  stats.push({ bytes: payload.length, key: isKey, persist });
  prev = gray;
  used++;
  if (MAX_FRAMES && used >= MAX_FRAMES) break;
  if (used % 200 === 0) console.log(`[decode] ${used} frames prepared...`);
}
const prepMs = performance.now() - t0;

// Defensive check: the mode-3 (delta) payloads must carry strictly increasing,
// unique frame ids in the exact order they'll be sent (payloads are streamed in
// array order, writes awaited, so build order == send order). If this passes, the
// sender is NOT reordering/duplicating frames — so any reorder/dup/skip the
// firmware's diagnostic flags must originate in the transport or the firmware.
{
  let prev = -1, bad = 0, n3 = 0;
  for (const p of payloads) {
    if (p[0] !== 3) continue; // only deltas carry a fid
    n3++;
    const fid = p[5]! | (p[6]! << 8);
    if (fid <= prev) { if (bad++ < 5) console.error(`[check] delta fid ${fid} not > prev ${prev} (out-of-order/dup in build)`); }
    prev = fid;
  }
  console.log(bad
    ? `[check] FAIL: ${bad}/${n3} delta fids out of order or duplicated in the build`
    : `[check] ok: ${n3} delta fids strictly increasing & unique (send side is clean)`);
}

const keyBytes = stats.filter((s) => s.key).reduce((a, s) => a + s.bytes, 0);
const keyN = stats.filter((s) => s.key).length;
const deltaBytes = stats.filter((s) => !s.key).reduce((a, s) => a + s.bytes, 0);
const deltaN = stats.length - keyN;
const totalBytes = keyBytes + deltaBytes;
const boxCov = DELTA && deltaN ? ` | avg box ${(100 * boxAreaSum / (deltaN * W * H)).toFixed(1)}% of frame` : "";
console.log(
  `[prepared] mode=${MODE} ${W}x${H} ${payloads.length} frames in ${(prepMs / 1000).toFixed(1)}s | ` +
    `${(totalBytes / 1024).toFixed(0)} KiB total, avg ${(totalBytes / payloads.length).toFixed(0)} B/frame ` +
    `(keyframes ${keyN}@${keyN ? (keyBytes / keyN).toFixed(0) : 0}B, deltas ${deltaN}@${deltaN ? (deltaBytes / deltaN).toFixed(0) : 0}B)${boxCov}`,
);

if (MEASURE_PERSIST) {
  const pKeyBytes = stats.filter((s) => s.key).reduce((a, s) => a + s.persist, 0);
  const pDeltaBytes = stats.filter((s) => !s.key).reduce((a, s) => a + s.persist, 0);
  const pTotal = pKeyBytes + pDeltaBytes;
  const deltaPct = totalBytes ? 100 * (1 - pTotal / totalBytes) : 0;
  console.log(
    `[persist]  if zlib state persisted across frames: ${(pTotal / 1024).toFixed(0)} KiB total, ` +
      `avg ${(pTotal / payloads.length).toFixed(0)} B/frame ` +
      `(keyframes ${keyN}@${keyN ? (pKeyBytes / keyN).toFixed(0) : 0}B, deltas ${deltaN}@${deltaN ? (pDeltaBytes / deltaN).toFixed(0) : 0}B) ` +
      `-> ${deltaPct >= 0 ? "saves" : "costs"} ${Math.abs(deltaPct).toFixed(1)}% vs per-frame deflate`,
  );
}

function pack(mode: number, raw: Uint8Array): Uint8Array {
  const z = deflateSync(raw);
  const out = new Uint8Array(z.length + 1);
  out[0] = mode;
  out.set(z, 1);
  return out;
}

// Run-length encode packed 4bpp pixels for CFW modes 3 and 6, whose payload is
// zlib(rle(pixels)) rather than zlib(pixels). Runs are over the pixel NIBBLES of
// `pix` in wire order (high nibble = left pixel), including the pad nibble that ends
// each row at odd widths — i.e. `pix` read as pix.length*2 nibbles. One token is:
//   [cnt4|color4]                 cnt 1..15
//   [0|color4][cnt8]              cnt 1..255
//   [0|color4][0][cntLo][cntHi]   cnt 1..65535, little-endian
// with the low nibble always the color; runs longer than 65535 split across tokens.
// Worst case (every nibble its own run) is exactly one byte per nibble, which is what
// we size the buffer for.
function rleEncode(pix: Uint8Array): Uint8Array {
  const n = pix.length * 2;
  const out = new Uint8Array(n);
  let o = 0;
  const nib = (i: number) => (i & 1 ? pix[i >> 1]! & 0x0f : pix[i >> 1]! >> 4);
  let i = 0;
  while (i < n) {
    const v = nib(i);
    let j = i + 1;
    while (j < n && nib(j) === v) j++;
    let run = j - i;
    while (run > 0) {
      const c = Math.min(run, 0xffff);
      if (c <= 15) {
        out[o++] = (c << 4) | v;
      } else if (c <= 255) {
        out[o++] = v;           // high nibble 0 = escape to an 8-bit count
        out[o++] = c;
      } else {
        out[o++] = v;
        out[o++] = 0;           // 8-bit count 0 = escape to a 16-bit count
        out[o++] = c & 0xff;
        out[o++] = c >> 8;
      }
      run -= c;
    }
    i = j;
  }
  return out.subarray(0, o);
}

// Tightly-packed 4bpp (mode 6): gray 0..255 -> nibble (gray>>4), 2 px/byte with
// the left pixel in the high nibble, rows top-down, stride = ceil(W/2), no
// padding. Matches the firmware's headerless-4bpp expander.
function pack4bpp(gray: Uint8Array): Uint8Array {
  const stride = (W + 1) >> 1;
  const out = new Uint8Array(stride * H);
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x += 2) {
      const hi = gray[y * W + x]! >> 4;
      const lo = x + 1 < W ? gray[y * W + x + 1]! >> 4 : 0;
      out[y * stride + (x >> 1)] = (hi << 4) | lo;
    }
  }
  return out;
}

// mode 3: bounding box of the pixels whose displayed (4bpp) value changed vs the
// previous frame, quantized so left/width are multiples of 4 and top/height of 2
// (the units the wire header stores, and what the firmware requires). The box's
// pixels are extracted as tight 4bpp (gray>>4). If nothing changed, we emit a
// minimal 4x2 box at the origin (a no-op update) to keep every frame a message.
type Box = { left: number; top: number; w: number; h: number; pixels: Uint8Array };
function computeBox(cur: Uint8Array, prev: Uint8Array): Box {
  let minx = W, miny = H, maxx = -1, maxy = -1;
  for (let y = 0; y < H; y++) {
    for (let x = 0; x < W; x++) {
      const i = y * W + x;
      if (cur[i]! >> 4 !== prev[i]! >> 4) {
        if (x < minx) minx = x;
        if (x > maxx) maxx = x;
        if (y < miny) miny = y;
        if (y > maxy) maxy = y;
      }
    }
  }
  if (maxx < 0) { minx = 0; maxx = 3; miny = 0; maxy = 1; } // no change -> tiny box
  const left = minx & ~3;
  const right = Math.min(W, (maxx + 4) & ~3); // round up to a multiple of 4
  const top = miny & ~1;
  const bottom = Math.min(H, (maxy + 2) & ~1); // round up to a multiple of 2
  const w = right - left, h = bottom - top;
  const stride = w >> 1;
  const pixels = new Uint8Array(stride * h);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x += 2) {
      const hi = cur[(top + y) * W + left + x]! >> 4;
      const lo = cur[(top + y) * W + left + x + 1]! >> 4; // w is even, so x+1 < w
      pixels[y * stride + (x >> 1)] = (hi << 4) | lo;
    }
  }
  return { left, top, w, h, pixels };
}

// [3][left/4][top/2][width/4][height/2][fid_lo][fid_hi][zlib(rle(tight 4bpp box pixels))]
// fid = uint16 per-frame counter, so the firmware can flag out-of-order/skipped frames.
// `rle` is the already-RLE'd box pixels (the caller keeps it for the persist measurement).
function packBox(box: Box, rle: Uint8Array, fid: number): Uint8Array {
  const z = deflateSync(rle);
  const out = new Uint8Array(7 + z.length);
  out[0] = 3;
  out[1] = box.left >> 2;
  out[2] = box.top >> 1;
  out[3] = box.w >> 2;
  out[4] = box.h >> 1;
  out[5] = fid & 0xff;
  out[6] = (fid >> 8) & 0xff;
  out.set(z, 7);
  return out;
}

if (DRY_RUN) {
  console.log("[dry-run] not sending. (re-run without G2_DRY_RUN=1 to stream to the device)");
  process.exit(0);
}

const session = await G2Session.open();
const hb = startHeartbeat({ session, nextMagic });
const suffix = String(Date.now() % 10_000).padStart(4, "0");

try {
  const create = buildCreateStartUpPageContainer({
    name: `b${suffix}`, items: ["."], containerId: 1, captureEvents: false, magic: nextMagic(), extraContainerNames: [`c${suffix}`],
  });
  if (!(await session.sendPb(0xe0, create.pb, create.magic, { ackTimeoutMs: ACK_MS }))) throw new Error("CREATE did not ack");
  const container: ImageContainerSpec = {
    name: `c${suffix}`, containerId: 2, x: Math.max(0, (576 - W) >> 1), y: Math.max(0, (288 - H) >> 1), width: W, height: H,
  };
  const rebuild = buildImageContainers({ containers: [container], magic: nextMagic() });
  if (!(await session.sendPb(0xe0, rebuild.pb, rebuild.magic, { ackTimeoutMs: ACK_MS }))) throw new Error("REBUILD did not ack");
  // Layout is created on the default arm; we stream to IMAGE_SEND_ARM. Let the
  // container replicate to that lens before the first frame to avoid a race.
  await new Promise((r) => setTimeout(r, 300));

  console.log(`[stream] sending ${payloads.length} frames, mode=${MODE}, window=${WINDOW}, arm=${IMAGE_SEND_ARM}...`);
  let sid = 1, sent = 0, sentBytes = 0, aborted = false;
  let latSum = 0, latN = 0, latWorst = 0;
  const lats: number[] = [];

  // Sliding window of in-flight acks: each image-raw message fires immediately
  // (only its BLE write is awaited); we await the oldest ack before exceeding
  // WINDOW outstanding, so the firmware sees backpressure but we don't pay a
  // full ack round trip per message.
  const inflight: Array<{ ack: Promise<unknown>; t: number }> = [];
  const awaitOldest = async (): Promise<boolean> => {
    const it = inflight.shift()!;
    const ack = await it.ack;
    const lat = performance.now() - it.t;
    latSum += lat; latN++; if (lat > latWorst) latWorst = lat;
    lats.push(lat);
    return ack !== null;
  };
  const sendMsg = async (pb: Uint8Array, mg: number): Promise<boolean> => {
    while (inflight.length >= WINDOW) {
      if (!(await awaitOldest())) return false;
    }
    if (FRAME_SLEEP > 0) {
      await new Promise((r) => setTimeout(r, FRAME_SLEEP));
    }
    const r = await session.sendPbPipelined(0xe0, pb, mg, { ackTimeoutMs: ACK_MS, arm: IMAGE_SEND_ARM });
    inflight.push({ ack: r.ack, t: performance.now() });
    return true;
  };

  const tStart = performance.now();
  for (let i = 0; i < payloads.length; i++) {
    for (const frag of planImageFragments(payloads[i]!, 4000)) {
      const raw = buildImageRawData({
        containerId: container.containerId, containerName: container.name, mapSessionId: sid,
        mapTotalSize: payloads[i]!.length, mapFragmentIndex: frag.index,
        mapRawData: frag.data, magic: nextMagic(), // packet size is derived from mapRawData.length
        compressMode: COMPRESS_MODE,
      });
      if (!(await sendMsg(raw.pb, raw.magic))) { aborted = true; break; }
    }
    sid++;
    if (aborted) { console.log(`[stream] frame ${i} NO_ACK — aborting`); break; }
    sent++;
    sentBytes += payloads[i]!.length;
    if (sent % 100 === 0) {
      const fps = sent / ((performance.now() - tStart) / 1000);
      console.log(`[stream] ${sent}/${payloads.length}  ${fps.toFixed(1)} fps  ${(sentBytes / 1024).toFixed(0)} KiB`);
    }
  }
  // drain the rest of the window so elapsed covers every ack
  while (!aborted && inflight.length) {
    if (!(await awaitOldest())) aborted = true;
  }

  const elapsed = (performance.now() - tStart) / 1000;
  const fps = sent / elapsed;
  // Percentiles over all observed ack latencies (nearest-rank on the sorted set).
  const sortedLats = [...lats].sort((a, b) => a - b);
  const pct = (p: number) =>
    sortedLats.length ? sortedLats[Math.min(sortedLats.length - 1, Math.ceil(p * sortedLats.length) - 1)]! : 0;
  const p90 = pct(0.9), p99 = pct(0.99);
  // A worst >1000 ms almost certainly means the run ended on a timeout/disconnect,
  // so the "worst" is that terminal stall, not a representative ack — surface the
  // next-worst latency too so the tail is still readable.
  const secondWorst = sortedLats.length >= 2 ? sortedLats[sortedLats.length - 2]! : 0;
  const showSecond = latWorst > 1000 && sortedLats.length >= 2;
  console.log(
    `\n=== RESULT (mode=${MODE}, ${W}x${H}, window=${WINDOW}) ===\n` +
      `frames sent : ${sent}${aborted ? " (aborted early)" : ""}\n` +
      `elapsed     : ${elapsed.toFixed(1)} s\n` +
      `framerate   : ${fps.toFixed(2)} fps  (${(1000 / fps).toFixed(0)} ms/frame)\n` +
      `ack latency : avg ${latN ? (latSum / latN).toFixed(0) : 0} ms, p90 ${p90.toFixed(0)} ms, p99 ${p99.toFixed(0)} ms, worst ${latWorst.toFixed(0)} ms` +
      `${showSecond ? ` (2nd-worst ${secondWorst.toFixed(0)} ms)` : ""}  (round trip pipelining hides)\n` +
      `wire image  : ${(sentBytes / 1024).toFixed(0)} KiB  (avg ${sent ? (sentBytes / sent).toFixed(0) : 0} B/frame, ${((sentBytes * 8) / elapsed / 1000).toFixed(0)} kbit/s)\n` +
      `(image bytes only; protobuf+aa21 envelope add per-fragment overhead)`,
  );
} finally {
  hb.stop();
  await session.close();
}
process.exit(0);
