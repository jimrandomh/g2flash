// Minimal LZ4 *block* compressor (no dependency, no frame header).
//
// Stock G2 firmware 2.2.6.10 added CompressMode=2, which feeds the payload straight to
// LZ4_decompress_safe(src, dst, srcLen, dstCapacity) at 0x0054f338 — i.e. it expects a
// raw LZ4 BLOCK, not an LZ4 frame. Most npm LZ4 packages emit frames (magic 0x184D2204),
// which that decoder rejects, so we produce the block format ourselves.
//
// Block format (one or more sequences, last one literals-only):
//   token   : 1 byte, high nibble = literal length, low nibble = match length - 4
//             a nibble of 15 means "read more": add 255-valued bytes until one is < 255
//   literals: literalLength raw bytes
//   offset  : 2 bytes little-endian, distance back to the match (1..65535)
//   matchlen: continuation bytes, if the low nibble was 15
//
// Two rules the reference decoder relies on, both enforced below:
//   * the last 5 bytes of a block are always literals (LASTLITERALS)
//   * no match may start within the last 12 bytes (MFLIMIT)
// Violating either makes LZ4_decompress_safe fail, which the firmware logs as
// "evenhub_ui: decompress failed, mode=%u raw_len=%u".

const MINMATCH = 4;
const MFLIMIT = 12;
const LASTLITERALS = 5;
const HASH_LOG = 16;
const MAX_DISTANCE = 65535;

/** Compress `src` to an LZ4 block. Output is always a valid block, worst case ~ src + src/255. */
export function lz4CompressBlock(src: Uint8Array): Uint8Array {
  const n = src.length;
  const out = new Uint8Array(lz4CompressBound(n));
  let op = 0;

  const emitLen = (len: number) => {
    while (len >= 255) { out[op++] = 255; len -= 255; }
    out[op++] = len;
  };
  const emitSeq = (anchor: number, litLen: number, offset: number, matchLen: number) => {
    // token
    out[op++] = (litLen >= 15 ? 15 << 4 : litLen << 4) | (matchLen >= 15 ? 15 : matchLen);
    if (litLen >= 15) emitLen(litLen - 15);
    out.set(src.subarray(anchor, anchor + litLen), op); op += litLen;
    if (offset > 0) {
      out[op++] = offset & 0xff;
      out[op++] = (offset >>> 8) & 0xff;
      if (matchLen >= 15) emitLen(matchLen - 15);
    }
  };

  // Too small to hold even one match: everything is literals.
  if (n < MFLIMIT + 1) {
    emitSeq(0, n, 0, 0);
    return out.subarray(0, op);
  }

  const hashTable = new Int32Array(1 << HASH_LOG).fill(-1);
  const read32 = (i: number) =>
    (src[i]! | (src[i + 1]! << 8) | (src[i + 2]! << 16) | (src[i + 3]! << 24)) >>> 0;
  const hash = (v: number) => (Math.imul(v, 2654435761) >>> (32 - HASH_LOG));

  const mflimit = n - MFLIMIT;        // no match may START at/after this
  const matchLimit = n - LASTLITERALS; // no match may EXTEND at/beyond this
  let anchor = 0;
  let ip = 0;

  hashTable[hash(read32(ip))] = ip;
  ip++;

  while (ip < mflimit) {
    const h = hash(read32(ip));
    const ref = hashTable[h]!;
    hashTable[h] = ip;

    if (ref < 0 || ip - ref > MAX_DISTANCE || read32(ref) !== read32(ip)) { ip++; continue; }

    // Extend the match backwards over bytes we would otherwise emit as literals.
    let s = ip, r = ref;
    while (s > anchor && r > 0 && src[s - 1] === src[r - 1]) { s--; r--; }

    // Extend forwards, never into the final LASTLITERALS bytes.
    let m = ip + MINMATCH, q = ref + MINMATCH;
    while (m < matchLimit && src[m] === src[q]) { m++; q++; }

    // The match now spans [s, m), NOT [ip, m) — the length must be measured from the
    // back-extended start or the decoder silently reproduces too few bytes.
    emitSeq(anchor, s - anchor, s - r, m - s - MINMATCH);
    ip = m;
    anchor = ip;
  }

  emitSeq(anchor, n - anchor, 0, 0);   // final sequence: literals only
  return out.subarray(0, op);
}

/** Worst-case compressed size for `n` input bytes (matches LZ4_compressBound). */
export function lz4CompressBound(n: number): number {
  return n + Math.ceil(n / 255) + 16;
}

/**
 * Reference LZ4 block decompressor — used to self-check the compressor before we put
 * bytes on the wire (a malformed block would just render garbage on the lens, since the
 * firmware treats an unknown/broken payload as raw rather than failing loudly).
 */
export function lz4DecompressBlock(src: Uint8Array, maxOut: number): Uint8Array {
  const out = new Uint8Array(maxOut);
  let ip = 0, op = 0;
  while (ip < src.length) {
    const token = src[ip++]!;
    let litLen = token >>> 4;
    if (litLen === 15) {
      let b = 255;
      while (b === 255 && ip < src.length) { b = src[ip++]!; litLen += b; }
    }
    out.set(src.subarray(ip, ip + litLen), op);
    ip += litLen; op += litLen;
    if (ip >= src.length) break;            // final literals-only sequence
    const offset = src[ip]! | (src[ip + 1]! << 8);
    ip += 2;
    let matchLen = token & 0x0f;
    if (matchLen === 15) {
      let b = 255;
      while (b === 255 && ip < src.length) { b = src[ip++]!; matchLen += b; }
    }
    matchLen += MINMATCH;
    if (offset === 0 || offset > op) throw new Error(`lz4: bad offset ${offset} at op=${op}`);
    let ref = op - offset;
    for (let i = 0; i < matchLen; i++) out[op++] = out[ref++]!;  // may overlap; byte-wise is correct
  }
  return out.subarray(0, op);
}
