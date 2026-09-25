export const KeyboardOp = { query: 0, enable: 1, disable: 2, scan: 3, connect: 4,
  disconnect: 5, pair: 6, auth: 7, compare: 8, find: 9, read: 10, write: 11, watch: 12 } as const;
export interface KeyboardRecord {
  kind: number; sequence: number; epoch: number; request: number; code: number;
  handle: number; timestamp: number; data: Uint8Array;
}
export const u16 = (p: Uint8Array, off = 0) => p[off]! | (p[off + 1]! << 8);
export const le16 = (v: number): number[] => [v & 255, (v >>> 8) & 255];
const varint = (v: number): number[] => v < 128 ? [v] : [(v & 127) | 128, ...varint(v >>> 7)];
export function keyboardControl(op: number, request: number, epoch: number, body: number[] = [], magic = 1): Uint8Array {
  if (op < 0 || op > 12 || body.length > 72) throw new Error("Keyboard control bounds");
  const data = [75, 66, 1, op, ...le16(request), ...le16(epoch), ...body];
  return Uint8Array.from([8, 2, 16, ...varint(magic), 0x22, 2, 8, 1, 0x92, 8, data.length, ...data]);
}
/** Bounded, ordered reassembly. A gap callback releases every held key and
 * cancels setup transactions. Heartbeats make a lost final release visible. */
export class KeyboardRecords {
  private epoch?: number;
  private last?: number;
  private fragment?: { record: KeyboardRecord; total: number; used: number };
  constructor(private onGap: () => void) {}
  reset(): void { this.epoch = this.last = undefined; this.fragment = undefined; this.onGap(); }
  push(p: Uint8Array): KeyboardRecord | undefined {
    const invalid = () => { this.fragment = undefined; this.onGap(); return undefined; };
    if (p.length < 24 || p.length > 120 || p[0] !== 75 || p[1] !== 66 || p[2] !== 1) return invalid();
    const view = new DataView(p.buffer, p.byteOffset, p.byteLength);
    const r: KeyboardRecord = { kind: p[3]!, sequence: view.getUint32(4, true), epoch: u16(p, 8),
      request: u16(p, 10), code: u16(p, 12), handle: u16(p, 14), timestamp: view.getUint32(20, true), data: new Uint8Array() };
    const total = u16(p, 16), offset = u16(p, 18), n = p.length - 24;
    if (r.kind < 1 || r.kind > 5 || total > 512 || offset + n > total || (total && !n)) return invalid();
    if (this.epoch !== r.epoch) {
      this.onGap(); this.fragment = undefined; this.last = undefined; this.epoch = r.epoch;
    }
    if (offset === 0) {
      if (this.last !== undefined) {
        const distance = (r.sequence - this.last) >>> 0;
        if (!distance || distance >= 0x80000000) return undefined; // duplicate/old delivery
      }
      if (this.fragment || (this.last !== undefined && r.sequence !== ((this.last + 1) >>> 0))) this.onGap();
      this.last = r.sequence;
      this.fragment = { record: { ...r, data: new Uint8Array(total) }, total, used: 0 };
    }
    const f = this.fragment;
    if (!f || f.used !== offset || f.total !== total || f.record.sequence !== r.sequence ||
        f.record.kind !== r.kind || f.record.request !== r.request || f.record.code !== r.code ||
        f.record.handle !== r.handle || f.record.timestamp !== r.timestamp) return invalid();
    f.record.data.set(p.subarray(24), offset); f.used += n;
    if (f.used !== total) return undefined;
    this.fragment = undefined;
    return f.record;
  }
}
