import { KeyboardOp as Op, KeyboardRecords, keyboardControl, le16, u16, type KeyboardRecord } from "./keyboard-protocol";
import { HidKeys, parseReportMap, type HidReport, type KeyEvent } from "./keyboard-hid";

export interface KeyboardTransport { send(pb: Uint8Array, magic: number): Promise<void> }
export interface KeyboardStatus { enabled: boolean; conn: number; connected: boolean; encrypted: boolean; scanning: boolean }
export interface KeyboardInput { handle: number; reportId: number; service: number; timestamp: number; data: Uint8Array }
export interface KeyboardCallbacks {
  keys?(events: KeyEvent[]): void;
  raw?(input: KeyboardInput): void;
  record?(record: KeyboardRecord): void;
  gap?(): void;
  error?(error: Error): void;
  servicesChanged?(): void;
  /** display=true: choose a random six-digit value, show it to the user for
   * typing on the keyboard, then return it. false: ask for the displayed PIN. */
  passkey?(display: boolean): Promise<number>;
  compare?(value: number): Promise<boolean>;
}
interface Attribute { handle: number; uuid: number | string }
export interface KeyboardReport { service: number; handle: number; id: number; type: number; ccc?: number; properties: number; map: number[] }
export interface KeyboardConfiguration { version: 1; epoch: number; reports: KeyboardReport[]; serviceChanged?: number }
interface Pending { op: number; resolve: (r: KeyboardRecord) => void; reject: (e: Error) => void; timer: ReturnType<typeof setTimeout> }
const delay = (ms: number) => new Promise<void>(resolve => setTimeout(resolve, ms));
const error = (message: string) => new Error(message);
/** ATT UUIDs are little endian. Normalize the Bluetooth base UUID as well as
 * the compact 16-bit encoding; leave vendor UUIDs as stable hex strings. */
export function attributeUuid(bytes: Uint8Array): number | string {
  if (bytes.length === 2) return u16(bytes);
  if (bytes.length !== 16) throw error("Invalid ATT UUID length");
  const base = [0xfb,0x34,0x9b,0x5f,0x80,0,0,0x80,0,0x10,0,0];
  if (base.every((v,i) => bytes[i] === v) && bytes[14] === 0 && bytes[15] === 0) return u16(bytes,12);
  return Array.from(bytes,x => x.toString(16).padStart(2,"0")).join("");
}
export class KeyboardClient {
  epoch = 0;
  status: KeyboardStatus = { enabled: false, conn: 0, connected: false, encrypted: false, scanning: false };
  private serial = 0;
  private magic = 30;
  private pending = new Map<number, Pending>();
  private keys = new HidKeys();
  private inputs = new Map<number, { report: HidReport; id: number; service: number }>();
  private configuration?: KeyboardConfiguration;
  private records = new KeyboardRecords(() => this.gap());
  constructor(private transport: KeyboardTransport, private callbacks: KeyboardCallbacks = {}) {}
  /** Feed field 131 from RIGHT sid-09 notifications (including while the
   * application's UI is suspended). Feed complete protobuf payloads only. */
  accept(data: Uint8Array): void {
    const r = this.records.push(data);
    if (!r) return;
    this.epoch = r.epoch;
    this.callbacks.record?.(r);
    if (r.kind === 1 && r.data.length === 10) {
      this.status = { enabled: !!r.data[0], conn: r.data[1]!, connected: !!r.data[2], encrypted: !!r.data[3], scanning: !!r.data[4] };
      if (!this.status.enabled || !this.status.connected || !this.status.encrypted) this.release();
    }
    const pending = this.pending.get(r.request);
    if (pending && (r.kind === 1 || r.kind === 2)) {
      this.pending.delete(r.request); clearTimeout(pending.timer);
      if (r.kind === 1 && r.code) pending.reject(error(`Keyboard command ${pending.op}: firmware error ${r.code}`));
      else pending.resolve(r);
    }
    if (r.kind === 4) {
      const event = r.code & 255;
      if (event === 0x28 || event === 0x2b || event === 0x2d) {
        this.gap(); this.inputs.clear(); this.configuration = undefined;
      }
      if (event === 0x2e && r.data.length === 2) {
        if (r.data[0]) {
          void this.command(Op.disconnect).catch(e => this.callbacks.error?.(e));
          this.callbacks.error?.(error("OOB-only keyboard pairing is unsupported"));
        } else void this.authenticate(!!r.data[1]).catch(e => this.callbacks.error?.(e));
      }
      if (event === 0x35 && r.data.length === 4) void this.confirm(new DataView(r.data.buffer, r.data.byteOffset, 4).getUint32(0, true)).catch(e => this.callbacks.error?.(e));
    }
    if (r.kind === 3) {
      if (r.handle === this.configuration?.serviceChanged) {
        this.gap(); this.inputs.clear(); this.configuration = undefined;
        this.callbacks.servicesChanged?.();
        return;
      }
      const input = this.inputs.get(r.handle);
      if (!input || this.configuration?.epoch !== r.epoch) { this.gap(); return; }
      this.callbacks.raw?.({ handle: r.handle, reportId: input.id, service: input.service, timestamp: r.timestamp, data: r.data });
      try { this.callbacks.keys?.(this.keys.update(r.handle, input.report, r.data)); }
      catch (e) { this.release(); this.callbacks.error?.(e as Error); }
    }
  }
  private release(): void { const events = this.keys.reset(); if (events.length) this.callbacks.keys?.(events); }
  private gap(): void {
    this.release(); this.callbacks.gap?.();
    for (const [id, p] of this.pending) if (p.op >= Op.find) {
      clearTimeout(p.timer); p.reject(error("Keyboard stream gap during ATT transaction")); this.pending.delete(id);
    }
  }
  /** Call on phone-link loss. Clear held state immediately. Restore the saved
   * mapping before feeding notifications after CoreBluetooth restoration. */
  lostPhoneLink(): void { this.records.reset(); this.cancel(); }
  close(): void { this.lostPhoneLink(); }
  private cancel(): void {
    for (const p of this.pending.values()) { clearTimeout(p.timer); p.reject(error("Keyboard client closed")); }
    this.pending.clear();
  }
  async command(op: number, body: number[] = []): Promise<KeyboardRecord> {
    if (this.pending.size >= 8) throw error("Too many keyboard commands");
    const request = this.serial = this.serial % 65535 + 1;
    const magic = this.magic = this.magic % 200 + 1;
    return new Promise<KeyboardRecord>((resolve, reject) => {
      const timer = setTimeout(() => { this.pending.delete(request); reject(error(`Keyboard command ${op} timed out`)); }, 32000);
      this.pending.set(request, { op, resolve, reject, timer });
      this.transport.send(keyboardControl(op, request, this.epoch, body, magic), magic).catch(e => {
        clearTimeout(timer); this.pending.delete(request); reject(e);
      });
    });
  }
  async enable(): Promise<void> { await this.command(Op.enable); }
  async disable(): Promise<void> { await this.command(Op.disable); this.release(); }
  async scan(): Promise<void> {
    await this.command(Op.scan);
    await this.until(() => !this.status.scanning, 10000);
  }
  async connect(addressType: number, address: Uint8Array): Promise<void> {
    if (address.length !== 6 || (!Number.isInteger(addressType) || addressType < 0 || addressType > 3)) throw error("Invalid BLE address");
    this.release(); this.inputs.clear(); this.configuration = undefined;
    await this.command(Op.connect, [addressType, ...address]);
    await this.until(() => this.status.connected, 32000);
    if (!this.status.encrypted) {
      // A peripheral security request can have started pairing already. BUSY
      // is not treated as success: still require the encrypted status below.
      try { await this.command(Op.pair); }
      catch (e) { if (!String(e).endsWith("firmware error 3")) throw e; }
      await this.until(() => this.status.encrypted, 65000);
    }
  }
  private async until(test: () => boolean, timeout: number): Promise<void> {
    const end = Date.now() + timeout;
    while (!test()) { if (Date.now() >= end) throw error("Keyboard state timeout"); await delay(50); }
  }
  private async authenticate(display: boolean): Promise<void> {
    if (!this.callbacks.passkey) { await this.command(Op.disconnect); throw error("Pairing needs a passkey handler"); }
    const epoch = this.epoch, pin = await this.callbacks.passkey(display);
    if (epoch !== this.epoch || !Number.isInteger(pin) || pin < 0 || pin > 999999) throw error("Invalid/stale pairing passkey");
    await this.command(Op.auth, [pin & 255, (pin >>> 8) & 255, pin >>> 16]);
  }
  private async confirm(value: number): Promise<void> {
    const epoch = this.epoch, accept = await this.callbacks.compare?.(value) ?? false;
    if (epoch === this.epoch) await this.command(Op.compare, [Number(accept)]);
  }
  private async att(op: number, body: number[]): Promise<KeyboardRecord> {
    const r = await this.command(op, body);
    if (r.kind !== 2) throw error("Expected keyboard ATT response");
    return r;
  }
  async read(handle: number): Promise<Uint8Array> {
    const parts: number[] = [];
    // Read Blob until a short/empty response or Invalid Offset. Do not assume
    // MTU=23; using one extra request works with every negotiated MTU.
    for (;;) {
      const r = await this.att(Op.read, [...le16(handle), ...le16(parts.length)]);
      const status = r.code >>> 8;
      if (parts.length && (status === 7 || status === 11)) break;
      if (status) throw error(`ATT read ${handle}: ${status}`);
      if (!r.data.length) break;
      parts.push(...r.data);
      if (parts.length > 512) throw error("Attribute exceeds 512 bytes");
      if (parts.length === 512) break;
    }
    return Uint8Array.from(parts);
  }
  /** Read a short attribute once (Report Reference, declaration, service UUID). */
  private async shortRead(handle: number): Promise<Uint8Array> {
    const r = await this.att(Op.read, [...le16(handle), 0, 0]);
    if (r.code >>> 8) throw error(`ATT read ${handle}: ${r.code >>> 8}`);
    return r.data;
  }
  async write(handle: number, value: Uint8Array, withoutResponse = false): Promise<void> {
    if (!value.length || value.length > 18) throw error("Keyboard write supports 1..18 bytes");
    const r = await this.att(Op.write, [...le16(handle), Number(withoutResponse), value.length, ...value]);
    if (r.code >>> 8) throw error(`ATT write ${handle}: ${r.code >>> 8}`);
  }
  private async attributes(): Promise<Attribute[]> {
    const attrs: Attribute[] = [];
    let start = 1;
    while (start <= 65535) {
      const r = await this.att(Op.find, [...le16(start), 255, 255]);
      if ((r.code >>> 8) === 10) break; // attribute not found: normal end
      if (r.code >>> 8) throw error(`ATT Find Information: ${r.code >>> 8}`);
      const width = r.data[0] === 1 ? 4 : r.data[0] === 2 ? 18 : 0;
      if (!width || r.data.length <= 1 || (r.data.length - 1) % width) throw error("Malformed Find Information response");
      for (let i = 1; i < r.data.length; i += width) {
        const handle = u16(r.data, i);
        if (handle < start || attrs.length >= 512) throw error("Invalid or excessive GATT handles");
        const uuid = attributeUuid(r.data.subarray(i + 2, i + width));
        attrs.push({ handle, uuid }); start = handle + 1;
      }
    }
    return attrs;
  }
  /** Discover HIDS instances, Report Maps and each Report Reference; select
   * Report Protocol and subscribe input reports. No fixed keyboard handles. */
  async configure(): Promise<KeyboardConfiguration> {
    const attrs = await this.attributes(), reports: KeyboardReport[] = [];
    const descriptorsFor = (handle: number) => {
      const end = attrs.find(a => a.handle > handle && [0x2800,0x2801,0x2803].includes(a.uuid as number))?.handle ?? 65536;
      return attrs.filter(a => a.handle > handle && a.handle < end);
    };
    const addReport = async (service: number, a: Attribute, map: Uint8Array, parsed: Map<number,HidReport>) => {
      const descriptors = descriptorsFor(a.handle);
      const ref = descriptors.find(d => d.uuid === 0x2908);
      const decl = attrs.filter(d => d.handle < a.handle).reverse().find(d => d.uuid === 0x2803);
      if (!ref || !decl) throw error("HID Report has no declaration/reference");
      const reference = await this.shortRead(ref.handle), declaration = await this.shortRead(decl.handle);
      if (reference.length !== 2 || (declaration.length !== 5 && declaration.length !== 19) ||
          u16(declaration,1) !== a.handle || attributeUuid(declaration.subarray(3)) !== a.uuid) throw error("Invalid HID Report metadata");
      const id = reference[0]!, type = reference[1]!, properties = declaration[0]!;
      if (type < 1 || type > 3) throw error("Invalid Report Reference type");
      const ccc = descriptors.find(d => d.uuid === 0x2902)?.handle;
      if (type === 1 && (!parsed.has(id) || !ccc || !(properties & 0x10))) throw error("Input Report lacks map/notify/CCC");
      if (reports.some(r => r.handle === a.handle)) throw error("Ambiguous report shared by multiple HID services");
      reports.push({ service, handle: a.handle, id, type, ccc, properties, map: Array.from(map) });
    };
    for (let i = 0; i < attrs.length; i++) {
      const service = attrs[i]!;
      if (service.uuid !== 0x2800) continue;
      if (attributeUuid(await this.shortRead(service.handle)) !== 0x1812) continue;
      const end = attrs.slice(i + 1).find(a => a.uuid === 0x2800 || a.uuid === 0x2801)?.handle ?? 65536;
      const members = attrs.filter(a => a.handle > service.handle && a.handle < end);
      const mapAttr = members.find(a => a.uuid === 0x2a4b);
      if (!mapAttr) throw error("HID service has no Report Map");
      const map = await this.read(mapAttr.handle), parsed = parseReportMap(map);
      for (const mode of members.filter(a => a.uuid === 0x2a4e)) await this.write(mode.handle, Uint8Array.of(1), true);
      for (const a of members.filter(a => a.uuid === 0x2a4d)) await addReport(service.handle,a,map,parsed);
      // HOGP can refer to input reports in other services (commonly Battery).
      for (const ext of descriptorsFor(mapAttr.handle).filter(a => a.uuid === 0x2907)) {
        const uuid = attributeUuid(await this.shortRead(ext.handle));
        const matches = attrs.filter(a => a.uuid === uuid && (a.handle < service.handle || a.handle >= end));
        if (!matches.length) throw error("External HID report characteristic missing");
        for (const a of matches) await addReport(service.handle,a,map,parsed);
      }
    }
    const inputs = reports.filter(r => r.type === 1);
    if (!inputs.length || inputs.length > 16) throw error("Keyboard requires 1..16 input reports");
    const changed = attrs.find(a => a.uuid === 0x2a05);
    const changedCcc = changed && descriptorsFor(changed.handle).find(a => a.uuid === 0x2902);
    const watches = inputs.map(r => r.handle);
    if (changed && changedCcc) watches.push(changed.handle);
    if (watches.length > 16) throw error("More than 16 subscribed report/Service Changed handles");
    const config: KeyboardConfiguration = { version: 1, epoch: this.epoch, reports,
      serviceChanged: changedCcc ? changed!.handle : undefined };
    this.restore(config);
    try {
      await this.command(Op.watch, [watches.length, ...watches.flatMap(le16)]);
      if (changedCcc) await this.write(changedCcc.handle, Uint8Array.of(2,0));
      for (const r of inputs) await this.write(r.ccc!, Uint8Array.of(1, 0));
    } catch (e) { this.inputs.clear(); this.configuration = undefined; this.release(); throw e; }
    return config;
  }
  /** Persist this with the G2 peripheral identity in the native app. A fresh
   * firmware boot requires enable/connect/discover again; epoch alone must
   * never be used as persistent proof of a matching keyboard after reboot. */
  restore(config: KeyboardConfiguration): void {
    if (config.version !== 1 || !config.reports.length || config.reports.length > 64) throw error("Invalid keyboard configuration");
    const inputs = new Map<number, { report: HidReport; id: number; service: number }>();
    for (const r of config.reports) if (r.type === 1) {
      if (!Number.isInteger(r.handle) || r.handle < 1 || r.handle > 65535 || inputs.has(r.handle)) throw error("Invalid restored Report handle");
      if (!Array.isArray(r.map) || !r.map.length || r.map.length > 512 || r.map.some(b => !Number.isInteger(b) || b < 0 || b > 255)) throw error("Invalid restored Report Map");
      const report = parseReportMap(Uint8Array.from(r.map)).get(r.id);
      if (!report) throw error("Restored Report ID missing");
      inputs.set(r.handle, { report, id: r.id, service: r.service });
    }
    if (!inputs.size || inputs.size > 16) throw error("Invalid restored input count");
    this.release(); this.inputs = inputs; this.configuration = config;
  }
}
