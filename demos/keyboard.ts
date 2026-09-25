#!/usr/bin/env bun
import { G2Session } from "g2-kit/ble";
import { findLenDelimField, queryGlasslyCfw } from "./glassly-cfw";
import { KeyboardClient } from "./keyboard-client";
import { KeyboardOp } from "./keyboard-protocol";
import { createInterface } from "node:readline/promises";
import { randomInt } from "node:crypto";

const addressTypes = ["public", "random", "public-identity", "random-identity"];
function keyName(page: number, usage: number): string {
  if (page !== 7) return `page=0x${page.toString(16)} usage=0x${usage.toString(16)}`;
  if (usage >= 4 && usage <= 29) return String.fromCharCode(65 + usage - 4);
  if (usage >= 30 && usage <= 39) return "1234567890"[usage - 30]!;
  if (usage >= 58 && usage <= 69) return `F${usage - 57}`;
  const names: Record<number, string> = {
    40: "Enter", 41: "Escape", 42: "Backspace", 43: "Tab", 44: "Space",
    45: "-", 46: "=", 47: "[", 48: "]", 49: "Backslash", 51: ";", 52: "Quote",
    53: "Backtick", 54: ",", 55: ".", 56: "/", 57: "Caps Lock",
    73: "Insert", 74: "Home", 75: "Page Up", 76: "Delete", 77: "End", 78: "Page Down",
    79: "Right", 80: "Left", 81: "Down", 82: "Up",
    224: "Left Ctrl", 225: "Left Shift", 226: "Left Alt", 227: "Left Meta",
    228: "Right Ctrl", 229: "Right Shift", 230: "Right Alt", 231: "Right Meta",
  };
  return names[usage] ?? `key 0x${usage.toString(16)}`;
}
const [action = "scan", address, addrType = "random"] = process.argv.slice(2);
if (!["scan", "connect", "disable", "status"].includes(action) ||
    (action === "connect" && (!address || !/^(?:[0-9a-f]{2}:){5}[0-9a-f]{2}$/i.test(address) || !addressTypes.includes(addrType)))) {
  console.error("usage: bun keyboard.ts scan|status|disable\n       bun keyboard.ts connect AA:BB:CC:DD:EE:FF random|public");
  process.exit(2);
}
const session = await G2Session.open();
let off: (() => void) | undefined;
let client: KeyboardClient | undefined;
try {
  const cfw = await queryGlasslyCfw(session, 29);
  if (!cfw || cfw.revision < 39) throw new Error("Keyboard bridge requires GLASSLYCFW/39 or later");
  const seen = new Set<string>();
  let lastDebugStatus = "";
  client = new KeyboardClient({ async send(pb, magic) {
    await session.sendPb(9, pb, magic, { arm: "R", ackTimeoutMs: 4000 });
    // The stock settings ACK only acknowledges parsing. Private field 131
    // carries the actual asynchronous command result.
  } }, {
    keys(events) { for (const e of events) console.log(`${e.down ? "down" : "up  "} ${keyName(e.page, e.usage)}`); },
    gap() { console.log("keyboard stream reset/gap: cleared held keys"); },
    error(e) { console.error(e.message); },
    record(r) {
      if (process.env.KEYBOARD_DEBUG && r.kind !== 3 && r.kind !== 5) {
        const detail = `kind=${r.kind} epoch=${r.epoch} request=${r.request} code=0x${r.code.toString(16)} handle=${r.handle} data=${Buffer.from(r.data).toString("hex")}`;
        const status = `${r.epoch}:${r.code}:${Buffer.from(r.data).toString("hex")}`;
        if (r.kind !== 1 || status !== lastDebugStatus) console.error(`[keyboard] ${detail}`);
        if (r.kind === 1) lastDebugStatus = status;
      }
      if (r.kind === 5 && r.data.length >= 10) {
        const p = r.data;
        const addr = Array.from(p.slice(1, 7)).reverse().map(x => x.toString(16).padStart(2, "0")).join(":");
        let name = "", hid = false;
        for (let i = 10; i < p.length;) {
          const n = p[i++]!;
          if (!n || i + n > p.length) break;
          const type = p[i]!;
          if (type === 8 || type === 9) name = new TextDecoder().decode(p.subarray(i + 1, i + n));
          if (type === 2 || type === 3) for (let j = i + 1; j + 1 < i + n; j += 2) if (p[j] === 0x12 && p[j + 1] === 0x18) hid = true;
          i += n;
        }
        const key = `${addr}:${name}:${hid}`;
        if (!seen.has(key)) {
          seen.add(key);
          console.log(`${addr} ${addressTypes[p[0]!]} RSSI=${(p[7]! << 24) >> 24} ${hid ? "HID " : ""}${name}`);
        }
      }
    },
    async passkey(display) {
      if (display) {
        const pin = randomInt(1000000);
        console.log(`Type ${String(pin).padStart(6, "0")} on the keyboard, then Enter.`);
        return pin;
      }
      const rl = createInterface({ input: process.stdin, output: process.stdout });
      try { const value = await rl.question("Pairing passkey: "); if (!/^\d{6}$/.test(value)) throw new Error("Expected six digits"); return Number(value); }
      finally { rl.close(); }
    },
    async compare(value) {
      const rl = createInterface({ input: process.stdin, output: process.stdout });
      try { return (await rl.question(`Does the keyboard show ${String(value).padStart(6, "0")}? [yes/no] `)).toLowerCase() === "yes"; }
      finally { rl.close(); }
    },
  });
  off = session.onRawFrame((frame, _raw, arm) => {
    if (arm !== "R" || !frame.ok || frame.sid !== 9) return;
    const field = findLenDelimField(frame.pb, 131);
    if (field) client!.accept(field);
  });
  if (action === "scan" || action === "connect") await client.enable();
  else await client.command(KeyboardOp.query);
  if (action === "scan") await client.scan();
  else if (action === "disable") await client.disable();
  else if (action === "status") { await client.command(KeyboardOp.query); console.log(client.status); }
  else {
    const bytes = Uint8Array.from(address!.split(":").reverse().map(x => parseInt(x, 16)));
    await client.connect(addressTypes.indexOf(addrType), bytes);
    const config = await client.configure();
    console.log(`Ready: ${config.reports.filter(r => r.type === 1).length} input reports. Ctrl-C disconnects the keyboard.`);
    await new Promise<void>(resolve => { process.once("SIGINT", resolve); process.once("SIGTERM", resolve); });
    await client.disable();
  }
} finally {
  off?.(); client?.close(); await session.close();
}
// Noble's native Bluetooth event loop can keep Bun alive after disconnect.
process.exit(0);
