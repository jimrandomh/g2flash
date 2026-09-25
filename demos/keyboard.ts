#!/usr/bin/env bun
import { G2Session } from "g2-kit/ble";
import { findLenDelimField, queryGlasslyCfw } from "./glassly-cfw";
import { KeyboardClient } from "./keyboard-client";
import { KeyboardOp } from "./keyboard-protocol";
import { createInterface } from "node:readline/promises";
import { randomInt } from "node:crypto";

const addressTypes = ["public", "random", "public-identity", "random-identity"];
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
  client = new KeyboardClient({ async send(pb, magic) {
    await session.sendPb(9, pb, magic, { arm: "R", ackTimeoutMs: 4000 });
    // The stock settings ACK only acknowledges parsing. Private field 131
    // carries the actual asynchronous command result.
  } }, {
    keys(events) { for (const e of events) console.log(`${e.down ? "down" : "up  "} page=0x${e.page.toString(16)} usage=0x${e.usage.toString(16)}`); },
    gap() { console.log("keyboard stream reset/gap: cleared held keys"); },
    error(e) { console.error(e.message); },
    record(r) {
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
