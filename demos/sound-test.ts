#!/usr/bin/env bun
// Test the CFW buzzer / UI-sound path (load_image_z mode 5).
//
// REQUIRES the CFW build whose load_image_z dispatches mode 5 to the firmware
// buzzer driver. A "sound message" is just an ordinary ImageRawData update whose
// reassembled buffer starts with 0x05 — it flows through the same image handler,
// but instead of touching the display it queues a tone and returns immediately.
//
//   wire buffer = [0x05][kind][args...]
//     kind 0  [5][0][type]              preset 0..8  (DRV_BuzzerPlayAfterQueue)
//     kind 1  [5][1][note][oct][beat]   one tone     (DRV_BuzzerPlayNote)
//                note 1..7, oct 0..3, beat = duration in ~62ms units
//     kind 2  [5][2]                    stop / silence
//
// PRIMARY QUESTION: does playing a sound BLOCK the firmware until the sound
// finishes, or is it fire-and-forget (osTimer-driven)? We answer it by timing
// each send from write to ack. A short beep gives the baseline message latency;
// a long preset (the ~15x repeating alarm, several seconds of audio) is sent
// right after. If its ack latency ~= the baseline, the driver is async and the
// CPU is free while the sound plays. If the ack is delayed by seconds, it blocks.
//
//   bun examples/sound-test.ts
//
// Env: G2_ARM=L|R (default R), G2_SOUND_ALARM=0 to skip the long alarm test.

import {
  G2Session,
  buildCreateStartUpPageContainer,
  buildImageContainers,
  buildImageRawData,
  type ImageContainerSpec,
} from "g2-kit/ble";
import { startHeartbeat } from "g2-kit/ui";

const ACK_MS = 12_000;
let magic = 100;
const nextMagic = () => (magic = magic >= 255 ? 100 : magic + 1);
const sleep = (ms: number) => new Promise<void>((r) => setTimeout(r, ms));
const ARM = (process.env.G2_ARM === "L" ? "L" : "R") as "L" | "R";

// ---- mode-5 sound payload builders ----
const soundPreset = (type: number) => Uint8Array.from([0x05, 0x00, type & 0xff]);
const soundNote = (note: number, oct: number, beat: number) =>
  Uint8Array.from([0x05, 0x01, note & 0xff, oct & 0xff, beat & 0xff]);
const soundStop = () => Uint8Array.from([0x05, 0x02]);
// raw tone: arbitrary freq (1..20000 Hz), duty (0..100%), duration ms — both 16-bit LE.
const soundTone = (freq: number, duty: number, ms: number) =>
  Uint8Array.from([0x05, 0x03, freq & 0xff, (freq >> 8) & 0xff, duty & 0xff, ms & 0xff, (ms >> 8) & 0xff]);

let soundSession = 200; // unique map-session id per sound (fresh single-frag stream each time)

// Send one sound message as a single-fragment ImageRawData update and time the
// write->ack round trip. Returns the elapsed ms (or -1 if it never ack'd).
async function playSound(
  session: G2Session,
  container: ImageContainerSpec,
  payload: Uint8Array,
  label: string,
): Promise<number> {
  const sid = soundSession++;
  const raw = buildImageRawData({
    containerId: container.containerId,
    containerName: container.name,
    mapSessionId: sid,
    mapTotalSize: payload.length,
    mapFragmentIndex: 0,
    mapFragmentPacketSize: payload.length,
    mapRawData: payload,
    magic: nextMagic(),
    // compressMode defaults to 0 -> frag_write copies the bytes verbatim, so the
    // recon buffer is exactly [5][...] and load_image_z sees mode 5.
  });
  const t0 = performance.now();
  const ack = await session.sendPb(0xe0, raw.pb, raw.magic, { ackTimeoutMs: ACK_MS, arm: ARM });
  const dt = performance.now() - t0;
  if (!ack) {
    console.log(`  ${label.padEnd(28)} NO_ACK after ${dt.toFixed(1)}ms`);
    return -1;
  }
  console.log(`  ${label.padEnd(28)} ack in ${dt.toFixed(1)}ms`);
  return dt;
}

const session = await G2Session.open();
const hb = startHeartbeat({ session, nextMagic });
const suffix = String(Date.now() % 10_000).padStart(4, "0");
const BOOT = `b${suffix}`;
const IMG = `c${suffix}`;

try {
  // A container/session must exist for the image path to run, even though mode 5
  // ignores its dimensions. A small container is fine.
  const create = buildCreateStartUpPageContainer({
    name: BOOT, items: ["."], containerId: 1, captureEvents: false, magic: nextMagic(), extraContainerNames: [IMG],
  });
  if (!(await session.sendPb(0xe0, create.pb, create.magic, { ackTimeoutMs: ACK_MS, arm: ARM }))) throw new Error("CREATE did not ack");

  const container: ImageContainerSpec = { name: IMG, containerId: 2, x: 0, y: 0, width: 64, height: 64 };
  const rebuild = buildImageContainers({ containers: [container], magic: nextMagic() });
  if (!(await session.sendPb(0xe0, rebuild.pb, rebuild.magic, { ackTimeoutMs: ACK_MS, arm: ARM }))) throw new Error("REBUILD did not ack");

  await playSound(session, container, soundTone(100, 50, 40), `  100hz tick`);
  await sleep(40);
  await playSound(session, container, soundNote(0, 0, 1), `note oct beat`);

  await sleep(500);
  await playSound(session, container, soundTone(150, 50, 40), `  150hz tick`);
  await sleep(40);
  await soundStop();

  await sleep(500);
  await playSound(session, container, soundTone(200, 50, 40), `  200hz tick`);
  await sleep(40);
  await soundStop();
  await sleep(500);

  console.log(`\n=== buzzer mode-5 sound test (arm ${ARM}) ===`);

  // 1) Baseline: preset 0 is a single short beep. Its ack latency is our yardstick
  //    for "a normal small message round trip."
  console.log("\n[1] single beep (preset 0) — baseline message latency:");
  const base = await playSound(session, container, soundPreset(0), "preset0 (single beep)");
  await sleep(800);

  async function play_preset_and_sleep(preset: number) {
    await playSound(session, container, soundPreset(preset), `preset${preset}`);
    await sleep(2000);
  }
  await play_preset_and_sleep(0); //beep
  await play_preset_and_sleep(1); //alarm
  await play_preset_and_sleep(2); //ring
  await play_preset_and_sleep(3); //beep
  await play_preset_and_sleep(4); //two beeps
  await play_preset_and_sleep(5); //three low beeps
  await play_preset_and_sleep(6); //falling
  await play_preset_and_sleep(7); //rising
  await play_preset_and_sleep(8); //beep

  // 2) A few notes to confirm the parametric path works (ascending run). Space
  //    them out so each tone finishes before the next resets the buzzer.
  console.log("\n[2] parametric notes (kind 1) — ascending run plus various others:");
  for (const [note, oct, beat] of [
    [1, 2, 1],
    [1, 2, 2],
    [1, 2, 3],
    [1, 2, 4],
    [1, 2, 5],
    [2, 2, 1],
    [3, 2, 1],
    [4, 2, 1],
    [5, 2, 1],
    [6, 2, 1],
    [7, 2, 1],
    [8, 2, 1],
    [9, 2, 1],
    [10, 2, 1],
    [11, 2, 1],
    [12, 2, 1],
    [13, 2, 1],
    [2, 2, 2],
    [3, 2, 2],
    [4, 2, 2],
    [5, 2, 2],
    [6, 2, 2],
    [7, 2, 2],
    [1, 3, 2],
    [1, 0, 2],
    [2, 0, 2],
    [3, 0, 2],
    [4, 0, 2],
    [5, 0, 2],
    [3, 0, 2]
  ] as const) {
    await playSound(session, container, soundNote(note, oct, beat), `note ${note} oct ${oct} beat ${beat}`);
    await sleep(600);
  }
  await sleep(600);

  // 3) THE BLOCKING TEST: preset 1 is the repeating alarm (~15 iterations, several
  //    seconds of audio). If mode 5 blocked until the sound finished, this ack
  //    would arrive seconds late. Compare its latency to the baseline above.
  if (process.env.G2_SOUND_ALARM !== "0") {
    console.log("\n[3] BLOCKING TEST — long alarm (preset 1, multi-second audio):");
    const alarm = await playSound(session, container, soundPreset(1), "preset1 (long alarm)");

    // Immediately fire 3 quick beeps back-to-back. If these ack promptly while the
    // alarm would still be sounding, the firmware clearly did NOT block on it.
    console.log("    firing 3 rapid follow-up messages right after the alarm ack:");
    for (let i = 0; i < 3; i++) {
      await playSound(session, container, soundNote(4, 2, 1), `  rapid follow-up #${i + 1}`);
    }

    if (base >= 0 && alarm >= 0) {
      const ratio = alarm / Math.max(1, base);
      console.log(
        `\n    baseline beep ack = ${base.toFixed(1)}ms; long-alarm ack = ${alarm.toFixed(1)}ms (${ratio.toFixed(1)}x).`,
      );
      console.log(
        ratio < 3
          ? "    => ack returned as fast as a normal message: buzzer is ASYNC / NON-BLOCKING."
          : "    => ack was delayed ~by the sound length: playback appears to BLOCK the handler.",
      );
    }
    // Silence whatever is still playing so the test ends quietly.
    await sleep(500);
    await playSound(session, container, soundStop(), "stop");
  }

  // 4) RAW TONES (kind 3): arbitrary frequency/duty/duration — bypasses the note
  //    table entirely. Proves the hardware takes any Hz, not just 7 notes x 4 oct.
  if (process.env.G2_SOUND_RAW !== "0") {
    console.log("\n[4] RAW TONES (kind 3) — arbitrary Hz / duty / ms:");

    // (a) a chromatic-ish scale by equal-tempered frequency (A4=440 up an octave).
    console.log("  (a) equal-tempered scale, 440->880 Hz, 180ms each @ 50% duty:");
    for (let semitone = 0; semitone <= 12; semitone++) {
      const freq = Math.round(440 * 2 ** (semitone / 12));
      await playSound(session, container, soundTone(freq, 50, 180), `  ${freq} Hz`);
      await sleep(220);
    }
    await sleep(400);

    // (b) a fast rising chirp — a run of short raw tones stepping in frequency.
    //     Each tone auto-stops after its ms; sending them back-to-back sweeps pitch.
    console.log("  (b) rising chirp 300->3000 Hz in 40ms steps:");
    for (let f = 300; f <= 3000; f += 180) {
      await playSound(session, container, soundTone(f, 50, 45), `  ${f} Hz`);
      await sleep(45);
    }
    await sleep(400);

    // (c) duty-cycle sweep at a fixed pitch — does duty audibly change timbre/level?
    console.log("  (c) duty sweep at 1000 Hz, 200ms each (10%..90%):");
    for (const duty of [10, 25, 50, 75, 90]) {
      await playSound(session, container, soundTone(1000, duty, 200), `  duty ${duty}%`);
      await sleep(260);
    }
    await playSound(session, container, soundStop(), "stop");
  }

  console.log("\ndone.");
} finally {
  hb.stop();
  await session.close();
}

process.exit(0);
