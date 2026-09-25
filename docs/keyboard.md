# BLE keyboard bridge — experimental, revision 39

The `keyboard` branch implements a right-lens BLE central connection alongside
the existing R1 ring. HID reports travel over the glasses' private GATT protocol
to the companion app. The keyboard pairs to the **glasses**, not to iOS's system
keyboard service.

This is an implementation for hardware bring-up, **not a hardware-validated
release**. The first physical installation passed OTA verification, subsequent
BLE authentication and a disabled-keyboard status query; see the installation
record below. The host tests cannot
prove that the controller will maintain phone + ring + keyboard connections,
that a particular keyboard's pairing succeeds, or that iOS wakes the native
app correctly. The [investigation](ble-keyboard-investigation.md) explains the
remaining controller and iOS questions.

## Implemented

- Explicit enable, five-second scan, selected-device connect, disconnect,
  encrypted ATT access, and status queries. Public, random and resolved identity
  address types are accepted. One keyboard on the right lens; ring events retain
  their stock path on either lens.
- Per-connection KeyboardDisplay SMP capability, passkey and numeric-comparison
  callbacks, stock key storage/reuse. A new keyboard bond is refused when it
  would evict an existing bond. OOB-only pairing is rejected.
- Phone-side HOGP discovery: primary HID services, variable handles, Report
  Maps, Report References, external reports (such as Battery Service reports),
  Report Protocol selection, CCC subscriptions, and Service Changed invalidation.
- Raw input/indication forwarding with characteristic handle, epoch, sequence,
  timestamp and fragmentation. The phone maps handles to HID service/report IDs;
  HOGP report values do **not** include an extra USB-style Report ID byte.
- Bounded HID decoder for keyboard arrays, modifiers, bitmap/NKRO reports and
  consumer keys. A per-report union generates usage-page/usage key-down/up events.
  Text layout/IME conversion belongs to the app. Other HID usages remain available
  in the raw callback; this is not an OS-wide keyboard injector.
- Small output/feature writes through the owned connection (including LED
  reports), with an asynchronous ATT result. Write Command success means queued
  by the host; it cannot acknowledge processing by the keyboard.
- Sequence gaps, disconnects and malformed reports release held keys. Heartbeats
  expose a lost final release even when no more keys arrive. There is no replay
  guarantee for missed taps.

The feature has **no framebuffer/display lease and no phone-renewal timeout**.
Once configured, it keeps forwarding through long idle periods. State is
volatile: reboot returns to stock behavior. Initial enable/configuration and
reconnection after a keyboard disconnect require the client. There is no boot
reconnect, autonomous private-address resolution policy, bond-forget UI, or
native iOS app implementation in this repository.

## Try the reference client

After separately installing and hardware-validating this branch's firmware:

```sh
cd demos
bun keyboard.ts scan
bun keyboard.ts connect aa:bb:cc:dd:ee:ff random
bun keyboard.ts status
bun keyboard.ts disable
```

Put the keyboard into BLE pairing mode first. Scan prints addresses in the usual
most-significant-byte-first notation; the client reverses them for Cordio.
The other address-type arguments are `public`, `public-identity` and
`random-identity`. An already-running stock scan/initiation returns BUSY;
retry once the ring connection settles. Scanning does not disconnect the ring.
A connect attempt times out after 30 seconds. The CLI shows pairing prompts and
prints HID usages; Ctrl-C after setup disables/disconnects the keyboard.

`keyboard-client.ts` is transport-independent. `keyboard.ts` supplies its G2 BLE
transport and subscribes to **raw right-lens sid-0x09 notifications**, extracting
field 131. The ordinary settings ACK acknowledges parsing only; field 131 is the
actual command result. The client needs `GLASSLYCFW/39` or later. `status` and
`disable` do not enable a connection.

Native integration must implement the same field framing and store the discovered
mapping with the peripheral identity. Restore that mapping before delivering
input on an **unchanged glasses/keyboard session**. Epoch is a 16-bit connection
generation, not a persistent boot identity: do not trust a saved mapping after
reboot/re-enrollment. Call `lostPhoneLink()` on link loss and `configure()` after
Service Changed (the `servicesChanged` callback reports invalidation). Explicit
`connect()` clears old maps and a new setup discovers fresh handles.

On iOS the native receiver needs `bluetooth-central`, an appropriate
CoreBluetooth restoration identifier and restoration handling. Process events
within the execution time iOS grants; a JavaScript UI being suspended must not
prevent the native BLE handler from decoding/releasing keys. This changes the
input transport to app-accessible GATT; it does not remove Apple's execution,
relaunch or force-quit rules. See the linked Apple sources in the investigation.

## Wire contract

All multi-byte integers are little endian. Controls are a length-delimited
**field 130** on a sid-0x09 settings request. Existing settings processing still
runs. Unknown fields are skipped by stock peers.

Control prefix: `['K','B',1,op, request:u16, epoch:u16]`. Request IDs must be
correlated by the client. ENABLE and QUERY do not require a matching epoch;
other controls do. At most 80 bytes including the prefix.

| Op | Name | Body |
| --- | --- | --- |
| 0 | QUERY | empty |
| 1 | ENABLE | empty; idempotent |
| 2 | DISABLE | empty; disconnect, leave existing bond |
| 3 | SCAN | empty; active 1M scan, five seconds |
| 4 | CONNECT | address type:u8, address:6 bytes (LSB first) |
| 5 | DISCONNECT | empty |
| 6 | PAIR | empty; use stored key or initiate bonding |
| 7 | AUTH | passkey:u24, 0..999999; only for a pending prompt |
| 8 | COMPARE | accept:u8, 0 or 1; only for a pending comparison |
| 9 | FIND | start handle:u16, end handle:u16; one Find Information response |
| 10 | READ | handle:u16, offset:u16; offset 0 = Read, nonzero = Read Blob |
| 11 | WRITE | handle:u16, without-response:u8, length:u8, value (up to 18 bytes) |
| 12 | WATCH | count:u8, input/indication handles:u16[count], at most 16 |

SCAN requires no owned connection and an idle stock scanner/initiator.
CONNECT requires a free host slot and no scan. ATT controls and WATCH require
an encrypted owned link. Only one ATT request is in flight; timeout closes the
owned link and retains ownership until its close event, so a late response cannot
be mistaken for a new transaction. An allocation failure or unknown/malformed
control may yield no result; the client has a bounded timeout.

Firmware error codes: 0 OK, 1 bad request, 2 disabled, 3 busy, 4 stale epoch,
5 no connection/bond space, 6 not connected, 7 not encrypted, 8 timeout,
9 allocation failure. ATT response status is the Cordio/ATT status, not one of
these control error codes.

Events are true sid-0x09 EVENT notifications with **field 131**:

| Byte offset | Content |
| --- | --- |
| 0..3 | `'K','B',1,kind` |
| 4..7 | event sequence:u32 (same for all fragments) |
| 8..9 | connection epoch:u16 |
| 10..11 | request ID:u16 (zero for unsolicited events) |
| 12..13 | code:u16 |
| 14..15 | handle/value:u16 |
| 16..17 | total body length:u16 |
| 18..19 | fragment offset:u16 |
| 20..23 | glasses millisecond timestamp:u32 |
| 24.. | body fragment, at most 96 bytes |

| Kind | Code | Body |
| --- | --- | --- |
| 1 STATUS | control error | enabled, connId, connected, encrypted, scanning, pending ATT event (six u8), cumulative drops:u32 |
| 2 ATT | event:u8, ATT status:u8 | raw ATT response value; request ID matches control; handle from ATT event |
| 3 INPUT | 13 notification / 14 indication | raw characteristic value; handle identifies the configured report |
| 4 DM | DM event:u8, status:u8 | normally empty; 0x2e has OOB/display bytes; 0x35 has six-digit comparison value:u32 |
| 5 ADVERT | 0 | addrType:u8, addr[6], RSSI:i8, eventType:u8, dataLen:u8, AD data |

No pairing keys are exported. DM events of interest: 0x27 open, 0x28 close,
0x2a paired, 0x2b pairing failed, 0x2c encrypted, 0x2d encryption failed,
0x2e passkey/OOB request, 0x35 numeric comparison. The comparison value is
computed with the stock confirm-value convention, not copied from the first
four bytes of the confirm buffer.

The firmware atomically queues a whole event or drops the whole event, consuming
its sequence. The 32-record SPSC queue never overwrites unread input. Four records
per 20 ms timer tick drain through the stock phone sender, outside the BLE task.
One-second status heartbeats expose loss. The one-shot timer parks after a
disabled instance drains, so a status query does not leave a permanent timer loop. Failed stock ATT callback allocations
also consume a sequence. Lost DM callback allocations use an immediate WSF-only
keyboard fallback, preserving close ownership. A send refusal creates a gap;
the relay does not retry old keystrokes.

## Bounds and validation status

HID maps/attributes/reports are limited to 512 bytes, GATT discovery to 512
attributes, and subscriptions to 16 handles including Service Changed. The
reference decoder rejects malformed descriptors, long HID items, usage delimiters,
mixed numbered/unnumbered input reports and excessive stack/report dimensions.
It detects keyboard rollover reports and clears held keys. Unsupported devices
fail setup with a diagnostic instead of silently falling back to a fixed layout.
Bluetooth Classic keyboards and proprietary USB dongles are outside HOGP.

The bridge uses the stock negotiated ATT MTU and has no separate MTU-exchange
control. Large-report keyboards therefore need MTU interoperability testing.
Bond-table exhaustion is reported; this implementation deliberately provides no
bulk erase operation. A keyboard with stale keys may need its bond removed using
the existing device tools before a fresh pairing.

Offline checks:

```sh
python3 patches/host/run_vector_tests.py
python3 patches/gen_patches.py g2_2.3.0.24.bin /tmp/keyboard-patches.json
```

The suite runs sanitizer-backed production C state-machine tests, TypeScript
protocol/descriptor tests, a simulated HOGP server (long reads, multiple reports,
external Battery report and Service Changed), existing display/ANCS/ring
regressions, and the freestanding Thumb relocation checks. The build pins the
exact donor SHA, reviewed stock bytes and the existing MRAM ceiling.

The [keyboard emulator verification](keyboard-emulator.md) additionally passes
eight isolated ARM scenarios against the exact committed image using
PaulMcMillan/g2-firmware-emulator. It covers production trampolines, relay and
failure handling with explicit stock-function shims. Full-system boot, real
Bluetooth coexistence and iOS background delivery remain unverified.

### First physical installation — 2026-09-24 (Pacific)

Installed image SHA-256
`4c7bf9a552df1a42f26b3378425c7c2121ca4058fed7283ec863c3f8d2cd13fe`
on both lenses. Each lens verified all six OTA components with status 8
(`UPDATING`), zero block resends, and no transfer errors. Transfers took 305 s
left and 309 s right. Both lenses subsequently reconnected and authenticated.
The right-lens settings response reported both base versions as `2.3.0.24` and
custom revision `GLASSLYCFW/39`. Direct left-lens settings queries timed out both
before and after installation; its custom revision was not independently read.

The reference client's status query returned `enabled=false`, `conn=0`,
`connected=false`, `encrypted=false`, `scanning=false`. No keyboard was paired
and no ring/keyboard coexistence or native iOS test was performed. The status
CLI remained alive after printing its disconnect message and was terminated;
its command-line shutdown lifecycle needs follow-up. This is one successful
installation and dormant control check, not general hardware qualification.

Hardware acceptance still required before calling this a supported firmware:

1. Verify dormant boot, OTA availability, phone and R1 reconnect on both lenses.
2. Enable on right; keep phone + ring + keyboard connected simultaneously and
   exercise ring input while typing, including with the display asleep.
3. Pair Just Works, passkey and Secure Connections keyboards; confirm ring/phone
   pairing policy and existing bonds survive. Test a full bond table.
4. Exercise arrays, bitmap reports, media keys, LEDs and long reports. Test
   keyboard sleep/wake, disconnect/cancel, stale keys, RPA rotation and reconnect.
5. Force backpressure and phone loss while a modifier is held. Confirm releases,
   gaps, no replayed stale taps, fresh discovery after Service Changed.
6. In the native iOS app, test a locked phone and a long idle background session,
   restoration, and supported system/user termination cases. No such test has
   been performed by the desktop reference client.
