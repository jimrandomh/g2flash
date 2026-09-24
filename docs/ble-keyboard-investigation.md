# BLE keyboard relay investigation

Investigated 2026-09-24 at repository commit `bb4064d`, against the exact
2.3.0.24 donor. This is a feasibility/design report; no firmware implementation
or hardware experiment was performed.

**Assessment: plausible, with useful support already in the stock stack, but
requires a new HID-over-GATT host and changes to application-level BLE event
routing. It is not an extension of the ring gesture decoder alone.** The main
unproven milestone is keeping phone, ring, and keyboard connected while
correctly isolating their security, discovery, and disconnect events.

The intended scope is input to the companion app, including when the phone is
locked or the app is backgrounded. It does not make the companion app a
system-wide keyboard or grant unlimited background execution.

## Evidence in this checkout and donor

| Finding | Evidence | Implication |
| --- | --- | --- |
| The stock host is Cordio; the controller is EM9305. | Donor source-path strings include `third_party/cordio/ble-host/sources/hci/ambiq/em9305/hci_drv_em9305.c`; `patches/ancs_relay.c` documents the WSF task. | Existing GAP, ATT/GATT and SMP infrastructure can potentially be reused. No controller replacement is indicated. |
| The DM connection allocator has three slots. | `dmConnCcbAlloc` at `0x4cd2b8`: base `0x20073c30`, stride `0x30`, loop exits at index 3; connection IDs are index + 1. | Phone + ring + one keyboard on one lens fits this particular host table. This does not prove controller configuration, all other tables, or radio scheduling support the combination. |
| ATT client initialization also iterates three connection indices. | `0x54dd3e`: connection stride `0x84`, three `0x2c` client slots per connection, three outer iterations. | Additional corroboration that the host was not built only for two connections. Client slots are not additional physical links. |
| Stock central discovery is explicitly ring-specific. | `APP_BleServerDiscCback` at `0x551318`; role-zero path at `0x5518d0` logs `discover Ring service` and calls `0x4dddec` at `0x551968`. Discovery state uses the object through `0x200775bc`, including byte `+0x57`. | A keyboard cannot safely run through this discovery state machine unchanged. |
| The ring profile keeps a single current connection and handle list. | `APP_BleRingHandlerInit` at `0x4ddd80` initializes control block `0x2007709c`: connection byte `+0`, handler byte `+1`, handle-list pointer `+4`. | Preserve this ring context; add separate keyboard state. |
| The stock central opener checks ring ownership. | `_ringConnOpenCommon` at `0x4b0f38`, including the `not owner anymore, stop` path; stock also has dominant-hand switching/retry logic. | Calling the ring connection helper with a keyboard address is not a generic connect API. |
| A symbol containing HID is not evidence of a generic HID host. | `RING_CmdHID` at `0x4790b8` checks ring message bytes and logs `ring touch enable ok`. | This inspected function handles a ring acknowledgement, not HID service/report-map discovery. A string search alone cannot prove absence of other HID code. |
| Ring input already reaches the phone, including across temples. | `patches/gesture_fwd.c`: `faceclaw_ring_report`, `faceclaw_input_bridge_received`. | Transport and source attribution have existing examples, though keyboard reports need their own contract. |
| Sending directly from a BLE callback can block the BLE task. | `patches/ancs_relay.c` explains the TX queue's up-to-500 ms wait and uses a producer queue plus timer drain. | Queue copied HID reports in the callback; drain outside the BLE task. |
| Appended code has limited remaining room. | Committed manifest appends 121,552 bytes at `0x7cda60`, ending at `0x7eb530`; conservative ceiling is `0x7f0000`. | Exactly 19,152 bytes (~18.7 KiB) remain under the current ceiling. Keep the firmware small; measure compiled size early. |

These are static observations. Function names recovered from nearby log strings
are labels for further investigation, not validated callable C ABIs. Some source
comments elsewhere in the repository retain older firmware addresses; use the
current donor and instruction evidence when implementing.

The controller vendor advertises multiple simultaneous links, but its published
documents differ by version: one EM9305 fact sheet says four, another says more
than eight. Neither establishes the capacity of Even's loaded controller image.
The three-slot donor evidence is more relevant to this project.
Sources: [EM9305 fact sheet](https://www.emmicroelectronic.com/sites/default/files/products/datasheets/EM9305-FS.pdf),
[alternate EM9305 fact sheet](https://www.emmicroelectronic.com/sites/default/files/products/datasheets/9305-FS.pdf).

## Proposed architecture

```text
BLE keyboard -- HID over GATT --> right lens -- private BLE events --> phone app
R1 ring ------ stock protocol --> either lens ---- existing relay ---> phone app
```

Start with one keyboard on the right lens. That is already the phone-notification
egress used by the extensions. If the ring is on the right, the right needs three
links; if the ring is on the left, each lens needs two. Preserve normal ring
ownership and dominant-hand switching. Do not make the first implementation
move the ring to free a slot. A left-lens keyboard with an inter-temple relay is
a possible later alternative, not an assumed fix for shared application state.

The glasses are a GAP central/GATT client toward the keyboard and remain a GAP
peripheral toward the phone. ANCS already demonstrates GATT client operations,
but its client role toward the phone is not by itself proof of a second GAP
central link; the ring and connection allocator provide that evidence.

Keep pairing, connection control, service discovery, subscription, and bounded
raw report buffering on the glasses. Put report descriptor parsing, keyboard
layouts, modifier/chord handling, text composition, and app commands on the
phone. Prefer native phone handling for restoration and initial buffering so
input does not depend on an already-running JavaScript/UI runtime.

## Required work

### Connection and event routing — highest uncertainty

Recover and validate the low-level connect, scan, cancel, disconnect, ATT read,
descriptor discovery, CCC write, and SMP callback interfaces. Cordio API names
are useful search terms, not permission to assume an ABI from another SDK.

Add a keyboard context keyed by connection ID **and connection generation**.
IDs can be reused after disconnect. Route application callbacks for an owned
keyboard connection before stock ring handlers touch ring globals. Preserve
common stack/security processing and leave phone/ring events on the stock path.
Do not swallow an entire DM/ATT event just because it concerns the keyboard;
some common processing is required for the stack to work.

Audit initiating/scanning arbitration, connection-parameter handling, MTU,
encryption events, service discovery serialization, disconnect cleanup, and
stack resets. Stock globals such as the current ring connection and discovery
state are the main hazards, even if every underlying stack table has space.

### Pairing and reconnect

Provide phone commands for scan, select, connect, cancel, disconnect, forget,
status, and pairing responses. Discovery should use HID advertisements as a
hint and confirm services after connection; allow a bounded broader scan for
devices with incomplete advertisements. Never overwrite the stored R1 address.

Support encrypted bonding, keyboard passkey entry where requested, privacy
addresses/IRKs, and clean bond removal. Display a pairing code on the phone or
glasses when the chosen association procedure calls for typing it on the
keyboard. Check whether stock SMP I/O capability policy is global before
changing it: a keyboard pairing requirement must not alter phone/ring pairing.

Before reusing the stock bond database, determine its capacity, replacement
policy, and how records are classified. Protect phone and ring bonds from
keyboard pairing/forget operations. Reconnect by bonded identity rather than a
cached random address. Handle stale keys and Service Changed/cache invalidation.

### A small HOGP report host

Discover HID services, read each Report Map, associate Report characteristics
with their Report Reference descriptors, and subscribe to all supported input
reports. Forward report identity separately from the characteristic bytes: HOGP
does not use the same on-wire Report ID convention as USB HID. Bonding and
encryption are required. Include output/feature operations and external report
references as compatibility scope grows. These requirements follow
[HOGP sections 4 and 6](https://www.bluetooth.org/docman/handlers/downloaddoc.ashx?doc_id=245141).

Useful UUIDs, from the Bluetooth SIG's
[Assigned Numbers](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Assigned_Numbers/out/en/index-en.html):

| Item | UUID |
| --- | --- |
| HID service | `0x1812` |
| HID Information / Report Map / HID Control Point | `0x2A4A` / `0x2A4B` / `0x2A4C` |
| Report / Protocol Mode | `0x2A4D` / `0x2A4E` |
| Report Reference / External Report Reference / CCC | `0x2908` / `0x2907` / `0x2902` |
| Boot Keyboard Input / Output | `0x2A22` / `0x2A32` |

A boot-keyboard-only prototype can prove the radio path, but is insufficient
for broad compatibility. Report maps vary, and keyboards can have separate
keyboard, consumer-control, and other reports. A fixed eight-byte parser would
miss those layouts. The phone should parse descriptors with strict bounds and
support array and bitmap key sets. See the vendor's
[HID keyboard example](https://docs.silabs.com/bluetooth/2.13/bluetooth-code-examples-applications/ble-hid-keyboard)
for the report-map/report-reference relationship and a basic six-key format.

“Any BLE keyboard” should mean a tested range of standard HOGP devices, with
explicit resource limits and diagnosed unsupported cases. Bluetooth Classic
HID and proprietary USB-dongle protocols are separate transports and outside
this design. Initially support one keyboard concurrently with the ring;
multiple simultaneous keyboards require another capacity assessment.

### Private phone relay

Use a new versioned private settings-channel message family, following the
existing relay pattern. Allocate field numbers after checking the current
firmware and phone decoder; this report reserves none. Keep keyboard records
separate from ring gestures and carry at least:

| Record | Proposed contents |
| --- | --- |
| Device/status | Device identity token, session generation, connected/encrypted/ready state, error and loss counters |
| Descriptor | HID service instance, descriptor generation/hash, length/offset, Report Map chunks, report ID/type mappings |
| Input | Session generation, sequence, glasses timestamp, service instance, report ID/type, length, raw characteristic value |
| Output/control | Request ID, destination report, payload, asynchronous result; constrained to the enrolled HID device |
| Reset/gap | Disconnect, queue overflow, or session change; phone clears held keys and requests current state |

Firmware callbacks must copy data before their stack-owned buffers expire.
Bound report lengths, number of services/reports, descriptors, scan results,
and pending operations. Use nonblocking queues and an appropriate drain path;
prioritize typing over bulk display traffic. Do not silently truncate reports
or overwrite unread records. Preserve fast press/release pairs rather than
coalescing to just the last keyboard state.

For loss recovery, use sequence numbers and a bounded acknowledgement/replay
window. If history is unavailable, report a gap and resynchronize held state;
do not promise to reconstruct missed taps from the latest report. On disconnect
or epoch change the phone must release all held keys. Keep descriptor metadata
available across app restoration, so the first background input is decodable.

**Enable keyboard input independently of display/framebuffer ownership.** The
ring's current `cfw_fb_lease_active()` gate is wrong for always-available keyboard
input. Copying the ANCS 90-second renewal lease is also wrong if a suspended
phone has no other events: after expiry the keyboard could no longer generate
the wake event needed to renew it. Use explicit session enable/disable with
defined phone-link-loss behavior. The first prototype should keep this state
volatile and require an explicit command after boot. Production unattended
reconnect requires a separate review of persistence and startup behavior.

## What this changes on iOS

Apple's explanation is that iOS consumes HID services itself; an app receives
normal keyboard input through system APIs, not arbitrary HID notifications
through Core Bluetooth. That rules out a general phone-only raw-HID relay for
ordinary keyboards using this route.
[Apple engineer explanation](https://developer.apple.com/forums/thread/69814).

With this design, the app subscribes to the glasses' custom characteristic and
receives input as accessory data. My assessment is that this is the appropriate
architecture for background companion-app input. Core Bluetooth supports
background notifications and opt-in state restoration, but gives the app
limited event-processing time; it does not grant permanent execution.
[Apple background processing guide](https://developer.apple.com/library/archive/documentation/NetworkingInternetWeb/Conceptual/CoreBluetooth_concepts/CoreBluetoothBackgroundProcessingForIOSApps/PerformingTasksWhileYourAppIsInTheBackground.html).

Plan for `bluetooth-central`, restoration identifiers, restoration delegate
handling, and prompt native processing of input. Test locked-screen storage
access as well as wake delivery. Normal system keyboard injection into other
apps is not part of the relay.

Do not use the older blanket claim that force-quit can never be recovered:
Apple's current rules add AccessorySetupKit exceptions starting with iOS 26,
including the user-force-quit case. Setup integration for the glasses is thus
worth evaluating, while wake behavior still needs testing on supported iOS
versions. [TN3115](https://developer.apple.com/documentation/technotes/tn3115-bluetooth-state-restoration-app-relaunch-rules),
[2026 Apple DTS clarification](https://developer.apple.com/forums/thread/840468).

## Implementation sequence and acceptance gates

1. **Map the remaining BLE interfaces and callback ownership.** Pin donor bytes
   and update the ABI inventory before using new firmware addresses. Trace
   controller initialization, connection/bond limits, and all shared state on
   the open/security/discovery/close paths. Establish an early code-size budget.
2. **Add an explicitly armed diagnostic prototype.** Stock behavior until a
   custom command; scan and connect to one selected keyboard through its own
   context. First prove phone + ring + keyboard coexistence and clean teardown.
   Do not add autonomous boot reconnect at this stage.
3. **Prove end-to-end input on one keyboard.** Pair, discover, subscribe, relay
   raw reports, parse on phone; demonstrate typing with the ring simultaneously
   active, screen asleep, and phone locked/backgrounded after a long idle.
4. **Broaden compatibility.** Report-mode layouts, consumer keys, output LEDs,
   passkey variants, private addresses, reconnect, invalidated services, and
   multiple HID service instances. Keep unsupported resource cases explicit.
5. **Harden and measure.** Host tests for state transitions, malformed ATT and
   HID data, stale callbacks, descriptor fragmentation, queue overflow, replay,
   and release-all behavior. Hardware tests for ring side switches, bond
   persistence, app restoration, connection order, OTA recovery, bulk display/
   microphone contention, latency, and battery consumption.

The first three-link test is the go/no-go milestone. If it needs a controller
firmware change or widespread boot-time table rewrites, reassess scope before
turning a small runtime extension into a stack reconfiguration project. The
repository's AGENTS.md identifies precisely this increased recovery risk.

Planning estimate, not a measured implementation schedule: allow roughly
**1–2 engineer-weeks for ABI investigation and a one-keyboard prototype**, then
**several more weeks for pairing/reconnect compatibility and iOS integration**,
assuming the existing controller and stack can sustain the links. The
uncertainty is stack integration and device testing, not the bandwidth of key
reports. Broad compatibility cannot be established from static analysis alone.

## Reproducing the strongest binary findings

Donor SHA-256:
`187ccf2bcc5c17a212106e8a376745511e8289c4232b634a7ea94b9bf25a0979`.
For the main application in this exact container,
`file_offset = instruction_address - 0x379bf5` (see `patch_compress.py`).
Decode as little-endian Thumb. Selected witnesses:

```text
0x4cd2b8: fe b5 05 00 df f8 5c 49 00 26 01 e0 76 1c 30 34
0x4cd2c8: 30 00 c0 b2 03 28 80 f2 bd 80 a0 7d 00 28 f5 d1
    allocator loads base through literal 0x4cdc1c = 0x20073c30;
    increments pointer by 0x30, compares slot index to 3, tests in-use +0x16.

0x54dd90: 52 1c 10 00 c0 b2 03 28 db db 5b 1c 18 00 c0 b2
0x54dda0: 03 28 01 da 00 22 f4 e7
    inner and outer ATT-client initialization loops each compare to 3.

0x4dddc8: df f8 b4 07 45 70 00 21 01 70 01 21 01 81 44 60
    ring initialization loads singleton 0x2007709c through 0x4de580;
    sets handler +1, clears connection +0, stores handle-list pointer +4.

0x55195e: df f8 7c 06 01 68 20 00 c0 b2 8c f7 40 fa 29 e2
    role-zero discovery path loads ring handle list and calls 0x4dddec.
```

The code-space calculation comes from the committed append operation in
`patches/cfw_patches.json`, including alignment, rather than an estimated build.
No new offsets in this report have been added to the executable patch manifest.
