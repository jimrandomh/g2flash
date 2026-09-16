# Dashboard connection status — G2 2.2.9.22

Branch: `connection-status`, created from `main` (the previous checkout was
`2.3.0-CFW`). This patch follows main's **2.2.9.22** firmware base. It is not a
2.3.0 or 2.2.10.10 port. No glasses were flashed during development.

## Behavior

The stock dashboard gains a diagnostic strip in its bottom 52 rows:

```
L Connected from: AA:BB:CC:DD:EE:FF
R Connected from: AA:BB:CC:DD:EE:FF
```

Each lens renders only its own row. The separate vertical positions prevent
conflicting text in binocular view if one side has no link or the two
connections have different addresses. A lens without a phone link displays
`L Phone Bluetooth: disconnected` or `R Phone Bluetooth: disconnected`.
A live ID whose address is not yet usable displays
`Connected from: address unavailable` rather than inventing a MAC.

The status comes from the raw phone-facing peripheral connection ID and
Cordio connection record, independently of any control-app message, settings
request, EvenHub session, or aggregate UX callback. While the dashboard is
visible, an idle display-task receive checks for a change every 500 ms. Only
changes cause a synthetic refresh, and only after a nonblocking acquisition of
the normal stock display-buffer semaphore. Real queued messages are returned unchanged.
A dashboard repaint always resamples the connection, including on entry.

The existing crossed-out stock icon is retained as the original combined
readiness indication. The new text explicitly exposes link occupancy. The
shared `UX_GetSystemBLEStatus` AND condition and its feature-gating consumers,
advertising policy, connection parameters, pairing, and app protocols are not
changed by this feature. This diagnoses a phone occupying a connection; it does
not disconnect it or enable another simultaneous phone connection.

The strip is drawn after the stock compositor copy, before the existing panel
refresh, using the existing 6x12 font scaled to 12x24. It reserves the bottom
52 rows of the 576x288 dashboard area and can cover stock content there. It
uses the stock even/clamped lens offsets and never modifies LVGL's source
buffer. Other foreground applications, sleeping display state, and a current
Faceclaw framebuffer lease suppress the overlay/polling. A normal subsequent
stock copy removes the strip on leaving the dashboard. There is no new heap
allocation, timer, startup request, or BLE callback.

## Can the phone name be identified?

Bluetooth LE's connection record contains the peer address, not a friendly
name. The standard permits a separate ATT read of the GAP Device Name
characteristic (`0x2A00`) when the peer exposes it. That is a potential future
best-effort enhancement, not a name supplied by link establishment. See the
[Bluetooth GAP specification](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-60/out/en/host/generic-access-profile.html).

The recovered G2 phone discovery path handles GATT/service-change/database-hash
and optional ANCS work; no verified phone Device Name cache or existing
Device Name read was identified. The app BLE advertising-name getter returns
the *glasses' own name*, and the central-role target-name storage belongs to
the *ring*: neither identifies the connected phone. A model/name cannot be
inferred reliably from these addresses. This implementation therefore uses the
user-requested MAC fallback immediately, without introducing competing ATT
transactions or requiring the phone app to announce itself.

The display uses the six bytes returned by the Cordio peer-address layout,
formatted in reverse byte order with uppercase hex and colons. It is the
address in the current connection record; do not interpret it as a permanent
hardware identifier. Bluetooth privacy/identity resolution can affect which
address is exposed and its stability across connections. Friendly-name reads
would need their own ATT sequencing, failure/timeout handling, and host tests.

## Recovered stock ABI and patch boundary

`connection_status_abi.json` pins the complete base image SHA-256 and hashes
18 supporting instruction/literal ranges. Generation fails before compilation
if a guard or the base identity differs. Addresses were checked against the
2.2.9.22 component's actual Thumb instructions and diagnostic strings, using
the older OpenCFW recovery only as a starting point. In particular, the local
ID getter is **0x4745C4**, not the nearby previous function's return at
0x4745C2; a raw wildcard byte match alone would have given the wrong entry.

| Signal/seam | 2.2.9.22 evidence |
|---|---|
| Local phone ID | Getter `0x4745C4`: pointer at `0x20076554`, then byte `+0x54`; reads guarded directly |
| Cordio peer address | `DmConnPeerAddr`, `0x4CB3D2`: table `0x20072D68`, three records, stride `0x30`, address `+0` |
| Record validity/role | `DmConnInUse`, `0x4CB3AE`: byte `+0x16`; `DmConnRole`, `0x4CB956`: byte `+0x19`, role 1 peripheral |
| Display/app state | `getRunningAppID`, `0x445074`: active byte `0x20077298`, startup type `0x20076774`, base app `0x20076770`, foreground app `0x20076778`; dashboard ID 1 |
| Lens identity | `0x45CFDC`: byte `0x200773E3`; 1 right, 2 left |
| Framebuffer | Stock copier `0x4708D0`, destination pointer `0x200008B4`; packed 640x480 4bpp |
| Lens origin | Copier loads x from `0x20000744 + 0x2000074C` and y from `0x20000748 + 0x20076760`; caps x/y at 64/192 and rounds down to even |
| Display queue receive | **Only new stock edit:** BL at `0x4798E0`, old `c9 f7 19 fd`, calls `osMessageQueueGet` at `0x443316` |
| Timeout ABI | `0x443370..0x44338F`: timed receive failure returns -2; real results pass through |
| Buffer semaphore | Handle at `0x2007698C`; stock lock calls `xQueueSemaphoreTake` at `0x442388` (1 success, 0 unavailable); consumer calls `display_buffer_unlock` at `0x4794CE` |
| Synthetic message | Nine words, zeroed except `[0]=3, [5]=640, [6]=480`, matching `send_async_reflash_msg` at `0x479D82` |
| Normal consumer | `0x4798F2` copy hook, then `0x4798F6` stock unlock; panel update follows the normal type-3 path |
| CFW storage | 12 bytes at `0x2029F4B0`, after the two existing anchor words in the already reserved 1 KiB TLSF tail; fixed storage, no stored pointers |

The link snapshot copies only eight bytes under a short PRIMASK save/restore.
It performs no calls to the Bluetooth stack while masked. Pointer/ID checks
prevent out-of-range record indexing; a live central/ring record is rejected.
The render snapshot is owned exclusively by the display-driver task, and its
magic is cleared on non-dashboard stock renders. The context allocator's
existing ownership is unchanged.

## Build and verification

```
python3 -m unittest discover -s tests -v
python3 patches/gen_patches.py g2_2.2.9.22.bin patches/cfw_patches.json
./build_cfw.sh --skip-venv
```

After regeneration, `OUT_SHA256` in `build_cfw.sh` must match the generated
patch set. The usual compiler-free patch application still checks base hash,
expected old bytes, output hash, and the existing container/CRC/MRAM limits.

The tests exercise the actual C implementation with mocked firmware memory
and calls, under ASan/UBSan: no app traffic, IDs 1–3, invalid pointers/IDs,
central/ring rejection, invalid addresses, MAC byte order, disconnects,
foreground/sleep/lease gating, real-message preservation, unchanged-link
suppression, semaphore contention, and raster bounds. They also corrupt every
pinned ABI range to verify that generation refuses it. No new phone-side code
is required. A rendered preview was inspected from the actual C raster output;
it is not a photograph or hardware test.

### Hardware validation still required

- Reproduce the iPhone-connected/app-stopped case on both sides and confirm
  link text updates within the next idle 500 ms check or normal refresh.
- Check all four watchfaces for readability/content overlap and verify the
  L/R rows align comfortably in binocular view with the user's lens offsets.
- Verify open, close, reconnect, address changes, one-sided connections, and
  ring-only connections; compare addresses against controller traces.
- Exercise sleep/wake, dashboard exit/re-entry, active EvenHub/Faceclaw frames,
  and a busy display queue; confirm stock timeouts and panel power behavior.
- Check OTA recovery and real-device battery impact. Software-only checks do
  not establish hardware safety, correct physical alignment, or power behavior.

The baseline local clang already differed from the committed main patch set.
Regeneration recompiles the complete existing injection blob, so the generated
binary diff is larger than the added connection-status feature. Review source
changes and the one additional stock hook separately from compiler drift.

## Generated artifact

- Base: main commit `d968c2ccbafb85a91d1dc4683eee77accfad3142`.
- Firmware: `g2_2.2.9.22_cfw.bin`.
- SHA-256: `fc47649cb1ad2f2418f55b42f2a33e71f552e342e3ff0717305deca5a94294d2`.
- Patch operations: 34; one additional stock call site.
- Rebuild through the pinned JSON; full compiler regeneration and replay verified.
