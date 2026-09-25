# Keyboard donor ABI review

This feature is pinned to `g2_2.3.0.24.bin`, SHA-256
`187ccf2bcc5c17a212106e8a376745511e8289c4232b634a7ea94b9bf25a0979`.
Runtime code address minus `0x379bf5` is the donor file offset. Callable addresses
in C have bit zero set for Thumb. The tables below use even code addresses.

These are static disassembly findings, not a hardware verification. Byte ranges
are pinned in `patches/stock_abi_230.json` with labels beginning `keyboard:`.
`audit_stock.py` checks the donor hash, every guard, the executable source address
inventory, and the ordered live patch footprint. Do not infer these addresses
for another firmware version.

## Live writes

| Address | Original bytes / meaning | Replacement |
| --- | --- | --- |
| `0x4caa60` | `0ef009fa`, BL WsfMsgAlloc in DM callback; r6 = original event | Allocation wrapper; on failure process only owned keyboard events on WSF |
| `0x4caab0` | `0ef0e1f9`, BL WsfMsgAlloc in ATT callback; r5 = original event | Allocation wrapper; count lost keyboard callbacks as gaps |
| `0x60093e` | `0a681279`, load pSmpCfg and its I/O byte; r5 = SMP CCB | Return KeyboardDisplay only for the owned keyboard |
| `0x601298` | same bytes/register ABI in Pairing Response builder | Same per-connection I/O wrapper |
| `0x4cc1ec` | `38b584b0`, push r3-r5/LR; sub SP,16 | Dispatcher trampoline, stock continuation `0x4cc1f0` |

Exactly five four-byte live writes are added. The I/O wrapper preserves every
register except the intended r2 result, and preserves APSR NZCVQ. Its six-word
push keeps the stack aligned. The dispatcher preserves its two incoming arguments
and LR, reproduces the displaced prologue on pass-through, and returns directly
only for handled keyboard events. Neither path allocates before a private control
has created the keyboard context. No controller initialization, capacity constant,
ring singleton or shared SMP configuration is rewritten.

## Event ownership and layouts

`_bleCommHandler` (`0x4cc1ec`) consumes application WSF messages. It sends ATT
events 2..24 to AppDisc/AppServer and central DM events to
`AppMasterProcDmMsg` (`0x51d0c2`) / `AppMasterSecProcDmMsg` (`0x51d162`), then to
Even's common, slave and master/ring handlers. Keyboard interception occurs here,
after the lower ATT/SMP clients have processed the stack event. Owned ATT events
skip ring discovery. Owned DM events retain the generic master/security handlers
but skip Even's ring routing. Global ECC and unrelated connections pass through.

The registered DM callback at `0x4caa42` sizes and copies the event; advertisements
append `len` bytes from `pData`. The ATT callback at `0x4caaa6` copies exactly
16 header bytes followed by `valueLen` bytes, fixing the data pointer. Both post
to handler byte `0x20073e64 + 0x56`, initialized at `0x4cc39c`.
WSF clients are ATT=0, SMP=1, application=3. The DM allocation fallback runs on
the same BLE task and calls only the bounded keyboard path; its security and ATT
commands post further messages rather than sending phone protobufs inline.

| Structure | Donor layout |
| --- | --- |
| WSF header | param:u16 @0, event:u8 @2, status:u8 @3 |
| ATT callback | header, value pointer @4, length:u16 @8, handle:u16 @10, continuing:u8 @12, MTU:u16 @14 |
| Legacy advertisement | header, data pointer @4, length @8, RSSI @9, event type @10, address type @11, address[6] @12 |
| DM auth request | header, OOB @4, display @5 |
| DM comparison | header, 16-byte confirm value @4 |

## Stack and database entry points

| Address | Evidence / use |
| --- | --- |
| `0x4d8e76`, `0x4d8e92` | WsfMsgAlloc / WsfMsgSend; called by stock ATT/DM wrappers |
| `0x579c12` | DmScanStart(phys, mode, scanTypes, filterDuplicates, duration, period); 14-byte message builder |
| `0x200769b4 + 0x18` | Scan state, set to zero by `0x579bd4` initialization |
| `0x4cd2b8` | Connection allocator: three records at `0x20073c30`, stride 0x30; in-use @0x16, HCI handle @0x0c, connId @0x10 |
| `0x51d248` | AppMasterConnOpen(initType, addrType, addr, dbHandle), calls DmConnOpen(client=3); per-conn app table `0x20073908`, stride 0x44 |
| `0x4ce3f4` | DmConnClose(client, connId, reason), 36-byte asynchronous message |
| `0x4843a8` | AppDbFindByAddr; stock app-open uses the same lookup |
| `0x483c2c` | Counts allocated bonds by role: ten records, stride 0xc8, allocated @0x2f, role @0xc3 |
| `0x483cc0` | New-record path evicts at five records for a role; keyboard checks before entering it |
| `0x483db6` | Searches ten slots and marks allocation immediately; no free slot returns null |
| `0x51c7e4` | Start per-connection security: existing LTK -> encryption, otherwise bond allocation/pair request; context byte 8 marks in-flight |
| `0x4e6684` | DmSecAuthRsp(conn, length, data); copies the three passkey bytes |
| `0x550238` | DmSecCompareRsp(conn, valid) |
| `0x550270` | Comparison display number = big-endian confirm bytes 12..15 modulo 1,000,000 |

The full 24-byte SMP configuration at `0x7aab68` has I/O capability 3
(NoInputNoOutput) at +4. Its pointer is `0x200004e8`. Pairing Request builder
`0x600900` stores its seven-byte PDU in SMP CCB+0x20; Pairing Response builder
`0x601286` uses CCB+0x27. Both load the I/O capability using the exact two
instructions patched above. Both keep their CCB in r5, with connection ID at
+0x3d. The replacement changes only that connection's on-wire/saved I/O field to
4 (KeyboardDisplay), leaving key lengths, authentication flags and the shared
configuration intact. The phone implements the advertised passkey/comparison UI.

The application security configuration at `0x7c5184` is `09 00 01 00 00 00`:
stock auth/key distribution/OOB/initiation policy is retained. This does not
promise authenticated pairing for a peer that selects Just Works. Bond updates
still use the stock persistence path. There is no custom NVM format or erase.

## ATT request building

The firmware implements one-PDU Find Information, Read, Read Blob, Write Request
and Write Command builders using `attMsgAlloc` (`0x4c9fcc`) and `attcSendMsg`
(`0x4ca560`). Stock examples at `0x4ca63e`, `0x4ca67a`, `0x4ca6bc` and `0x556c42`
confirm the packet parameter layout and callback numbers.

The allocated block begins with length:u16; range/offset parameters begin at +2;
the ATT PDU begins at +8. Range requests leave start/end serialization to Cordio.
Read Blob stores its offset at +2, handle at PDU+1. The request table at
`0x729f34`, loaded by `0x54d292`, has a non-null entry 6 pointing to
`0x54cf8c`. That sender explicitly handles event 6 by serializing the offset
from the saved parameter, proving that long reads remain linked even though the
public `AttcReadLongReq` wrapper was not found in this donor. All requests use
continuing=false; the phone performs pagination with a new request ID each time.

`0x4ca560` validates connection/MTU, posts the request, and frees rejected packet
buffers. Request-buffer allocation failure is reported immediately by the custom
builder; downstream allocation loss is bounded by the transaction timeout.
`AttcIndConfirm` (`0x4ca758`) clears the outstanding indication bit and transmits
opcode 0x1e; Service Changed indications are confirmed even if a relay queue is
full. No stock ATT server table is replaced.

Cordio source was used as a layout cross-check, with donor disassembly taking
precedence: [DM API](https://github.com/ARMmbed/mbed-os/blob/master/connectivity/FEATURE_BLE/libraries/cordio_stack/ble-host/include/dm_api.h),
[ATT client](https://github.com/ARMmbed/mbed-os/blob/master/connectivity/FEATURE_BLE/libraries/cordio_stack/ble-host/sources/stack/att/attc_main.c),
[ATT read builders](https://github.com/ARMmbed/mbed-os/blob/master/connectivity/FEATURE_BLE/libraries/cordio_stack/ble-host/sources/stack/att/attc_read.c),
[SMP types](https://github.com/ARMmbed/mbed-os/blob/master/connectivity/FEATURE_BLE/libraries/cordio_stack/ble-host/include/smp_api.h).

## Unproven hardware assumptions

The three host connection slots do not prove three simultaneous controller links
or satisfactory radio scheduling. Stock scan/connection state is shared and a
ring reconnect during a keyboard scan can contend with it. Firmware build tests
and source simulations cannot prove these controller interactions or the native
iOS lifecycle. Keep the hardware acceptance list in `keyboard.md` open until
measurements exist; do not label the branch as universally compatible.
