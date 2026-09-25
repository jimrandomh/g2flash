# Keyboard ARM emulator verification

Verified on 2026-09-24 with [PaulMcMillan/g2-firmware-emulator](https://github.com/PaulMcMillan/g2-firmware-emulator)
at commit `f30527c0b7e6fa6e9be06e7067e49468197fa258`, using its
`CfwUnicornHarness`, manifest loader, Thumb branch decoder and allocator.
All eight scenarios pass. The machine-readable evidence is
[keyboard-emulator-result.json](keyboard-emulator-result.json).

This executes the exact compiled keyboard instructions from the committed
`cfw_patches.json`, including the production context allocator, atomic operations,
control dispatcher, event queue, timer callback and naked assembly trampolines.
The runner verifies the complete patched image SHA-256, matches fresh compiler
output against the appended bytes, and checks all five installed branch targets.
It also checks the revision and unchanged source of the imported emulator modules.
The image tested is
`4c7bf9a552df1a42f26b3378425c7c2121ca4058fed7283ec863c3f8d2cd13fe`.

## Reproduce

From the g2flash repository, with its normal firmware build prerequisites and
the exact stock `g2_2.3.0.24.bin` already present:

```sh
git clone https://github.com/PaulMcMillan/g2-firmware-emulator /tmp/g2-firmware-emulator
git -C /tmp/g2-firmware-emulator checkout --detach f30527c0b7e6fa6e9be06e7067e49468197fa258
python3 -m venv /tmp/g2-keyboard-emu-venv
/tmp/g2-keyboard-emu-venv/bin/pip install -e /tmp/g2-firmware-emulator 'unicorn==2.1.4'
python3 patches/build.py patches/patches_main.c --json > /tmp/g2-keyboard-emulator-build.json
python3 patches/apply_patches.py g2_2.3.0.24.bin patches/cfw_patches.json /tmp/g2-keyboard-cfw.bin
/tmp/g2-keyboard-emu-venv/bin/python patches/host/keyboard_emulator_test.py \
  --image /tmp/g2-keyboard-cfw.bin \
  --build-json /tmp/g2-keyboard-emulator-build.json \
  --output /tmp/g2-keyboard-emulator-result.json
```

On this macOS host, Unicorn aborted with exit 132 during memory setup inside
the coding sandbox, including for a minimal unrelated ARM program. Running the
same interpreter outside that sandbox succeeded. This is a native emulator/JIT
execution requirement; the runner never opens a physical Bluetooth connection.

## What passed

| Scenario | Observed checks |
| --- | --- |
| Dormant hooks | Stock dispatcher continuation and stack restoration; ATT/DM allocation failure before initialization; no CFW allocation or timers; malformed controls and left-lens controls ignored. |
| Pairing and routing | Synthetic third connection with two occupied CCB slots; passkey and numeric comparison relay; encrypted-only watch setup; phone/ring/global events pass to the stock continuation; watched keyboard report reaches the private phone notification. |
| Pairing capability trampoline | Only owned enabled keyboard gets KeyboardDisplay; phone/ring/disabled links retain stock capability; registers and APSR NZCVQ preserved; shared config unchanged. Both installed SMP sites target this tested trampoline. |
| ATT requests | Exact stock-import arguments and packet layouts for Find Information, Read, Read Blob, Write Request and Write Command; async response correlation; allocation failure reporting. |
| Fragmentation and message loss | Exact reconstruction of a 512-byte input from six fragments; queue overflow consumes a sequence; heartbeat exposes drops; failed ATT callback allocation records loss; failed DM close allocation clears ownership so a reused connId passes through. |
| Timeout and disable | ATT timeout disconnects once; closing links stop relaying input but acknowledge indications; disable plus close drains and parks the timer. |
| Scan and capacity | Busy scanner rejected; exact bounded scan arguments; advertising payload relayed; full central bond table rejects a new peer. |
| Allocation failures | Initial context OOM stays dormant; failed timer creation frees unpublished keyboard state without an invalid free. |

The sender shim additionally rejects any notification emitted outside the timer
callback. Unknown stock calls fail closed. No keyboard firmware changes were
needed for these scenarios. During runner development an expected Find Information
packet contained one excess zero byte; the expectation was corrected to its
13-byte allocation. No captured firmware failure was suppressed.

## Qualification boundary

The upstream full-system Renode runtime is pinned to **2.2.9.22**, whereas this
branch uses **2.3.0.24**. Its canonical stock import addresses cannot be reused.
The local runner uses explicit 2.3.0.24 shims from
[the reviewed ABI inventory](keyboard-abi.md); it does not change the emulator
checkout or silently substitute the older firmware. Stock allocators, RTOS timers,
WSF delivery, Cordio procedures and the notification sender are synthetic host
boundaries. Timer callbacks and WSF messages are scheduled serially.

Consequently this run does **not** verify full-system boot/OTA, the implementation
of stock imports, controller capacity or three actual BLE links, radio timing,
cryptographic pairing, arbitrary physical keyboards, concurrent task races, or
iOS background delivery. The phone and ring pass-through checks terminate at the
stock dispatcher continuation; they do not run the complete ring application.
Counts in the JSON are observed instruction addresses, not branch/source coverage
percentages. The hardware acceptance list in [keyboard.md](keyboard.md) remains
required. A full-system emulator qualification needs a separately reviewed
2.3.0.24 runtime port and a synthetic HOGP keyboard peripheral.
