# Legacy 0x045 Stalk NAG Port

This source variant ports only the observed Legacy NAG behavior into the
Waveshare single-CAN firmware.  It does not add AP_CYCLE or an automatic
driving-profile controller.

## Trigger and transmit sequence

1. Decode `DAS_status` (`0x399`): AP state from byte 0 low nibble and Hands
   state from byte 5 bits 2..5.
2. Arm on normal/recovered state and latch exactly one event on the captured
   edge `AP 6 -> 1` with `Hands=3`.
3. Wait for a checksum-valid OEM `STW_ACTN_RQ` (`0x045`) neutral frame.  The
   `0x045` integrity byte (`data[7]`) is CRC8-J1850 over bytes 0..6 — the
   same field format the JITTER procedure verified on the target car — not
   the additive checksum used by `0x370`/`0x3EE`.
4. Clone that frame, preserve byte 0 bit 6, write Resume action `2` into byte
   0 bits 0..5, clear byte 0 bit 7, keep the rolling counter (byte 6 high
   nibble) as-is, and recompute byte 7 CRC8-J1850.
5. On the next valid neutral OEM `0x045` frame, send one explicit neutral
   action `0` release.
6. Re-arm only after `AP=6, Hands<=1` remains clear for 500 ms.  A 10-second
   pulse cooldown prevents a noisy status edge from creating rapid actions.

The existing automatic driving-profile input remains on `0x045` byte 1.  The
NAG echo only changes byte 0 and byte 7, so it does not overwrite that field.

## Fail-closed rules

- No send on short or checksum-invalid `0x045` frames.
- No send while a physical stalk action is present.
- No send while the vehicle OTA guard is active or the abort guard (0x399
  state 8/9 driver override latch) has fired; a pending action is dropped,
  not deferred — the same permit family as the JITTER tick.  The AP
  injection gate is deliberately not applied: the trigger is the AP 6->1
  warning drop, an AP-inactive moment by definition.
- No periodic retry loop.  Press and release each get one transmit attempt.
- All older Legacy `0x370` NAG algorithms are disabled when this runtime path
  is selected, preventing two NAG transmitters from running together.

## Dashboard

On Legacy hardware the port has its own defense-page switch (`0x045 拨杆
NAG`, config `stalk_nag`, NVS `stalk_nag`, default off).  The NAG mode
selector keeps its v1.20.0 meaning — on Legacy any non-zero selection drives
the cadence-aware `0x370` late human replay — and while the stalk switch is
on, the selector yields, so only one NAG transmitter is ever live.  Runtime
status reports the warning edge count, press/release transmit counts,
pending state, template validity, blocking reason, and the outer-gate
(OTA/abort) block counter (`stalkBlockedOuterGate`); while the switch is
active the `builtInNag` block reports the stalk implementation
(`legacy_045_stalk_resume_v1`, `stalkSwitch`).
