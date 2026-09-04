# Citrus Racing — Mac Plus input converter

## What this is

A QMK firmware target that converts an Apple Macintosh Plus M0110A keyboard
and M0100 mouse into a single composite USB HID device, running on one
Adafruit ItsyBitsy 32u4 (5 V / 16 MHz).

## Scope

**Only modify files under `keyboards/converter/m0110_citrus/`.**

Do not edit anything else in this repo. In particular do not edit
`keyboards/converter/m0110_usb/` (the upstream original, kept for reference),
`tmk_core/`, `quantum/`, or `platforms/`. If a change appears to require
touching those, stop and explain why instead of doing it.

Read-only reference material you should consult:

- `keyboards/converter/m0110_usb/` — upstream original
- `keyboards/converter/m0110_citrus/m0110.c` — the keyboard protocol driver.
  This is a local copy compiled into the firmware (see `SRC` in rules.mk), NOT
  upstream tmk_core. It is editable, but do not modify it as part of adding the
  pointing device. Its INSTANT polling and its interrupt-enabled busy-waits are
  load-bearing for mouse responsiveness — see constraints 2 and 3.
- `docs/features/pointing_device.md` — the pointing device API **in this
  checkout**. The custom-driver signatures have changed across QMK versions.
  Read the file; do not assume the API from memory.

## Hardware — these are physical facts, not preferences

The board is already wired. Firmware conforms to the wiring, never the
reverse. **Never reassign a pin to resolve a build error or a bug.**

| Function | ItsyBitsy label | AVR pin |
|---|---|---|
| Keyboard CLOCK | `D2` | PD1 |
| Keyboard DATA | `D3` | PD0 |
| Mouse X1 | `SCK` | PB1 |
| Mouse X2 | `MO` | PB2 |
| Mouse button | `MI` | PB3 |
| Mouse Y1 | `D9` | PB5 |
| Mouse Y2 | `D10` | PB6 |

- MCU is **ATmega32u4 at 16 MHz**. `F_CPU = 16000000` in `rules.mk`.
  Upstream ships `F_CPU = 8000000` for the 3.3 V Feather; that value is wrong
  for this board and will break all M0110 bit timing by exactly 2x.
- Bootloader is `caterina` and processor is `atmega32u4`. Both are already
  set correctly in `keyboard.json` (not `rules.mk`, and not `info.json` —
  this board uses the newer filename). Do not change them.
- PB0 is unused — it is the RX LED.

## Design constraints

1. **Mouse quadrature must use pin-change interrupts on PORTB
   (`PCINT0_vect`).** Not INT0-INT3. Those live only on PD0-PD3 and collide
   with the keyboard's CLOCK and DATA lines. The 32u4's only other external
   interrupt is INT6 on PE6, so four dedicated external interrupts is
   impossible. One ISR reads `PINB` once and decodes both axes from the same
   sample.

2. **Do not change the keyboard driver from INSTANT (0x12) to INQUIRY
   (0x10).** INQUIRY blocks up to 250 ms waiting for a key event, which would
   throttle the pointer to ~4 Hz. INSTANT returns immediately, giving a
   ~6-7 ms scan and a ~150 Hz loop.

3. **Do not add `cli()` around keyboard transactions.** `m0110.c` deliberately
   leaves global interrupts enabled during its busy-wait loops. That is what
   lets the quadrature ISR keep counting through a keyboard transfer.

4. **No pointer acceleration or speed multiplier.** The M0100 is ~100 CPI,
   which is correct for the 512x342 CRT this drives.

5. `report_mouse_t` x/y are 8-bit unless `MOUSE_EXTENDED_REPORT` is enabled.
   Clamp accumulated counts to +/-127.

## Build and verify

```bash
qmk compile -kb converter/m0110_citrus -km default
```

Compile after every change. A change that has not been compiled is not done.

Flashing is manual — do not run `qmk flash`. It requires physically pressing
reset on the board at the right moment.

## Working style

- Make one change at a time and compile between changes.
- If a build fails, fix the actual cause. Do not resolve build errors by
  disabling features, removing code, changing pin assignments, or relaxing
  any constraint listed above.
- If a constraint here appears to be wrong or is blocking progress, say so and
  stop. Do not work around it silently.
- Prefer small diffs. This is embedded code that gets debugged with a scope.
