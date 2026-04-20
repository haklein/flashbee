# Flash Bee

Handheld lightning sensing and ranging device — a PlatformIO port of
[biemster's original Instructables build](https://www.instructables.com/Flash-Bee-Handheld-Lighting-Sensing-and-Ranging-De/).

Pairs a **Seeed Studio XIAO ESP32-C3** with a **Seeed Studio Round Display**
(1.28", GC9A01, 240×240) and the **Grove Lightning Sensor (AS3935)** to show
strike distance, a radar sweep, per-strike energy, and a short history of the
last strikes.

## Credits

- Original project, mechanical design, and photos:
  <https://www.instructables.com/Flash-Bee-Handheld-Lighting-Sensing-and-Ranging-De/>
- AS3935 sensor: [Seeed Grove Lightning Sensor AS3935](https://www.seeedstudio.com/Grove-Lightning-Sensor-AS3935-p-5603.html)
- Display stack:
  [Seeed_GFX](https://github.com/Seeed-Studio/Seeed_GFX) (TFT_eSPI fork) and
  [Seeed_Arduino_RoundDisplay](https://github.com/Seeed-Studio/Seeed_Arduino_RoundDisplay)

## Hardware

| Part                                      | Role                                 |
|-------------------------------------------|--------------------------------------|
| Seeed XIAO ESP32-C3                       | MCU                                  |
| Seeed Round Display (GC9A01, 240×240)     | UI (SPI)                             |
| Seeed Grove AS3935                        | Lightning detection (I²C, addr `0x03`) |

I²C pins (ESP32-C3): `SDA = D4 (GPIO6)`, `SCL = D5 (GPIO7)`. SPI for the
display is configured via Setup501 in Seeed_GFX.

### Extra wire: AS3935 `INT` → XIAO `D2`

The Instructables assembly already asks you to cut the Grove connector
off the module and solder direct bridges for 3V3, GND, SDA and SCL.
**Add one more wire:** from the AS3935 module's `INT` pad to XIAO
**D2 (GPIO4)**. This lets the firmware measure the antenna's LC-tank
resonance and pick the correct `TUN_CAP` value automatically — without
it, every distance reading is biased. Override the pin with
`-DAS3935_INT_PIN=<n>` if D2 isn't convenient for your wiring.

> **Safety disclaimer.** This is a hobby device. Do **not** use it as your
> sole basis for deciding whether it is safe to be outdoors. The AS3935 is
> a statistical single-antenna detector with ±1 bin distance uncertainty
> and well-known disturber/false-positive behavior. When in doubt, follow
> the [NWS 30/30 rule](https://www.weather.gov/safety/lightning) and
> official weather advisories.

## Build

Requires [PlatformIO Core](https://platformio.org/install/cli).

```bash
pio run -e seeed_xiao_esp32c3            # compile
pio run -e seeed_xiao_esp32c3 -t upload  # flash via USB-C
pio device monitor -b 115200             # serial log
```

The first build fetches `Seeed_GFX` and `Seeed_Arduino_RoundDisplay` from
GitHub into `.pio/libdeps/`.

### Why `-DBOARD_SCREEN_COMBO=501`?

Seeed_GFX's `Dynamic_Setup.h` selects the pinout/driver via the
`BOARD_SCREEN_COMBO` macro. The canonical Arduino-IDE workflow drops a
`driver.h` into the sketch folder, which Seeed_GFX picks up with
`__has_include("driver.h")`. Under PlatformIO the project's `include/`
directory is **not** on the include path when library source files
(inside `Seeed_GFX/`) are compiled, so that `__has_include` silently
returns false and the library falls through to combo `666` — a CI stub
with the wrong pinout for the round display. The build still succeeds
but the display stays dark on hardware. Defining the macro via
`build_flags` makes it visible to every compilation unit uniformly.

## Layout

```
platformio.ini          # board, libs, build_flags
src/main.ino            # single-file firmware
```

## What it shows

- **Big yellow number** — estimated distance in km, or `OVERHEAD` / `>40` /
  `-- distance unknown`.
- **Arc gauge** — last-strike energy (0-21 bit AS3935 word) mapped 0-100%.
- **Radar sweep** — decorative, not a bearing indicator (single-antenna
  detector: no direction-finding).
- **Strikes / Energy** — running count and last-strike energy.
- **Energy history** — 20-slot ring buffer bar chart. Labelled `(stale)`
  once the last strike is more than 5 min old.
- **Status line under the title** — `OUT/IN WD:n SR:n` (current AFE mode +
  filter levels). Turns amber when filters have tightened beyond
  mid-range, red `ENV TOO NOISY` when the noise floor has hit the
  hardware ceiling and the chip is no longer operating within its
  acceptable input-noise range.
- **`SENSOR LOST` overlay** — shown when I²C has failed 8+ times in a
  row; the firmware keeps retrying `initAS3935()` every 3 s until
  the sensor comes back.

## First-time setup — antenna calibration

After the first flash, open the serial monitor before plugging in (or
hit the reset button after plugging in) and watch for:

```
[tune] no saved antenna calibration.
[tune] send 'tune' on serial to calibrate now.
```

Type `tune` and hit enter within the 5-second window. The firmware:

1. Issues `PRESET_DEFAULT` + `CALIB_RCO` to the AS3935.
2. Sets `DISP_LCO` bit and `LCO_FDIV = ÷128` — routes the LC-tank
   oscillator to the AS3935 `INT` pin, divided down to ~3906 Hz.
3. Sweeps `TUN_CAP` from 0 to 15, counting rising edges on the
   XIAO GPIO for 200 ms per step.
4. Picks the value closest to 3906 Hz, saves it to NVS (via
   `Preferences`), and continues into normal detector mode with
   that `TUN_CAP` applied.

Serial output looks like:

```
TUN_CAP= 0 -> 3863 Hz (-43)
TUN_CAP= 1 -> 3885 Hz (-21)
TUN_CAP= 2 -> 3903 Hz  (-3)
TUN_CAP= 3 -> 3921 Hz (+15)
...
best: TUN_CAP=2 @ 3903 Hz (dev -3, WITHIN TOL)
```

The screen shows a live progress bar during the sweep and a summary
panel at the end (`TUNED` green, or `OUT OF RANGE` red if nothing on
the sweep lands within ±3.5 %).

**To re-calibrate later:** on any subsequent boot there's a 1.5-second
window right after the AS3935 init screen where `tune` will retrigger
the sweep. Hit reset, type `tune` on serial, done.

**If the sweep fails with `NO SIGNAL` / `check INT wire`:** the
firmware saw zero edges on `AS3935_INT_PIN`. Either the jumper from
the AS3935 module's `INT` pad isn't connected, or it's on a different
GPIO than D2. Without that wire the tune cannot run and the firmware
falls back to the compile-time default (`AS3935_TUN_CAP`, 0 unless
overridden) — the detector still works, but distance estimates will
be whatever the out-of-the-box LC tank produces.

## Safety-critical fixes vs. the Instructables original

The original sketch was reviewed twice (once by Claude, once by Codex)
against the AS3935 datasheet (rev 1.07 §8.10–§8.11). The import commit
is kept intact at `7707720` so the deltas are traceable. Fix commit
`107898d` addresses the following:

### Accuracy of the distance readout

- **AFE gain byte was encoded wrong.** `AFE_GB` lives in `REG0x00[5:1]`,
  so the 5-bit field value has to be shifted one bit up before being
  written to the register. The original wrote `0b00010010` (0x12) when
  aiming for the indoor encoding `0b10010` — which placed `0b01001` = 9
  into the field, outside the datasheet's valid `{14 outdoor, 18
  indoor}` pair. Fix: use `(0b01110 << 1) = 0x1C` for outdoor (new
  default — this is a handheld) or `(0b10010 << 1) = 0x24` for indoor.
  Override at build time with `-DAS3935_AFE_GB=AFE_GB_INDOOR`.
- **`PRESET_DEFAULT` + `CALIB_RCO` never ran.** The original skipped
  both direct commands, so the internal RCO timebase used to measure
  strike-pulse energy was uncalibrated and distance estimates were
  therefore unspecified. `initAS3935()` now issues both on every init
  and verifies `TRCO_CALIB_DONE` / `SRCO_CALIB_DONE` (bit 7) plus the
  corresponding `_NOK` bit (bit 6) in regs 0x3A/0x3B before proceeding.
- **Antenna `TUN_CAP` was never set.** Reg 0x08 [3:0] controls the LC
  tank trim; the factory-tuned value is board-specific. The firmware
  now runs a built-in sweep (see
  [First-time setup — antenna calibration](#first-time-setup--antenna-calibration))
  and stores the result in NVS.
- **Distance value `0x00` was coerced to `OVERHEAD`.** The datasheet
  only defines `0x01` (overhead) and `0x3F` (out of range); `0x00`
  is not a valid distance output. The original displayed the
  scariest-possible reading for an undefined input. Now it shows
  `-- distance unknown`.

### Robustness during a storm

- **Noise-floor ratchet only went up.** `INT_NH` caused `NF` to
  increment (stricter) but nothing ever decremented it, so a minute
  of nearby EMI could permanently deafen the detector for the rest
  of the session. `lowerNoiseFloor()` now decays `NF` by one step
  every 60 s of quiet.
- **High-noise fault was hidden.** Datasheet: `INT_NH` means the
  device cannot operate properly under the current input noise.
  The original treated it as "turn up the filter and keep going".
  When `NF` reaches the hardware maximum and `INT_NH` still fires,
  the UI now shows `ENV TOO NOISY` in red so the user knows the
  readings are no longer trustworthy.
- **No I²C error detection.** All reads blindly returned `0xFF` on
  failure, and `0xFF & 0x0F == 0x0F` didn't match any of the three
  interrupt branches — so a wedged bus caused silent permanent
  deafness. I²C calls now propagate a `bool` result; after 8
  consecutive failures the sensor is marked lost and the `SENSOR
  LOST` overlay appears. `Wire.setTimeOut(50)` prevents bus hangs
  from stalling the MCU during ESD events.
- **No stall watchdog.** If the AS3935 wedged (ESD, brownout),
  nothing noticed. A re-init fires automatically after 10 min of
  zero interrupts of any kind.

### Code clarity

- `increaseSensitivity()` / `decreaseSensitivity()` were named
  *opposite* to what they actually did relative to the datasheet
  (higher `WDTH`/`SREJ` = **less** sensitive per §8.4). Renamed to
  `tightenFilters()` / `loosenFilters()`.
- Every AS3935 register, field, and bit now has a named symbol;
  no magic `0x01`/`0x02`/`0x03` scattered through the logic.
- Reg `0x02` writes now preserve `CL_STAT_EN` / `CL_STAT` in
  bits [7:6]; the original blind full-byte write clobbered them.

### What has *not* been validated

All of the above are datasheet-correctness fixes — none have been
run against a real Grove AS3935 on hardware yet. In particular:

1. The built-in antenna tune has been compile-tested but not run
   against a real module. Expect one round of edge-count scaling
   corrections the first time it runs on hardware (e.g. `attachInterrupt`
   overhead at 3.9 kHz, glitch filtering on the INT line).
2. The reviewer critique stands that a single-antenna detector
   produces **statistical** distance to the *head of the storm*,
   not range to the individual strike that triggered the IRQ.
   Do not treat the displayed number as a ranging result you
   would stake a decision on.

## License

Same spirit as the Instructables source — hobby / educational.
