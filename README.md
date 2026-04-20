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

- **Big yellow number** — estimated distance in km, or `OVERHEAD` / `>40`.
- **Arc gauge** — last-strike energy (0-21 bit AS3935 word) mapped 0-100%.
- **Radar sweep** — decorative, not a bearing indicator (single-antenna
  detector: no direction-finding).
- **Strikes / Energy** — running count and last-strike energy.
- **Energy history** — 20-slot ring buffer bar chart.

## License

Same spirit as the Instructables source — hobby / educational.
