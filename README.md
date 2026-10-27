# QuinLED Ultra firmware

Firmware for the QuinLED Ultra boards: LED drivers with their own microcontroller (WCH CH32V203, or CH32V407 on the
Digital-Medium) that speak the [QuinLED Ultra protocol](https://github.com/intermittech/quinled-ultra-protocol)
(protocol 0.3, wire version 3). On the data line a board works in one of two ways:

- **WS mode** (Dig2Analog-Plus-5ch): it behaves like a WS2915 (16-bit) or WS2805 (8-bit) chip, so WLED, ESPHome's
  `ws2915:` or any controller for these chips drives it. Auto-detected, with high-frequency PWM and 16-bit dithering on top.
- **Ultra mode:** together with ESPHome, the board becomes a co-processor. ESPHome sends what should happen, and the
  board computes transitions and gamma itself, in 16 bit every millisecond, in step with every other board.

| Board | Build | |
|---|---|---|
| [QuinLED Dig2Analog-Plus-5ch](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Dig2Analog-Plus-5ch): five analog channels | `BOARD=1` (CH32V203) | Prototypes |
| [QuinLED Diff-Power Ultra Receiver Analog-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Analog-M): five analog channels, return channel | `BOARD=2` (CH32V203) | In development |
| [QuinLED Diff-Power Ultra Receiver Digital-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Digital-M): one pixel output, four segments, local pixel effects, return channel | `BOARD=3` (CH32V203, or CH32V407 with `MCU=ch32v40x`) | In development |

An image for one board type, MCU or revision never installs on another.

## Status

**Firmware 0.7.1 is a source release.** It is host- and emulator-tested (unit and soak suites, the bootloaders and the
update chain on the compiled code, timing and size budgets), but not yet qualified on hardware. No update images or
factory files are published for it yet: release packaging stays off until the hardware qualification is done.

## Getting the firmware onto a board

- **Updates go over the data line** from ESPHome: an `update` entity per data line shows new releases in Home
  Assistant and installs them on the boards of the line. See [Updating over the data line](https://github.com/intermittech/quinled-ultra-firmware/wiki/Updating-Over-The-Data-Line).
- **Development:** build it yourself and flash it over SWD with a WCH-LinkE. See [Building](https://github.com/intermittech/quinled-ultra-firmware/wiki/Building)
  and [Flashing over SWD](https://github.com/intermittech/quinled-ultra-firmware/wiki/Flashing-Over-SWD).

## Using it

Signal loss defaults to hold: a momentary controller drop keeps the last setting,
and returning data resumes normally with no blink.

- **With ESPHome:** Ultra mode with the [`quinled-ultra-esphome`](https://github.com/intermittech/quinled-ultra-esphome) components
  (ready-made configurations in its `examples/`), WS mode with the `ws2915` component of
  [`esphome-ws2915-ws2805`](https://github.com/intermittech/esphome-ws2915-ws2805).
- **With WLED or another WS2805 / WS2915 controller:** see [WS modes and config frames](https://github.com/intermittech/quinled-ultra-firmware/wiki/WS-Modes-And-Config-Frames).
- More examples are in [`examples/`](examples).

## Releases

A [release](https://github.com/intermittech/quinled-ultra-firmware/releases) with images holds the update image (what
ESPHome installs) and a factory hex (bootloader + firmware) for production. ESPHome offers three channels: **stable**
(releases), **beta** (release candidates and betas) and **alpha** (early test builds). Every channel is signed, and
ESPHome checks the signature before it installs anything.

## Documentation

See the **[wiki](https://github.com/intermittech/quinled-ultra-firmware/wiki)**:

| | |
|---|---|
| [Home](https://github.com/intermittech/quinled-ultra-firmware/wiki) | What the firmware does, supported boards |
| [Updating over the data line](https://github.com/intermittech/quinled-ultra-firmware/wiki/Updating-Over-The-Data-Line) | The ESPHome update entity, channels, blink codes, recovery |
| [WS modes and config frames](https://github.com/intermittech/quinled-ultra-firmware/wiki/WS-Modes-And-Config-Frames) | WS2805 / WS2915 emulation, board settings from the data line, WLED |
| [PWM and dithering](https://github.com/intermittech/quinled-ultra-firmware/wiki/PWM-And-Dithering) | The PWM presets, dithering, the dim floor, stagger and spread spectrum |
| [Console](https://github.com/intermittech/quinled-ultra-firmware/wiki/Console) | The output-only UART console and its records ([format](docs/console-format.md)) |
| [Ultra protocol](https://github.com/intermittech/quinled-ultra-firmware/wiki/Ultra-Protocol) | Summary ([in this firmware](ULTRA_PROTOCOL.md)); the full specification is in [quinled-ultra-protocol](https://github.com/intermittech/quinled-ultra-protocol) |
| [Current sense](https://github.com/intermittech/quinled-ultra-firmware/wiki/Current-Sense) | Board current limit, short protection, calibration, the overcurrent signal |
| [Pin map](https://github.com/intermittech/quinled-ultra-firmware/wiki/Pin-Map) | Pins, the voltage channels, hardware ids and revisions |
| [Building](https://github.com/intermittech/quinled-ultra-firmware/wiki/Building) · [Flashing over SWD](https://github.com/intermittech/quinled-ultra-firmware/wiki/Flashing-Over-SWD) | For developers; the boot services: [docs/boot-api.md](docs/boot-api.md), the bootloader: [bootloader/README.md](bootloader/README.md) |
| [Development and tests](https://github.com/intermittech/quinled-ultra-firmware/wiki/Development-And-Tests) | Layout, tests, measurements; the Digital-Medium: [firmware](docs/ur_digital_m-firmware.md), [effects](docs/ur_digital_m-effects.md) |
| [Releases and signing](https://github.com/intermittech/quinled-ultra-firmware/wiki/Releases-And-Signing) | For maintainers |

## License

Copyright (C) 2026 QuinLED. GPL-3.0-or-later, see [LICENSE](LICENSE). Third-party material (effect algorithms and
palettes, with their licences) is listed in [NOTICE.md](NOTICE.md).

"QuinLED" is a trademark of its owner. You may build on this firmware under the GPL, but a derived product may not
be called QuinLED or presented as one. Keep the credit to the original project.
