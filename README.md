# QuinLED dig2analog-ultra firmware

Firmware for the **QuinLED dig2analog-ultra-5ch**: a 5-channel LED driver with its own microcontroller (WCH
CH32V203) in the QuinLED dig2analog+ form factor. On the data line it works in one of two ways:

- **WS mode:** it behaves like a WS2915 (16-bit) or WS2805 (8-bit) chip, so WLED, ESPHome's `ws2915:` or any
  controller for these chips drives it. Auto-detected, with high-frequency PWM and 16-bit dithering on top.
- **Ultra mode:** together with ESPHome, the board becomes a co-processor. ESPHome sends what should happen, and the
  board computes transitions, gamma and effects itself, in 16 bit every millisecond, in step with every other board.

| Board | Firmware | |
|---|---|---|
| QuinLED dig2analog-ultra-5ch (production revision, PB11 wired) | release images `-hw1` | In development |
| QuinLED dig2analog-ultra-5ch with input current sense | `-hw1` (current sense unused); an `-isense` image follows | Next board revision |

## Getting the firmware onto a board

- **Production boards** come with the bootloader and a release installed.
- **Updates go over the data line** from ESPHome: an `update` entity per data line shows new releases in Home
  Assistant and installs them on every board of the line. See [Updating over the data line](https://github.com/intermittech/quinled-ultra-firmware/wiki/Updating-Over-The-Data-Line).
- **Development:** build it yourself and flash it over SWD with a WCH-LinkE. See [Building](https://github.com/intermittech/quinled-ultra-firmware/wiki/Building)
  and [Flashing over SWD](https://github.com/intermittech/quinled-ultra-firmware/wiki/Flashing-Over-SWD).

## Using it

- **With ESPHome:** Ultra mode with the [`quinled-ultra-esphome`](https://github.com/intermittech/quinled-ultra-esphome) components (`dig2analog_ultra`,
  `dig2analog_ultra_strip`; ready-made configurations in its `examples/`), WS mode with the `ws2915` component of
  [`esphome-ws2915-ws2805`](https://github.com/intermittech/esphome-ws2915-ws2805).
- **With WLED or another WS2805 / WS2915 controller:** see [WS modes and config frames](https://github.com/intermittech/quinled-ultra-firmware/wiki/WS-Modes-And-Config-Frames).
- More examples are in [`examples/`](examples).

## Releases

Each [release](https://github.com/intermittech/quinled-ultra-firmware/releases) holds:

- the update image (what ESPHome installs);
- a factory hex (bootloader + firmware) for production.

ESPHome offers three channels:

- **stable:** releases;
- **beta:** release candidates and betas;
- **alpha:** early test builds.

Every channel is signed, and ESPHome checks the signature before it installs anything.

## Documentation

See the **[wiki](https://github.com/intermittech/quinled-ultra-firmware/wiki)**:

| | |
|---|---|
| [Home](https://github.com/intermittech/quinled-ultra-firmware/wiki) | What the firmware does, supported boards |
| [Updating over the data line](https://github.com/intermittech/quinled-ultra-firmware/wiki/Updating-Over-The-Data-Line) | The ESPHome update entity, channels, blink codes, recovery |
| [WS modes and config frames](https://github.com/intermittech/quinled-ultra-firmware/wiki/WS-Modes-And-Config-Frames) | WS2805 / WS2915 emulation, board settings from the data line, WLED |
| [Console](https://github.com/intermittech/quinled-ultra-firmware/wiki/Console) | The UART console: status, settings |
| [Ultra protocol](https://github.com/intermittech/quinled-ultra-firmware/wiki/Ultra-Protocol) | Summary; the full specification is in [quinled-ultra-protocol](https://github.com/intermittech/quinled-ultra-protocol) |
| [Current sense](https://github.com/intermittech/quinled-ultra-firmware/wiki/Current-Sense) | Board current limit, short protection, calibration (next board revision); local on the board, not reported back |
| [Building](https://github.com/intermittech/quinled-ultra-firmware/wiki/Building) · [Flashing over SWD](https://github.com/intermittech/quinled-ultra-firmware/wiki/Flashing-Over-SWD) | For developers |
| [Development and tests](https://github.com/intermittech/quinled-ultra-firmware/wiki/Development-And-Tests) · [Bench plan](https://github.com/intermittech/quinled-ultra-firmware/wiki/Bench-Plan) | Layout, host tests, measurements, what still needs hardware |
| [Releases and signing](https://github.com/intermittech/quinled-ultra-firmware/wiki/Releases-And-Signing) | For maintainers |

## Status

Functionally complete and host-tested, not yet run on hardware: see the [bench plan](https://github.com/intermittech/quinled-ultra-firmware/wiki/Bench-Plan).

## License

Copyright (C) 2026 QuinLED. GPL-3.0-or-later, see [LICENSE](LICENSE). The files in `vendor/wch/` are WCH's and keep WCH's terms; see
[NOTICE.md](NOTICE.md).

"QuinLED" is a trademark of its owner. You may build on this firmware under the GPL, but a derived product may not
be called QuinLED or presented as one. Keep the credit to the original project.
