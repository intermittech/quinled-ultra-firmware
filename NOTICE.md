# Notices

## This firmware

Copyright (C) 2026 QuinLED

The QuinLED dig2analog-ultra firmware is free software: you can redistribute it and/or modify it under the terms
of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or
(at your option) any later version. It is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
[LICENSE](LICENSE).

Every source file of this project starts with:

```
SPDX-License-Identifier: GPL-3.0-or-later
Copyright (C) 2026 QuinLED
```

The firmware says so itself: its console banner and `status` show
"QuinLED dig2analog-ultra firmware vX.Y.Z, Copyright (C) 2026 QuinLED, GPL-3.0, source: github.com/intermittech/quinled-ultra-firmware".

## Third-party files

`vendor/wch/`: from WCH's CH32V20x SDK, https://github.com/openwch/ch32v20x (EVT/EXAM/SRC, fetched 2026-10-11).
They are not covered by this project's license; they keep WCH's copyright and notice ("This software (modified or
not) and binary are used for microcontroller manufactured by Nanjing Qinheng Microelectronics"). See
`vendor/wch/NOTICE.md`.

| File | |
|---|---|
| `ch32v20x.h`, `core_riscv.h`, `startup_ch32v20x_D6.S` | unmodified |
| `Link.ld`, `Link_sections.ld` | WCH's linker script, modified (flash length) and split in two: the memory map (`Link.ld`) and the unchanged sections (`Link_sections.ld`) |
| `Link_app.ld` | this project's (GPL-3.0-or-later): the app's memory map behind the bootloader |

## Trademark

"QuinLED" is a trademark of its owner, and this license does not grant any right to it. You may build, change and
share this firmware under the GPL, but a derived product may not be called QuinLED or presented as one. Keep the
credit to the original project in the source and documentation.
