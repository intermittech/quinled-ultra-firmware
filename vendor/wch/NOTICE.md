# WCH vendor files

From the official WCH SDK, https://github.com/openwch/ch32v20x (EVT/EXAM/SRC, fetched 2026-10-11). That repository
states no license. The files keep WCH's copyright and notice and are not covered by this project's GPL:
"This software (modified or not) and binary are used for microcontroller manufactured by Nanjing Qinheng
Microelectronics."

| File | |
|---|---|
| `ch32v20x.h` | device register definitions (V1.0.1, 2025/11/04), unmodified |
| `core_riscv.h` | QingKe core / PFIC helpers, unmodified |
| `startup_ch32v20x_D6.S` | startup and vector table for CH32V20x_D6 (the CH32V203F8 is a D6 part), unmodified |
| `Link.ld` | WCH's linker script, **modified** by this project: the CH32V203F8 section is active with the flash shortened to 0xF600 (the top holds the calibration, the power-on scene log and the settings; `src/hal/board.h`), and the sections moved to `Link_sections.ld`, which it includes |
| `Link_sections.ld` | the SECTIONS part of WCH's `Link.ld`, unchanged except for a header comment |
| `Link_app.ld` | **this project's** (GPL-3.0-or-later): the app's memory map behind the 8 KB bootloader (0x2100, RAM after the 32-byte hand-over block), sharing `Link_sections.ld` |
