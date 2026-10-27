# Firmware-owned boards

Each `board.json` owns its product name, allocated board ID, assembly options,
electrical ratings, channels, senses, link abilities and supported MCU bindings.
`src/mcu/<family>/target.json` owns build and memory geometry. No external board
registry is consulted. A new product adds a directory and its allocated ID.

Run `python tools/gen_targets.py` after editing a definition; `--check` verifies
generated files and runs in every build. `--shell --board <id or directory name>
--mcu <family>` selects a supported binding. Pin rows are ordered
`[port, pin, initial_level, mode]` recipes translated by the MCU driver; preserve
safe output initialization before enabling inputs and peripherals. A binding's
`overrides` may replace `calibration_ms` (the descriptor's `cal_ms`) where the MCU
changes the board's calibration length.

Assembly IDs are opaque. [Dig2Analog-Plus-5ch](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Dig2Analog-Plus-5ch) has only 3 A and 5 A channel assemblies:
it always forwards and has no return/relay path. [Diff-Power Ultra Receiver Analog-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Analog-M) and [Diff-Power Ultra Receiver Digital-Medium](https://github.com/intermittech/quinled-ultra-firmware/wiki/Board-Diff-Power-UR-Digital-M)
also describe chain-port fitment. Definitions do not constitute hardware timing
qualification. The application descriptor's profile mask is 15; all four
profiles have application model coverage. Bootloader transactions and physical
qualification remain unfinished.

Board definitions contain firmware conversion constants and qualified link limits.
Complete-board timing requires bench qualification.

Current gain fields express firmware conversion constants.
