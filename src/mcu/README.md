# MCU family interface

`profile.h` supplies integer compile-time rates and conversions without device
register headers. `device.h` selects the device header; `target.h` combines board
facts, the MCU profile, a registered binding and the record layout. The build puts the
selected family's folder first on the include path (`TARGET_HAL`), so `"hw.h"` and its
siblings resolve to that family. New board descriptions contain no device registers.

Each family must provide:
- `clock.h`: HCLK, PCLK1/2, timer and ADC rates in Hz; a 20 kHz / 50 us poll tick;
  UART baud divisors from the actual peripheral bus; timer conversions from ns
  and 10 ns, with explicit rounding and preserved 32-bit runtime arithmetic.
- `sample.h`: SPI sampling rate in Hz, 16-bit ring-word conversions from ns/us,
  symbol grid, decoder thresholds, ring capacities and audited CPU/scan margins.
  Current conversions are ceiling unless named NEAREST/ROUND; presets keep
  their quantized physical times exactly. Constants must fold at compile time.
- `systick.h`: hardware width, low-word reads and unsigned delta semantics.
  Never compare absolute free-running timestamps across a wrap. App SysTick
  reloads every 50 us; flash clock changes preserve that interval and its phase.
- `irq.h`, `isr.S`, `startup.S`: the audited interrupt ABI, priority/nesting and
  SRAM rules. V20x uses QingKe HPE + mret wrappers. `.ramtext` and everything it
  calls during flash busy must stay in SRAM; startup copies it with `.data`.
- `hw*.c/h`, `dma_map.h`, `bindings.h`: timers, DMA, ADC, GPIO, UART, pin-mode
  translation, safe reset ordering, delay-line primitives and existing HAL API.
  Hardware capabilities/routes belong in bindings, electrical limits in boards.
- `flash.h`, `layout.h`, `link_mem*.ld`: controller geometry in bytes, clock policy,
  unlock/program/erase/verify rules, records, memory bounds and flash-safe ISRs.
  V20x single-halfword programming additionally uses the SRAM 50 us tick for
  the declared operation deadline; other app writes retain their poll guard.
  Cold flash/boot functions stay in `hw_cold.c` with its save/restore build flags;
  changing translation-unit/function order can perturb LTO and image identity.
- `bl_hw.c`: bootloader controller operations. Its existing APB2 clock differs
  from the application's APB2 clock; use each peripheral's real rate.
- `target.json` boot API layout: protected slot size, physical/link aliases,
  fixed table offset and callable range; generated `boot_api_layout.h/.ld`.
  Prove the [QBA1 calling contract](../../docs/boot-api.md), including gp,
  reentrancy, live app interrupts, stack and flash execution for that family.

A future `ch32v40x` family must meet this contract with its own registered bindings,
startup/link memory and instruction/flash audits. Its requested operating point
is 200 MHz, no crystal by default, and no CPU downclocking. Implement HSI rate-lock
to the hub's host-us timestamps for UART/answer timing; retain the unpopulated
crystal option for bench qualification. A 32-bit SysTick needs wrap-safe deltas.
Sampling grid, LUT/copy geometry, scan quotas and SRAM capacity must be certified
for that family before enabling a binding. The current encoder uses an unrolled
48-sample kernel; replacing that kernel is MCU sampling work, not board work.
No V40x implementation or timing certificate is supplied here.

CONFIG 2 carries 10 ns units; the MCU converts to actual timer ticks and back to the applied physical value.
FIFO depths, buffer sizes and measured late-scan limits are discrete constraints,
not physical durations. Flash loop guards need measured iteration bounds before
any later deadline conversion. HSI calibration and ISR BARRIER contracts remain
unchanged; flash writes remain restricted to the existing dark policy.
