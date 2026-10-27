# MCU support provenance

The register interface, core helpers, startup and linker scripts in this directory
are project-authored firmware support, licensed GPL-3.0-or-later.
Hardware facts come from the CH32FV2x/V3x reference manual V2.5 and CH32V203
datasheet V3.0. Core interrupt/CSR facts are described by the QingKe V4 processor
manual referenced by the reference manual. The firmware's own HAL, interrupt
entry, bootloader, target registry and update/record definitions specify the
required interface and memory layout. No SDK implementation is an authoring
source for the replacement files.

## Replacement inventory

The starting tree is commit `4db97688cf2594b0fe2e110b3255fbc6cc3131ec`.
The complete vendor directory and its MCU-layer dependencies were inventoried
before replacement, including files without attribution headers:

| Starting file(s) | Starting provenance | Replacement location |
|---|---|---|
| `vendor/wch/ch32v20x.h` | WCH-attributed SDK register header | `ch32v20x.h` |
| `vendor/wch/core_riscv.h` | WCH-attributed SDK core helpers | `core_riscv.h` |
| `vendor/wch/startup_ch32v20x_D6.S` | WCH-attributed startup with project changes | `startup.S` |
| `vendor/wch/Link.ld`, `Link_lower.ld` | Headerless inherited linker scripts | `Link.ld`; former bench variant removed |
| `vendor/wch/Link_app.ld`, `Link_app_lower.ld` | Headerless app layout using inherited sections | `Link_app.ld`; former bench variant removed |
| `vendor/wch/Link_sections.ld` | Headerless inherited sections with project changes | `Link_sections.ld` |
| `vendor/wch/text_order.ld` | Headerless project-generated ordering placeholder | `text_order.ld` |
| `vendor/wch/NOTICE.md` | Project documentation describing SDK provenance | Root `NOTICE.md` |
| `Link.ld`, `Link_lower.ld`, `Link_app.ld`, `Link_app_lower.ld` | Headerless copies of inherited scripts | Independently written `Link.ld` and `Link_app.ld`; former bench variants removed |
| `link_mem.ld`, `link_mem_lower.ld`, `link_mem_app.ld`, `link_mem_app_lower.ld` | Headerless inherited memory-region excerpts | Independently written `link_mem.ld` and `link_mem_app.ld`; former bench variants removed |
| `startup.S` | Wrapper including the vendor startup | Independently written startup at this path |

`system_ch32v20x.h` and `ch32v20x_conf.h` were minimal project stubs without
copyright headers; their project authorship and licence are now explicit.
The HAL and `bootloader/bl_start.S` are project implementations, not SDK copies.
The bootloader's linker and startup remain separate project-owned implementations.

The public source tree ships no WCH SDK files. Compiler selection and the MCU's
documented instruction set do not change the provenance of these support files.

`check_startup.py` runs the actual application and bootloader reset instructions
in the project's rv32emu decoder, with CSR semantics and PLL-ready responses.
It checks initialized RAM, BSS, retained state, gp/sp, trap dispatch and the reset
command. An optional reference-build folder also checks binary and reset-state
identity; this is a software model, not electrical or hardware-stack qualification.
