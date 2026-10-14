# The compilers

| Build | Compiler | How the build finds it |
|---|---|---|
| The app (both boards), `XW=1`: the default since 0.6.7 | **WCH GCC 15.2** with WCH's XW extension (`-march=rv32imc_zicsr_zifencei_xw`) | `WCH_GCC_DIR=<its folder>` (the one with `bin/riscv32-wch-elf-gcc`) |
| The app, `XW=0` (the comparison and the fallback) | xPack GNU RISC-V Embedded GCC 15.2.0-1 | `RISCV_GCC`, else `%LOCALAPPDATA%\xPacks\xpack-riscv-none-elf-gcc-*` |
| The bootloader (frozen at the first factory hex) | xPack GCC 15.2.0-1, always | as `XW=0` |

`tools/build_target.sh` checks the WCH compiler before each `XW=1` build (`check_wch_gcc.sh`: its version line and the
SHA-256 of the files a build uses, `wch-gcc15-win64.sha256`) and stops if it differs.

## XW

The QingKe V4B core of the CH32V203 has 16-bit forms of the byte and halfword loads and stores that RV32C lacks
(`c.lbu`, `c.lhu`, `c.sb`, `c.sh` and the sp-relative `c.lbusp`, `c.lhusp`, `c.sbsp`, `c.shsp`; QingKe V4 manual:
"extended instructions"). WCH's GCC emits them with `_xw` in `-march`; the app gets ~1.6 KB smaller (0.6.7: 55,812 ->
54,196 B before the other 0.6.7 changes). Without `_xw` WCH's GCC 15.2 gives byte-identical output to xPack's 15.2.

- **Tools:** no objdump disassembles XW (the listings show `.insn 2, 0x....`); `tools/rv32emu.py` executes it (its
  `xw_decode`), so `measure.py`, `hal_check.py`, `flash_stores.py`, `mainloop.py` and the cycle caps run XW images.
  `stack.py` and `hal_check.py`'s SRAM check read only sp adjustments, calls and branches, which XW doesn't change.
- **Bench:** an XW image must boot and pass the acceptance on real silicon (test/ROUND4_BENCH_PLAN.md).
- **Sources:** WCH has not published its GCC changes; LLVM has the same instructions upstream (`Xwchc`).

## Getting WCH GCC 15.2

**Windows (how the releases are built):** MounRiver Studio 2.5.0's installer (`MounRiver_Studio_Setup_V2.5.0.zip`
from mounriver.com, ~511 MB) carries the toolchain as a 7z:
`resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC15.7z` (52,550,041 bytes, SHA-256
`dc3a86f33354fde834d8bc51c91b4e60477efda3236c9f91411b67bdeb5f9450`). Either install MounRiver Studio, or open the
installer with 7-Zip (it is an archive) and extract only that 7z. Then:

```
7z x "RISC-V Embedded GCC15.7z" -o<folder>        (its files have no top folder: bin/, lib/, ... land in <folder>)
bash tools/toolchain/check_wch_gcc.sh <folder> "RISC-V Embedded GCC15.7z"
export WCH_GCC_DIR=<folder>
```

**Linux:** MounRiver also publishes a standalone Linux toolchain bundle (`MRS_Toolchain_Linux_X64_V240.tar.xz` and
later, ~400 MB, with a `RISC-V Embedded GCC15` folder: the same GCC 15.2.0, `riscv32-wch-elf-`). It is a different
binary build of the same compiler: before a Linux machine builds a release, record its SHA-256 in a
`wch-gcc15-linux64.sha256` next to this file (and teach `check_wch_gcc.sh` to pick it), and check that its `XW=1`
images are byte-identical to the Windows build's for the same tree (build both, compare the `.bin`s).

**CI** (`.github/workflows/release.yml`, a Windows runner like the development PC): MounRiver's download links expire
and the installer is too large to fetch per run, so the workflow takes the 7z from a URL the repository is given at
setup (the `WCH_GCC15_URL` variable: a copy kept where the project controls it), checks its SHA-256 against
`wch-gcc15-win64.sha256`, extracts it and checks the files. Without the variable the release job stops before
building. (WCH's toolchain is GPL software whose patched sources WCH hasn't published: keep the copy private to the
build rather than redistributing it.)
