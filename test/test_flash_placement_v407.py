# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""CH32V407 RM V1.2 p.579 (32.2.1): "Functions related to FLASH erasure and programming may only be placed in the
zero-wait zone of the FLASH" (RM-CN p.512 the same: only in the zero-wait flash). With RAM_CODE_MOD = 0 the zero-wait
zone is 0x08000000-0x0807FFFF (512 KB; RM p.6): the bootloader and the app body, as their zero alias 0x0-0x7FFFF.

Every function of a V407 ELF (app or bootloader) whose code addresses the flash controller (0x40022000: lui 0x40022)
must lie there, not in SRAM. The SRAM tick and fault stubs that run during an operation do not touch the controller.

    python test/test_flash_placement_v407.py <elf> [<elf> ...]
"""
import os
import pathlib
import re
import subprocess
import sys

ZERO_WAIT = (0x00000000, 0x00080000)                # the zero alias of 0x08000000-0x0807FFFF


def objdump():
    base = pathlib.Path(os.environ.get('LOCALAPPDATA', '')) / 'xPacks/xpack-riscv-none-elf-gcc-15.2.0-1/bin'
    return str(base / 'riscv-none-elf-objdump.exe')


def main():
    bad, seen = [], 0
    for elf in sys.argv[1:]:
        dis = subprocess.run([objdump(), '-d', '--no-show-raw-insn', elf], capture_output=True, text=True,
                             check=True, timeout=300).stdout
        fn, fn_addr = None, 0
        hits = {}
        for line in dis.splitlines():
            m = re.match(r'^([0-9a-f]+) <(.*)>:$', line)
            if m:
                fn, fn_addr = m.group(2), int(m.group(1), 16)
                continue
            if fn and re.search(r'\tlui\s+\w+,0x40022\b', line):
                hits[fn] = fn_addr
        for name, addr in sorted(hits.items(), key=lambda kv: kv[1]):
            seen += 1
            where = 'zero-wait flash' if ZERO_WAIT[0] <= addr < ZERO_WAIT[1] else \
                'zero-wait flash (physical)' if 0x08000000 <= addr < 0x08080000 else 'NOT zero-wait flash'
            print(f'{pathlib.Path(elf).parent.name}: {name} at {addr:#010x}: {where}')
            if where == 'NOT zero-wait flash':
                bad.append((elf, name, hex(addr)))
    print(f'flash-controller functions: {seen}, outside the zero-wait flash: {len(bad)}')
    sys.exit(1 if bad or not seen else 0)


if __name__ == '__main__':
    main()
