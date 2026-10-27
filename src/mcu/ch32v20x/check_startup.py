"""SPDX-License-Identifier: GPL-3.0-or-later
Copyright (C) 2026 QuinLED
Written from the CH32FV2x/V3x reference manual V2.5 and QingKe V4 manual V1.5.

Run reset, CSR, trap routing and retained-state checks on real firmware ELFs:
python src/mcu/ch32v20x/check_startup.py build/own-mcu/final [--reference build/own-mcu/baseline]
The ordinary rv32emu decoder executes instructions; this adapter supplies core
CSR semantics and the PLL ready/switch responses needed by SystemInit.
It does not model electrical timing or the hardware interrupt save stack.
"""
import argparse
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))
from rv32emu import Emu, RAM_BASE, RAM_SIZE


class Core(Emu):
    def __init__(self, elf):
        super().__init__(elf)
        self.csr = {0x300: 0, 0x342: 0, 0x341: 0}
        self.csr_writes = []
        self.mmio[0x40021000] = 0x83

    def rd(self, address, width):
        value = super().rd(address, width)
        if address == 0x40021000 and value & (1 << 24):
            value |= 1 << 25
        if address == 0x40021004:
            value = (value & ~12) | ((value & 3) << 2)
        return value

    def wr(self, address, width, value):
        super().wr(address, width, value)
        if self._mem(address, width)[0] is None:
            self.mmio[address] = value & ((1 << (width * 8)) - 1)

    def step(self, pc, registers):
        half = self.rd(pc, 2)
        if half & 3 != 3:
            next_pc, _ = self._c(half, pc, registers)
        else:
            ins = self.rd(pc, 4)
            if ins == 0x30200073:  # mret: resume at mepc and restore MIE
                status = self.csr[0x300]
                self.csr[0x300] = (status & ~0x1888) | ((status >> 4) & 8) | 0x80
                next_pc = self.csr[0x341]
            elif ins & 0x7f == 0x73 and (ins >> 12) & 7:
                number, op = ins >> 20, (ins >> 12) & 7
                source = (ins >> 15) & 31
                operand = source if op & 4 else registers[source]
                old = self.csr.get(number, 0)
                new = {1: operand, 2: old | operand, 3: old & ~operand}[op & 3]
                if op & 3 == 1 or source:
                    self.csr[number] = new & 0xffffffff
                    self.csr_writes.append((number, new & 0xffffffff))
                registers[(ins >> 7) & 31] = old
                next_pc = pc + 4
            else:
                next_pc, _ = self._i(ins, pc, registers)
        registers[0] = 0
        return next_pc


def reset(elf, app):
    core = Core(elf)
    core.ram[:] = b'\xa5' * RAM_SIZE
    begin, end = core.addr('_data_vma'), core.addr('_edata')
    initialized = core.peek(core.addr('_data_lma'), end - begin)
    registers, pc = [0] * 32, core.addr('_start')
    for _ in range(150000):
        if pc == core.addr('main'):
            break
        pc = core.step(pc, registers)
    else:
        raise AssertionError((elf, 'reset did not reach main'))
    assert registers[2] == core.addr('_eusrstack'), 'stack pointer'
    assert registers[3] == core.addr('__global_pointer$'), 'global pointer'
    assert core.peek(begin, end - begin) == initialized, 'RAM code and data load'
    bss, bss_end = core.addr('_sbss'), core.addr('_ebss')
    assert core.peek(bss, bss_end - bss) == bytes(bss_end - bss), 'BSS zeroing'
    assert core.peek(RAM_BASE, 32) == b'\xa5' * 32, 'boot handover preserved'
    if app:
        assert core.peek(core.addr('bcrumb'), 16) == b'\xa5' * 16, 'breadcrumbs preserved'
        assert core.csr_writes == [(0xbc0, 31), (0x804, 3), (0x300, 0x88),
                                   (0x305, core.addr('trap_entry')), (0x341, core.addr('main'))]
        assert core.csr[0x300] == 0x88, 'application interrupt state after mret'
        assert core.csr[0x305] & 3 == 0, 'unified trap mode'
    else:
        assert core.csr_writes == [(0xbc0, 31)], 'bootloader keeps interrupts masked'
    return core, registers


def traps(elf, board):
    routes = {12: 'poll_isr'}
    routes.update({51: 'dl_spi1_isr', 53: 'rc_uart_isr'} if board == 3 else
                  {44: 'slew2_isr', 45: 'slew3_isr', 46: 'gate_isr'})
    if board == 2:
        routes[53] = 'rc_uart_isr'
    for cause, target in [(0x80000000 | n, t) for n, t in routes.items()] + [
            (n, 'HardFault_Handler') for n in (2, 4, 6, 11, 0x80000002)]:
        core, registers = Core(elf), [0] * 32
        core.csr.update({0x342: cause, 0x341: 0x12345678})
        pc = core.addr('trap_entry')
        for _ in range(40):
            if pc == core.addr(target):
                break
            pc = core.step(pc, registers)
        else:
            raise AssertionError((elf, cause, target))
        if target == 'HardFault_Handler':
            bc = core.addr('bcrumb')
            core.poke(bc, b'\xa5' * 16)
            for _ in range(30):
                pc = core.step(pc, registers)
            assert struct.unpack('<4I', core.peek(bc, 16)) == (0xa5a5a5a5, 0xa5a5a5a5, 0x12345678, cause)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('images', type=Path)
    parser.add_argument('--reference', type=Path)
    args = parser.parse_args()
    names = [(f'app{b}-{c}', b, True) for b in (1, 2, 3) for c in ('xw', 'xpack')]
    names += [(f'bl{b}-xpack', b, False) for b in (1, 2, 3)]
    for name, board, app in names:
        filename = 'fw.elf' if app else 'bl.elf'
        elf = args.images / name / filename
        core, registers = reset(elf, app)
        if app:
            traps(elf, board)
        else:
            core.writes.clear()
            try:
                core.call('hw_soft_reset', max_steps=50)
            except RuntimeError as exc:
                assert 'step limit' in str(exc), str(exc)
            assert (0xe000e048, 0xbeef0080) in core.writes, 'software reset command'
        if args.reference:
            ref = args.reference / name / filename
            other, other_registers = reset(ref, app)
            assert core.ram == other.ram and registers == other_registers, 'reset equivalence'
            assert core.csr == other.csr, 'CSR equivalence'
            for symbol in ('_data_lma', '_data_vma', '_edata', '_sbss', '_ebss', '_susrstack', '_eusrstack', '__global_pointer$'):
                assert core.addr(symbol) == other.addr(symbol), symbol
            binary = 'fw.bin' if app else 'bl.bin'
            assert (args.images / name / binary).read_bytes() == (args.reference / name / binary).read_bytes()
        print(f'{name}: reset/data/BSS/retained state/CSR/trap checks PASS')


if __name__ == '__main__':
    main()
