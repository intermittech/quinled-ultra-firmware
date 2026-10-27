"""SPDX-License-Identifier: GPL-3.0-or-later
Copyright (C) 2026 QuinLED
Written from the CH32V407 reference manual V1.2 (pp.19-23 clocks, p.69 mtvec, pp.73-77 CSRs) and the QingKe V3 manual.

Reset, CSR, trap routing and retained-state checks on the CH32V40x firmware ELFs, as src/mcu/ch32v20x/check_startup.py
does for the V20x:
    python src/mcu/ch32v40x/check_startup.py --app <fw.elf> [--app <fw.elf> ...] [--bl <bl.elf>]
The ordinary rv32emu decoder executes the instructions (its memory map widened to this chip's 512 KB flash and 200 KB
SRAM); this adapter supplies the core CSR semantics and the HSI / PLL / clock-switch responses SystemInit waits for.
It does not model electrical timing, the hardware save stack (HPE) or the vector table-free (VTF) slots' latency.
"""
import argparse
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'tools'))
import rv32emu                                   # noqa: E402

rv32emu.RAM_SIZE = 200 * 1024                    # (module globals: Emu reads them at run time)
rv32emu.FLASH_SIZE = 0x80000
RAM_BASE, RAM_SIZE = rv32emu.RAM_BASE, rv32emu.RAM_SIZE
RCC_CTLR, RCC_CFGR0 = 0x40021000, 0x40021004
PFIC_CFGR = 0xE000E048


class Core(rv32emu.Emu):
    def __init__(self, elf):
        super().__init__(elf)
        self.csr = {0x300: 0, 0x342: 0, 0x341: 0}
        self.csr_writes = []
        self.mret_mpp = None
        self.mmio[RCC_CTLR] = 0x83               # HSION | HSIRDY | HSITRIM 16 (the reset value)

    def rd(self, address, width):
        value = super().rd(address, width)
        if address == RCC_CTLR and value & (1 << 24):
            value |= 1 << 25                     # PLLON -> PLLRDY
        if address == RCC_CFGR0:
            value = (value & ~12) | ((value & 3) << 2)   # SWS follows SW
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
            if ins == 0x30200073:                # mret: resume at mepc, MIE from MPIE, the privilege from MPP
                status = self.csr[0x300]
                self.mret_mpp = (status >> 11) & 3
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
    for _ in range(400000):
        if pc == core.addr('main'):
            break
        pc = core.step(pc, registers)
    else:
        raise AssertionError((elf, 'reset did not reach main'))
    assert registers[2] == core.addr('_eusrstack'), 'stack pointer'
    assert registers[3] == core.addr('__global_pointer$'), 'global pointer'
    assert core.peek(begin, end - begin) == initialized, 'SRAM code and data load'
    bss, bss_end = core.addr('_sbss'), core.addr('_ebss')
    assert core.peek(bss, bss_end - bss) == bytes(bss_end - bss), 'BSS zeroing'
    assert core.peek(RAM_BASE, 32) == b'\xa5' * 32, 'boot handover preserved'
    if app:
        trap = core.addr('trap_entry')
        assert core.peek(core.addr('bcrumb'), 16) == b'\xa5' * 16, 'breadcrumbs preserved'
        assert core.csr_writes == [(0xbc0, 0x01), (0xbc1, 0x01), (0x804, 0x07), (0x300, 0x1888),
                                   (0x305, trap), (0x341, core.addr('main'))], core.csr_writes
        assert core.mret_mpp == 3, 'main runs in machine mode'
        assert core.csr[0x300] & 8, 'interrupts on in main'
        assert core.csr[0x305] & 3 == 0 and trap & 1023 == 0, 'unified trap mode, 1 KB aligned base'
        fault = core.addr('flash_fault')
        assert fault & 1023 == 0 and RAM_BASE <= fault < RAM_BASE + RAM_SIZE, 'flash_fault: 1 KB aligned SRAM base'
        clk = core.mmio
        assert (clk[RCC_CTLR] >> 3) & 31 == 16 - 12, 'the boot trim (MCU_HSI_BOOT_TRIM_STEPS 12 below 16)'
        assert (clk[RCC_CFGR0] >> 17) & 31 == 0b00010 and clk[RCC_CFGR0] & 3 == 2, 'PLL x10, SW = PLL'
    else:
        assert core.csr_writes == [(0xbc0, 0x01)], 'bootloader keeps interrupts masked'
    return core, registers


def wrong_split(elf):
    """RAM_CODE_MOD = 1 (FLASH_OBR bit 8): 136 KB SRAM, not this image's 200 KB: the reset path touches no RAM and
    never reaches main (it waits for the watchdog: the bootloader then writes mode 0)"""
    core = Core(elf)
    core.ram[:] = b'\xa5' * RAM_SIZE
    core.mmio[0x4002201C] = 0x100
    registers, pc = [0] * 32, core.addr('_start')
    for _ in range(20000):
        assert pc != core.addr('main'), 'main reached with RAM_CODE_MOD = 1'
        pc = core.step(pc, registers)
    assert core.ram == bytearray(b'\xa5' * RAM_SIZE), 'RAM touched with RAM_CODE_MOD = 1'
    assert registers[2] == 0 and not core.csr_writes, 'stack or CSRs set up with RAM_CODE_MOD = 1'


def traps(elf):
    routes = {12: 'poll_isr', 51: 'dl_spi1_isr', 53: 'con_uart_isr', 88: 'rc_uart_isr'}
    for cause, target in [(0x80000000 | n, t) for n, t in routes.items()] + [
            (n, 'HardFault_Handler') for n in (2, 4, 6, 11, 0x80000002, 0x80000003, 0x80000067)]:
        core, registers = Core(elf), [0] * 32
        core.csr.update({0x342: cause, 0x341: 0x12345678})
        pc = core.addr('trap_entry')
        for _ in range(40):
            if pc == core.addr(target):
                break
            pc = core.step(pc, registers)
        else:
            raise AssertionError((elf, hex(cause), target))
        if target == 'HardFault_Handler':
            bc = core.addr('bcrumb')
            core.poke(bc, b'\xa5' * 16)
            for _ in range(30):
                pc = core.step(pc, registers)
            assert struct.unpack('<4I', core.peek(bc, 16)) == (0xa5a5a5a5, 0xa5a5a5a5, 0x12345678, cause)
    for stub, target in (('rc_uart_entry', 'rc_uart_isr'), ('flash_tick', 'flash_poll_isr'),
                         ('flash_fault', 'hw_flash_trip')):
        core, registers = Core(elf), [0] * 32
        pc = core.addr(stub)
        for _ in range(4):
            if pc == core.addr(target):
                break
            pc = core.step(pc, registers)
        else:
            raise AssertionError((elf, stub, target))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--app', type=Path, action='append', default=[])
    parser.add_argument('--bl', type=Path, action='append', default=[])
    args = parser.parse_args()
    for elf in args.app:
        reset(elf, True)
        traps(elf)
        wrong_split(elf)
        print(f'{elf}: reset/data/BSS/retained state/CSR/clock/trap/memory-split checks PASS')
    for elf in args.bl:
        core, _ = reset(elf, False)
        note = ' (hw_soft_reset inlined: not called)'
        if 'hw_soft_reset' in core.syms:
            core.writes.clear()
            try:
                core.call('hw_soft_reset', max_steps=50)
            except RuntimeError as exc:
                assert 'step limit' in str(exc), str(exc)
            assert (PFIC_CFGR, 0xbeef0080) in core.writes, 'software reset command'
            note = ', software reset'
        print(f'{elf}: reset/data/BSS/retained state/CSR checks PASS{note}')


if __name__ == '__main__':
    main()
