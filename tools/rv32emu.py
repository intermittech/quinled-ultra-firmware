# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Minimal RV32IMC emulator for counting executed instructions of functions in the firmware ELF.

Not cycle-exact: QingKe V4 executes most instructions in 1 cycle; loads, taken branches/jumps and divisions
take longer. cycles() applies a simple per-class cost model (documented below). Peripheral reads come from an
MMIO dict, writes are recorded.

Usage (library):
    emu = Emu("build/target/fw.elf"); emu.mmio[0x40000024] = 112
    n = emu.call("ws_decode_frame", [buf_addr, nsamples, nbits, tm_addr, out_addr])
"""
import struct

RAM_BASE, RAM_SIZE = 0x20000000, 20 * 1024
FLASH_SIZE = 0x38000                                     # (0.6.5) the user CodeFlash: 64 KB zero-wait + the 160 KB above (the records)
RET_MAGIC = 0x0FFFFFF0

# cost model (cycles), QingKe V4 estimate: 1 for ALU, +1 load, +1 taken branch / jump, div/rem 17 worst
COST = {"alu": 1, "load": 2, "store": 1, "branch": 1, "branch_taken": 2, "jump": 2, "mul": 1, "div": 17}


def sx(v, bits):
    v &= (1 << bits) - 1
    return v - (1 << bits) if v >> (bits - 1) else v


class Emu:
    def __init__(self, elf):
        data = open(elf, "rb").read()
        self.flash = bytearray(FLASH_SIZE)
        self.ram = bytearray(RAM_SIZE)
        self.mmio = {}
        self.writes = []
        self.syms = {}
        e_phoff, = struct.unpack_from("<I", data, 28)
        e_shoff, = struct.unpack_from("<I", data, 32)
        e_phentsize, e_phnum, e_shentsize, e_shnum = struct.unpack_from("<HHHH", data, 42)
        for i in range(e_phnum):
            p_type, p_off, p_vaddr, p_paddr, p_filesz, p_memsz = struct.unpack_from("<IIIIII", data, e_phoff + i * e_phentsize)
            if p_type != 1:
                continue
            seg = data[p_off:p_off + p_filesz]
            if p_paddr < FLASH_SIZE:
                self.flash[p_paddr:p_paddr + p_filesz] = seg
            if RAM_BASE <= p_vaddr < RAM_BASE + RAM_SIZE:      # .data initial values (startup copy)
                o = p_vaddr - RAM_BASE
                self.ram[o:o + p_filesz] = seg
        secs = [struct.unpack_from("<IIIIIIIIII", data, e_shoff + i * e_shentsize) for i in range(e_shnum)]
        for s in secs:
            if s[1] == 2:                                          # SHT_SYMTAB
                strtab = secs[s[6]]
                for k in range(s[5] // 16):
                    st_name, st_value, st_size, st_info, st_other, st_shndx = struct.unpack_from("<IIIBBH", data, s[4] + k * 16)
                    nm = data[strtab[4] + st_name:data.index(b"\0", strtab[4] + st_name)].decode()
                    if nm:
                        self.syms[nm] = (st_value, st_size)
        self.gp = self.syms["__global_pointer$"][0]

    # ---------------------------------------------------------------------------------------------- memory
    def _mem(self, a, n):
        if RAM_BASE <= a < RAM_BASE + RAM_SIZE:
            return self.ram, a - RAM_BASE
        if 0x08000000 <= a < 0x08000000 + FLASH_SIZE:
            return self.flash, a - 0x08000000
        if a < FLASH_SIZE:
            return self.flash, a
        return None, a

    def rd(self, a, n):
        m, o = self._mem(a, n)
        if m is None:
            return self.mmio.get(a, 0) & ((1 << (8 * n)) - 1)
        return int.from_bytes(m[o:o + n], "little")

    def wr(self, a, n, v):
        m, o = self._mem(a, n)
        if m is None:
            self.writes.append((a, v & ((1 << (8 * n)) - 1)))
            return
        m[o:o + n] = (v & ((1 << (8 * n)) - 1)).to_bytes(n, "little")

    def sym(self, name):
        """(address, size); LTO may rename a static to name.lto_priv.N (two files used the same name)"""
        if name not in self.syms:
            k = [n for n in self.syms if n.startswith(name + ".lto_priv.")]
            if len(k) == 1:
                return self.syms[k[0]]
        return self.syms[name]

    def addr(self, name):
        return self.sym(name)[0]

    def poke(self, a, b):
        m, o = self._mem(a, len(b))
        m[o:o + len(b)] = b

    def peek(self, a, n):
        m, o = self._mem(a, n)
        return bytes(m[o:o + n])

    # ---------------------------------------------------------------------------------------------- run
    def call(self, fn, args=(), max_steps=50_000_000):
        x = [0] * 32
        stack_args = max(0, len(args) - 8)
        x[2] = RAM_BASE + RAM_SIZE - max(16, (4 * stack_args + 15) & ~15)
        x[3] = self.gp
        x[1] = RET_MAGIC
        for i, a in enumerate(args):
            if i < 8:
                x[10 + i] = a & 0xFFFFFFFF
            else:
                self.wr(x[2] + 4 * (i - 8), 4, a & 0xFFFFFFFF)
        pc = self.syms[fn][0] if isinstance(fn, str) else fn
        cnt = {k: 0 for k in COST}
        n = 0
        while pc != RET_MAGIC:
            n += 1
            if n > max_steps:
                raise RuntimeError("step limit")
            h = self.rd(pc, 2)
            if h & 3 != 3:
                pc, cls = self._c(h, pc, x)
            else:
                pc, cls = self._i(self.rd(pc, 4), pc, x)
            cnt[cls] += 1
            x[0] = 0
        self.last_counts = cnt
        self.ret = x[10]
        return n

    def cycles(self):
        return sum(COST[k] * v for k, v in self.last_counts.items())

    def _alu(self, f3, f7, a, b, imm=False):
        if f7 == 1 and not imm:                                    # M extension
            sa, sb = sx(a, 32), sx(b, 32)
            if f3 == 0: return (a * b) & 0xFFFFFFFF, "mul"
            if f3 == 1: return ((sa * sb) >> 32) & 0xFFFFFFFF, "mul"
            if f3 == 2: return ((sa * b) >> 32) & 0xFFFFFFFF, "mul"
            if f3 == 3: return ((a * b) >> 32) & 0xFFFFFFFF, "mul"
            if f3 == 4: return (0xFFFFFFFF if b == 0 else int(sa / sb) & 0xFFFFFFFF), "div"
            if f3 == 5: return (0xFFFFFFFF if b == 0 else a // b), "div"
            if f3 == 6: return (a if b == 0 else (sa - sb * int(sa / sb)) & 0xFFFFFFFF), "div"
            return (a if b == 0 else a % b), "div"
        if f3 == 0: return ((a - b) if (f7 == 0x20 and not imm) else (a + b)) & 0xFFFFFFFF, "alu"
        if f3 == 1: return (a << (b & 31)) & 0xFFFFFFFF, "alu"
        if f3 == 2: return int(sx(a, 32) < sx(b, 32)), "alu"
        if f3 == 3: return int(a < b), "alu"
        if f3 == 4: return a ^ b, "alu"
        if f3 == 5:
            if f7 == 0x20: return (sx(a, 32) >> (b & 31)) & 0xFFFFFFFF, "alu"
            return a >> (b & 31), "alu"
        if f3 == 6: return a | b, "alu"
        return a & b, "alu"

    def _load(self, f3, a):
        if f3 == 0: return sx(self.rd(a, 1), 8) & 0xFFFFFFFF
        if f3 == 1: return sx(self.rd(a, 2), 16) & 0xFFFFFFFF
        if f3 == 2: return self.rd(a, 4)
        if f3 == 4: return self.rd(a, 1)
        return self.rd(a, 2)

    def _branch(self, f3, a, b):
        return {0: a == b, 1: a != b, 4: sx(a, 32) < sx(b, 32), 5: sx(a, 32) >= sx(b, 32), 6: a < b, 7: a >= b}[f3]

    def _i(self, ins, pc, x):
        op, rd, f3, rs1, rs2, f7 = ins & 0x7F, (ins >> 7) & 31, (ins >> 12) & 7, (ins >> 15) & 31, (ins >> 20) & 31, ins >> 25
        if op == 0x37: x[rd] = ins & 0xFFFFF000; return pc + 4, "alu"
        if op == 0x17: x[rd] = (pc + (ins & 0xFFFFF000)) & 0xFFFFFFFF; return pc + 4, "alu"
        if op == 0x6F:
            imm = sx(((ins >> 31) << 20) | (((ins >> 12) & 0xFF) << 12) | (((ins >> 20) & 1) << 11) | (((ins >> 21) & 0x3FF) << 1), 21)
            x[rd] = pc + 4; return (pc + imm) & 0xFFFFFFFF, "jump"
        if op == 0x67:
            t = (x[rs1] + sx(ins >> 20, 12)) & 0xFFFFFFFE; x[rd] = pc + 4; return t, "jump"
        if op == 0x63:
            imm = sx(((ins >> 31) << 12) | (((ins >> 7) & 1) << 11) | (((ins >> 25) & 0x3F) << 5) | (((ins >> 8) & 0xF) << 1), 13)
            if self._branch(f3, x[rs1], x[rs2]): return (pc + imm) & 0xFFFFFFFF, "branch_taken"
            return pc + 4, "branch"
        if op == 0x03:
            x[rd] = self._load(f3, (x[rs1] + sx(ins >> 20, 12)) & 0xFFFFFFFF); return pc + 4, "load"
        if op == 0x23:
            imm = sx(((ins >> 25) << 5) | ((ins >> 7) & 31), 12)
            self.wr((x[rs1] + imm) & 0xFFFFFFFF, 1 << f3, x[rs2]); return pc + 4, "store"
        if op == 0x13:
            imm = sx(ins >> 20, 12) & 0xFFFFFFFF
            if f3 in (1, 5): imm &= 31
            x[rd], cls = self._alu(f3, f7 if f3 == 5 else 0, x[rs1], imm, imm=True); return pc + 4, cls
        if op == 0x33:
            x[rd], cls = self._alu(f3, f7, x[rs1], x[rs2]); return pc + 4, cls
        if op == 0x73 or op == 0x0F:
            return pc + 4, "alu"                                    # csr / fence: treated as no-ops
        raise RuntimeError("unknown instruction %08x at %08x" % (ins, pc))

    def _c(self, h, pc, x):
        q, f3 = h & 3, h >> 13
        r = lambda v: 8 + (v & 7)
        if q == 0:
            rdp, rs1p = r(h >> 2), r(h >> 7)
            if f3 == 0:                                            # c.addi4spn
                imm = (((h >> 7) & 0xF) << 6) | (((h >> 11) & 3) << 4) | (((h >> 5) & 1) << 3) | (((h >> 6) & 1) << 2)
                x[rdp] = (x[2] + imm) & 0xFFFFFFFF; return pc + 2, "alu"
            imm = (((h >> 10) & 7) << 3) | (((h >> 6) & 1) << 2) | (((h >> 5) & 1) << 6)
            if f3 == 2: x[rdp] = self.rd((x[rs1p] + imm) & 0xFFFFFFFF, 4); return pc + 2, "load"
            if f3 == 6: self.wr((x[rs1p] + imm) & 0xFFFFFFFF, 4, x[rdp]); return pc + 2, "store"
        elif q == 1:
            rd = (h >> 7) & 31
            imm6 = sx((((h >> 12) & 1) << 5) | ((h >> 2) & 31), 6)
            if f3 == 0: x[rd] = (x[rd] + imm6) & 0xFFFFFFFF; return pc + 2, "alu"
            if f3 in (1, 5):                                       # c.jal / c.j
                imm = sx((((h >> 12) & 1) << 11) | (((h >> 8) & 1) << 10) | (((h >> 9) & 3) << 8) | (((h >> 6) & 1) << 7) |
                         (((h >> 7) & 1) << 6) | (((h >> 2) & 1) << 5) | (((h >> 11) & 1) << 4) | (((h >> 3) & 7) << 1), 12)
                if f3 == 1: x[1] = pc + 2
                return (pc + imm) & 0xFFFFFFFF, "jump"
            if f3 == 2: x[rd] = imm6 & 0xFFFFFFFF; return pc + 2, "alu"
            if f3 == 3:
                if rd == 2:                                        # c.addi16sp
                    imm = sx((((h >> 12) & 1) << 9) | (((h >> 3) & 3) << 7) | (((h >> 5) & 1) << 6) | (((h >> 2) & 1) << 5) | (((h >> 6) & 1) << 4), 10)
                    x[2] = (x[2] + imm) & 0xFFFFFFFF; return pc + 2, "alu"
                x[rd] = (imm6 << 12) & 0xFFFFFFFF; return pc + 2, "alu"
            if f3 == 4:
                rdp = r(h >> 7)
                f2 = (h >> 10) & 3
                sh = (((h >> 12) & 1) << 5) | ((h >> 2) & 31)
                if f2 == 0: x[rdp] = x[rdp] >> sh; return pc + 2, "alu"
                if f2 == 1: x[rdp] = (sx(x[rdp], 32) >> sh) & 0xFFFFFFFF; return pc + 2, "alu"
                if f2 == 2: x[rdp] = x[rdp] & (imm6 & 0xFFFFFFFF); return pc + 2, "alu"
                rs2p = r(h >> 2)
                f = (h >> 5) & 3
                x[rdp] = [(x[rdp] - x[rs2p]) & 0xFFFFFFFF, x[rdp] ^ x[rs2p], x[rdp] | x[rs2p], x[rdp] & x[rs2p]][f]
                return pc + 2, "alu"
            if f3 in (6, 7):                                       # c.beqz / c.bnez
                rs1p = r(h >> 7)
                imm = sx((((h >> 12) & 1) << 8) | (((h >> 5) & 3) << 6) | (((h >> 2) & 1) << 5) | (((h >> 10) & 3) << 3) | (((h >> 3) & 3) << 1), 9)
                take = (x[rs1p] == 0) if f3 == 6 else (x[rs1p] != 0)
                return ((pc + imm) & 0xFFFFFFFF, "branch_taken") if take else (pc + 2, "branch")
        else:
            rd, rs2 = (h >> 7) & 31, (h >> 2) & 31
            if f3 == 0:
                sh = (((h >> 12) & 1) << 5) | ((h >> 2) & 31)
                x[rd] = (x[rd] << sh) & 0xFFFFFFFF; return pc + 2, "alu"
            if f3 == 2:
                imm = (((h >> 12) & 1) << 5) | (((h >> 4) & 7) << 2) | (((h >> 2) & 3) << 6)
                x[rd] = self.rd((x[2] + imm) & 0xFFFFFFFF, 4); return pc + 2, "load"
            if f3 == 4:
                if (h >> 12) & 1 == 0:
                    if rs2 == 0: return x[rd], "jump"               # c.jr
                    x[rd] = x[rs2]; return pc + 2, "alu"             # c.mv
                if rs2 == 0:
                    if rd == 0: raise RuntimeError("ebreak")
                    t = x[rd]; x[1] = pc + 2; return t, "jump"       # c.jalr
                x[rd] = (x[rd] + x[rs2]) & 0xFFFFFFFF; return pc + 2, "alu"
            if f3 == 6:
                imm = (((h >> 9) & 0xF) << 2) | (((h >> 7) & 3) << 6)
                self.wr((x[2] + imm) & 0xFFFFFFFF, 4, x[rs2]); return pc + 2, "store"
        raise RuntimeError("unknown compressed %04x at %08x" % (h, pc))
