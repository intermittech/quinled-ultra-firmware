#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""order_proof.py <objdump> <a.elf> <b.elf>: the two images hold the same code and data, apart from the placement and
the call encodings (the call-graph order, tools/link_order.py). Both linked with -Wl,--emit-relocs
(tools/build_target.sh LINK_EXTRA).

Every named symbol of every allocated section with contents is compared, by name, over its extent (its size; a symbol
without one: up to the next symbol):
 - code (a function, or an asm label in an executable section): its instructions (objdump -dr), each relocated operand
   as its relocation's symbol and addend (an address that moved compares equal); calls and tail jumps (JAL, RVC_JUMP,
   CALL / CALL_PLT with its jalr) as one form whatever their encoding (c.jal / jal / auipc + jalr); a branch or jump
   without a relocation inside the symbol as the number of instructions to its target. Every byte of the extent must
   decode; an asm label's padding after its last instruction (nop, c.nop, zeros) is left out;
 - data (an object, anything in a section without code): its bytes from the ELF, each relocated field as its
   relocation (a section-relative one as the symbol it points into);
 - the bytes outside every symbol (unnamed constants, the alignment padding): as runs in address order, without the
   padding at their ends;
 - the objects without contents (.bss, .noinit): name and size.
Prints the counts and every difference; exit 1 on any.
"""
import bisect
import re
import struct
import subprocess
import sys

RX_IN = re.compile(r'^\s*([0-9a-f]+):\s+((?:[0-9a-f]{2,8} )+)\s*(\S*)\s*(.*)$')
RX_ANY = re.compile(r'^\s*([0-9a-f]+):\t')
RX_RL = re.compile(r'^\s+([0-9a-f]+): (R_RISCV_\w+)\s+(\S+)\s*$')
JUMPS = ('R_RISCV_JAL', 'R_RISCV_RVC_JUMP', 'R_RISCV_CALL', 'R_RISCV_CALL_PLT')
SKIP = ('R_RISCV_RELAX', 'R_RISCV_NONE', 'R_RISCV_ALIGN')
PAD = ('13000000', '0100', '0000', '00')                 # nop, c.nop, zeros: the alignment between things
R_SKIP = (0, 43, 51)                                     # (in the ELF: NONE, ALIGN, RELAX)
R_WIDTH = {1: 4, 2: 8, 33: 1, 34: 2, 35: 4, 36: 8, 37: 1, 38: 2, 39: 4, 40: 8, 54: 1, 55: 2, 56: 4, 57: 4}


def read_elf(path):
    """the allocated sections with contents, the named symbols, the relocations of those sections"""
    b = open(path, 'rb').read()
    shoff, = struct.unpack_from('<I', b, 0x20)
    shentsize, shnum, shstrndx = struct.unpack_from('<HHH', b, 0x2e)
    sh = [struct.unpack_from('<10I', b, shoff + i * shentsize) for i in range(shnum)]

    def cstr(off):
        return b[off:b.index(b'\0', off)].decode('latin-1')
    names = [cstr(sh[shstrndx][4] + s[0]) for s in sh]
    secs = {}                                            # index -> (name, address, bytes, executable)
    for i, s in enumerate(sh):
        if s[1] == 1 and s[2] & 2 and s[5]:
            secs[i] = (names[i], s[3], b[s[4]:s[4] + s[5]], bool(s[2] & 4))
    allsyms, syms, bss = [], [], []
    for i, s in enumerate(sh):
        if s[1] != 2:
            continue
        stab = sh[s[6]][4]
        for k in range(s[5] // 16):
            nm, val, size, info, other, shn = struct.unpack_from('<IIIBBH', b, s[4] + k * 16)
            name = cstr(stab + nm)
            allsyms.append((name, val, info & 15, shn))
            if info & 15 not in (0, 1, 2) or not name or name.startswith(('.L', '$')):
                continue
            if shn in secs:
                syms.append((val, size, info & 15, name, shn))
            elif 0 < shn < shnum and sh[shn][1] == 8 and sh[shn][2] & 2:
                bss.append((name, size))                 # (NOBITS)
    rels = {}                                            # section index -> [(address, type, symbol index, addend)]
    for s in sh:
        if s[1] == 4 and s[7] in secs:
            rels[s[7]] = sorted(struct.unpack_from('<IIi', b, s[4] + k * 12) for k in range(s[5] // 12))
    return secs, syms, bss, allsyms, rels


def norm_ops(mn, ops, rels, rel_target=None):
    """an instruction with its relocated operand(s) as symbol + addend; rel_target: a branch / jump without a
    relocation inside the same symbol, as the number of instructions to its target"""
    ops = ops.split('#')[0].strip()
    if not rels:
        if rel_target is not None:
            return f'{mn} ' + re.sub(r'\b[0-9a-f]{3,8}\b\s*<[^>]*>', f'+{rel_target}', ops)
        return f'{mn} {ops}'
    # objdump aliases ADDI rd,rs,0 to MV when a relocated low part is zero.
    # Keep the registers and relocation; only the printed alias changes.
    if mn == 'mv' and any(r[0] == 'R_RISCV_LO12_I' for r in rels):
        mn, ops = 'addi', ops + ',0'
    regs = re.sub(r'\s*<[^>]*>', '', ops)                # (objdump's label for a target: either name)
    regs = re.sub(r'-?0x[0-9a-f]+|-?\b\d+\b|\b[0-9a-f]{3,8}\b', '#', regs)
    return f'{mn} {regs} @' + ','.join(f'{r[0]}:{r[1]}' for r in rels)


def jump_rd(mn, ops):
    """the link register of a jal / c.jal / j / c.j, or of the jalr / jr of an auipc pair"""
    ops = ops.split('#')[0].strip()
    if mn in ('j', 'c.j', 'jr', 'c.jr'):
        return 'zero'
    m = re.match(r'^(\w+),', ops)
    return m.group(1) if m and mn in ('jal', 'jalr') else 'ra'


def le(groups):
    """objdump's hex groups (each a little-endian word / halfword / byte) as the bytes in memory order"""
    return ''.join(''.join(g[i:i + 2] for i in range(len(g) - 2, -2, -2)) for g in groups.split())


def strip_pad(h):
    """hex bytes without the alignment padding at either end"""
    while True:
        for p in PAD:
            if h.endswith(p):
                h = h[:-len(p)]
                break
            if h.startswith(p):
                h = h[len(p):]
                break
        else:
            return h


def load(elf):
    secs, syms, bss, allsyms, rels = read_elf(elf)
    ext = []                                             # (name, start, end, type, section)
    for val, size, typ, name, shn in syms:
        if size:
            end = val + size
        else:
            nxt = [v for v, s, t, n, h in syms if h == shn and v > val]
            end = min(nxt) if nxt else secs[shn][1] + len(secs[shn][2])
        ext.append((name, val, end, typ, shn))

    def where(addr):
        """an address as the symbol it lies in (+ offset), else as the next symbol (- distance)"""
        best = None
        for name, a, e, typ, shn in ext:
            if a <= addr < e and (best is None or (e - a, name) < (best[2] - best[1], best[0])):
                best = (name, a, e)
        if best:
            return f'{best[0]}+{addr - best[1]:#x}'
        nxt = min(((a, name) for name, a, e, typ, shn in ext if a > addr), default=None)
        return f'{nxt[1]}-{nxt[0] - addr:#x}' if nxt else f'{addr:#x}'

    def data(shn, a, e):
        """the bytes a..e of a section, each relocated field as its relocation"""
        sname, saddr, d, x = secs[shn]
        out, p = [], a
        for ra, info, add in rels.get(shn, []):
            t = info & 255
            if ra < a or ra >= e or t in R_SKIP:
                continue
            out.append(d[p - saddr:max(p, ra) - saddr].hex())
            sn, sv, styp, sshn = allsyms[info >> 8]
            tgt = where(sv + add) if styp == 3 or not sn else f'{sn}{add:+#x}'
            out.append(f'@{t}:{tgt};')
            p = max(p, ra + R_WIDTH.get(t, 0))           # (a type of unknown width: its bytes stay)
        out.append(d[p - saddr:e - saddr].hex())
        return ''.join(out)

    # the code, decoded
    cmd = [OBJDUMP, '-dr', '-z'] + [a for i in secs if secs[i][3] for a in ('-j', secs[i][0])] + [elf]
    out = subprocess.run(cmd, capture_output=True, text=True).stdout
    items, last = [], None
    for l in out.split('\n'):
        m = RX_RL.match(l)
        if m and last is not None:
            last[2].append((m.group(2), m.group(3)))
            continue
        m = RX_IN.match(l)
        if m:
            last = [m.group(3), m.group(4), [], le(m.group(2)), int(m.group(1), 16)]
            items.append(last)
        elif RX_ANY.match(l):
            last = ['?', l.strip(), [], '', int(RX_ANY.match(l).group(1), 16)]   # (a line without its bytes)
            items.append(last)
    items.sort(key=lambda it: it[4])
    addrs = [it[4] for it in items]

    res = {}
    for name, a, e, typ, shn in ext:
        if not (secs[shn][3] and typ in (0, 2)):        # data
            res.setdefault(name, []).append(('D', data(shn, a, e)))
            continue
        its = items[bisect.bisect_left(addrs, a):bisect.bisect_left(addrs, e)]
        p, ok = a, True
        for it in its:
            ok = ok and it[4] == p and it[3] != ''
            p = it[4] + len(it[3]) // 2
        ok = ok and p == e
        if typ == 0:
            while its and its[-1][3] in PAD:
                its = its[:-1]
        idx = {it[4]: i for i, it in enumerate(its)}
        norm, i = ([] if ok else ['(bytes not decoded)']), 0
        while i < len(its):
            mn, ops, rl, raw, ad = its[i]
            rl = [r for r in rl if r[0] not in SKIP]
            if any(r[0] in JUMPS for r in rl):
                rd = jump_rd(mn, ops)
                if mn == 'auipc' and i + 1 < len(its) and its[i + 1][0] in ('jalr', 'jr'):
                    rd = jump_rd(its[i + 1][0], its[i + 1][1])
                    i += 1                               # (the pair as one)
                norm.append(f'JUMP {rd} -> ' + ','.join(r[1] for r in rl))
            else:
                rt = None
                t = re.search(r'\b([0-9a-f]{3,8})\b\s*<', ops.split('#')[0])
                if t and int(t.group(1), 16) in idx:
                    rt = idx[int(t.group(1), 16)] - i
                norm.append(norm_ops(mn, ops, rl, rt))
            i += 1
        res.setdefault(name, []).append(('F', norm))
    runs = []
    for shn in sorted(secs, key=lambda i: secs[i][1]):
        saddr, end = secs[shn][1], secs[shn][1] + len(secs[shn][2])
        p = saddr
        for a, e in sorted((a, e) for n, a, e, t, h in ext if h == shn) + [(end, end)]:
            if a > p:
                h = strip_pad(data(shn, p, a))
                if h:
                    runs.append(h)
            p = max(p, e)
    res['(outside the symbols)'] = [('G', runs)]
    res['(without contents)'] = [('B', sorted(bss))]
    return res


def main():
    global OBJDUMP
    OBJDUMP, A, B = sys.argv[1:4]
    a, b = load(A), load(B)
    bad = 0
    for n in sorted(set(a) ^ set(b)):
        print('only in one image:', n)
        bad += 1
    jumps = funcs = 0
    for n in sorted(set(a) & set(b)):
        la, lb = sorted(a[n], key=repr), sorted(b[n], key=repr)     # (a name more than once: as a set)
        if la != lb:
            bad += 1
            print('differs:', n)
            for (ka, x), (kb, y) in zip(la, lb):
                if ka == 'F' and kb == 'F':
                    for i, (p, q) in enumerate(zip(x, y)):
                        if p != q:
                            print(f'    [{i}]', p, '|', q)
                            break
                    if len(x) != len(y):
                        print(f'    {len(x)} | {len(y)} instructions')
        for k, x in la:
            if k == 'F':
                funcs += 1
                jumps += sum(1 for s in x if s.startswith('JUMP'))
    g = a['(outside the symbols)'][0][1]
    print(f'{len(set(a) & set(b)) - 2} symbols compared ({funcs} code, {jumps} calls / jumps by relocation), '
          f'{len(g)} runs of bytes outside them ({sum(len(x) for x in g) // 2} B), '
          f'{len(a["(without contents)"][0][1])} without contents; {bad} differences')
    sys.exit(1 if bad else 0)


if __name__ == '__main__':
    main()
