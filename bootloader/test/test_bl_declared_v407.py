# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Declared V407 group transactions on the production ELF, including cut/fault recovery.

No SELECT and no status polls authorize these operations. Explicit selection of
another UID must remain excluded. The selected status suites run separately.
"""
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import MCU, APP, BODY, SECTOR, PowerCut, Reset, image, frame, seal


def fresh(elf):
    m = MCU(elf)
    m.installed(image(m.target, version=0x30000))
    m.init_session()
    return m


def announce(m, img, session=7):
    for half in (0, 1):
        m.send(frame(0x81, session, half, img[half*32:(half+1)*32]))


def transfer(m, img, session=7, skip=(), end=True):
    for _ in range(130):
        if m.state != 3: break
        m.run_due()
    if m.state != 1: return
    for i in range((len(img)-64+31)//32):
        if i not in skip:
            m.send(frame(0x82, session, i, img[64+32*i:96+32*i].ljust(32, b'\0')))
        if i % 8 == 7: m.advance(m.now_us + 61000)
    if end:
        m.send(frame(0x83, session, data=img[24:28]+img[20:24]+bytes(24)))


def committed(m, img):
    return m.peek(APP, 64) == img[:64] and m.peek(BODY, len(img)-64) == img[64:]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('elf', type=Path)
    p.add_argument('--json', type=Path)
    a = p.parse_args()
    rows = []
    def case(name, fn):
        try:
            fn()
            row = dict(case=name, ok=True)
        except Exception as e:
            row = dict(case=name, ok=False, error=str(e))
        rows.append(row)
        print(json.dumps(row), flush=True)
    def transaction(length, multipass):
        m = fresh(a.elf); img = image(m.target, length)
        announce(m, img)
        if multipass:
            transfer(m, img, skip=(1, 12), end=False)
            assert not committed(m, img)
        transfer(m, img)
        assert committed(m, img), 'declared image was not committed'
        erases = [addr for kind, addr, _ in m.ops if kind == 'erase']
        assert erases == list(range(APP, BODY+((length+4095)//4096)*4096, SECTOR)), erases
        assert m.ops[0][:2] == ('erase', APP), 'metadata-sector erase must be first'
        assert not m.stv('op', 1) and not m.stv('op_due', 1) and not m.stv('reply', 1)
    for length in (772, 4356):
        for multipass in (False, True):
            case(f'transfer-{length}-multipass-{multipass}', lambda l=length, q=multipass: transaction(l, q))
    def refusal(kind):
        m = fresh(a.elf); img = image(m.target)
        if kind == 'selected-other': m.select(uid=bytes(12))
        if kind in ('family', 'board', 'layout'):
            d = bytearray(img[:64]); d[{'family':10, 'board':8, 'layout':12}[kind]] ^= 0x40
            img = seal(d) + img[64:]
        before = bytes(m.flash)
        announce(m, img); transfer(m, img)
        assert not m.ops and bytes(m.flash) == before, 'excluded/foreign image touched flash'
        assert m.poll() is None and not m.stv('op_due', 1)
    for kind in ('selected-other', 'family', 'board', 'layout'):
        case(kind, lambda k=kind: refusal(k))
    def dirty_revoke(offset):
        m = fresh(a.elf); img = image(m.target)
        operation = m.operation
        def dirty(kind, address, payload, latency):
            operation(kind, address, payload, latency)
            if kind == 'erase' and address == APP: m.poke(APP+offset, b'\0\0')
        m.operation = dirty
        before = m.peek(BODY, 772)
        announce(m, img); transfer(m, img)
        assert m.state == 4 and len(m.ops) == 1, 'full metadata readback must precede body erase'
        assert m.peek(BODY, 772) == before
    for offset in (512, 4094):
        case(f'dirty-metadata-{offset}', lambda o=offset: dirty_revoke(o))
    def repeated(conflict, programmed):
        m = fresh(a.elf); img = image(m.target)
        announce(m, img)
        while m.state == 3: m.run_due()
        for i in range(8 if programmed else 1): m.send(frame(0x82, 7, i, img[64+32*i:96+32*i]))
        before = bytes(m.flash); count = len(m.ops)
        d = bytearray(img[64:96]); d[0] ^= int(conflict)
        m.send(frame(0x82, 7, 0, d))
        assert m.state == (4 if conflict else 1), 'repeat policy'
        assert bytes(m.flash) == before and len(m.ops) == count
    for conflict in (False, True):
        for programmed in (False, True):
            case(f'repeat-{conflict}-{programmed}', lambda c=conflict, p=programmed: repeated(c, p))
    def exit_case(revoked, abort):
        m = fresh(a.elf); img = image(m.target, version=0x20000 if not revoked else 0x30100)
        announce(m, img)
        if revoked:
            while m.state == 3: m.run_due()
        before = bytes(m.flash); ops = len(m.ops)
        if abort: m.send(frame(0x84, 7))
        else:
            # Inactivity counts from the end of the declared erase lead (two units: 500 ms; test_bl_lead_v407.py).
            m.advance(m.now_us + 500000 + 10001000); m.run_due()
        assert bool(m.run_due()) == (not revoked), 'inactivity/ABORT exit policy'
        assert bytes(m.flash) == before and len(m.ops) == ops
    for revoked in (False, True):
        for abort in (False, True):
            case(f'exit-revoked-{revoked}-abort-{abort}', lambda r=revoked, b=abort: exit_case(r, b))
    reference = fresh(a.elf); img = image(reference.target)
    announce(reference, img); transfer(reference, img)
    if committed(reference, img):
        def cut(op, fraction):
            m = fresh(a.elf); m.cut_at = op; m.cut_fraction = fraction
            try: announce(m, img); transfer(m, img)
            except PowerCut: pass
            else: raise AssertionError('cut did not occur')
            m.call('bl_app_read', [0x20006000])
            if m.peek(0x20006000, 1) != b'\0':
                assert committed(m, img) or m.peek(BODY, 772) == image(m.target, version=0x30000)[64:]
            if op == 1: assert all(k != 'page' for k, _, _ in m.ops)
        for op in range(1, len(reference.ops)+1):
            for fraction in (0, .5, 1):
                case(f'cut-{op}-{fraction}', lambda o=op, f=fraction: cut(o, f))
        def fault(mode, op):
            m = fresh(a.elf); m.fail_at = op; m.fault = mode
            try:
                announce(m, img); transfer(m, img)
                assert mode != 'stuck' and m.state == 4
            except Reset: assert mode == 'stuck'
            assert not committed(m, img)
            if op == 1: assert len(m.ops) == 1, 'failed revoke must not erase the body'
        for mode in ('eop', 'wrp', 'stuck'):
            for op in (1, 2, 3, len(reference.ops)):
                case(f'fault-{mode}-{op}', lambda m=mode, o=op: fault(m, o))
        def deadline(which, duration):
            m = fresh(a.elf); setattr(m, which, duration)
            try: announce(m, img); transfer(m, img)
            except Reset: pass
            assert not committed(m, img)
        for which, duration in (('erase_us', 210000), ('program_us', 61000), ('half_us', 61000)):
            case('deadline-'+which, lambda w=which, d=duration: deadline(w, d))
    if a.json: a.json.write_text(json.dumps(rows, indent=2))
    return int(not all(r['ok'] for r in rows))


if __name__ == '__main__': sys.exit(main())
