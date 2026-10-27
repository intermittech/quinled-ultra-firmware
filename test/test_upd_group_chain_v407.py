# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Coupled group-update model: two V203 receivers, V407 app mid-chain and V407 app at its end.

V407 application update dispatch and the whole sampled GPIO tick run from API
builds of main_ur_digital_m.c; boot transactions run the production boot ELFs.
Frames propagate only if the upstream gate (FWD_EN = TIM2 CH4's output compare,
forced HIGH) was already open. Every app runs
its delay line in CHAIN (the update-only forward never opens on a marker port),
and FWD_EN never changes while DIN is HIGH. The public
UPD_APP_TAKE action is the app/boot reset boundary (black-output handover and
startup have separate gates). V203 transaction/forwarding behavior is unchanged.
This is model evidence; it is not a physical mixed-chain qualification.
"""
import argparse
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'tools'), str(ROOT/'bootloader/test')]
import rv32emu as rv
import abi3_model as v203
from abi3_model_v407 import image, frame, APP, BODY, PowerCut
from test_bl_declared_v407 import announce, transfer, committed
from test_bl_forward_v407 import Forward
rv.RAM_SIZE = 200*1024
rv.FLASH_SIZE = 0x90000


class App(rv.Emu):
    def __init__(self, elf, boot, installed=None, chain=True):
        super().__init__(str(elf))
        self.out = 0; self.tick = self.words = self.falls = self.edge_base = self.gate_base = 0
        self.ocm = 4                             # TIM2 OC4M (FWD_EN): 4 forced LOW, 5 forced HIGH, 3 the slot plan
        self.samples = bytearray(); self.events = []
        b = Path(boot).with_suffix('.bin').read_bytes()
        self.flash[:len(b)] = b
        self.target = b[21:37]
        self.installed = installed or image(self.target, version=0x30000)
        self.poke(APP, self.installed[:64]+b'\xb7\xa5\xc3\xa5')
        self.call('upd_boot')
        self.call('dline_init', [self.addr('dl'), 0, self.addr('dl_cap')])
        self.poke(self.addr('dl_on'), b'\1')
        self.poke(self.addr('adc_ready'), b'\1'); self.poke(self.addr('inj_run'), b'\1')
        self.mmio[0x40012400] = 4
        if chain:                                # a CHAIN line (forced CHAIN or AUTO): the chain scanner runs
            self.call('dline_ch_start', [self.addr('dl'), 0])
        self.drive(bytes(25000))

    @property
    def forwarding(self): return self.ocm == 5

    def wr(self, a, n, v):
        if a == 0x40010810:
            self.out = (self.out | (v & 0xffff)) & ~(v >> 16)
        elif a == 0x40010814: self.out &= ~v
        elif a == 0x4000001C:                    # TIM2 CHCTLR2: OC4M drives FWD_EN (PA3)
            before = self.forwarding
            self.ocm = (v >> 12) & 7
            if self.forwarding != before:
                self.events.append(dict(tick=self.tick, open=self.forwarding))
                if self.forwarding:
                    assert self.out & (1 << 10), 'downstream driver not enabled'
                assert not self.mmio.get(0x40010c08, 0) & 16,                     'opened into a live HIGH' if self.forwarding else 'closed during a live HIGH'
        elif a == 0x40000824: self.edge_base = self.falls - v
        elif a == 0x40000024: self.gate_base = self.falls - v
        super().wr(a, n, v)

    def drive(self, bits):
        self.samples.extend(bits)
        while (self.tick+1)*2500 <= len(self.samples):
            self.tick += 1; end = self.tick*625//4
            for w in range(self.words, end):
                at = w*16; part = self.samples[at:at+16]
                value = sum(v << (15-i) for i, v in enumerate(part))
                self.poke(self.addr('dl_ring')+2*(w & 511), struct.pack('<H', value))
                self.falls += sum(self.samples[j-1] and not self.samples[j] for j in range(max(1, at), at+16))
            self.words = end
            self.mmio.update({0x40020020:512-(end & 511), 0x40020034:512-((end+2) & 511),
                              0x40000824:(self.falls-self.edge_base) & 65535,
                              0x40000024:(self.falls-self.gate_base) & 65535,
                              0x40010c08:int(self.samples[self.tick*2500-1])*16})
            self.call('poll_isr')

    def receive(self, f, now_ms):
        self.poke(self.addr('ms_now'), struct.pack('<I', now_ms))
        bits = bytearray()
        for i in range(len(f)*8):
            n = ((i+1)*125//2)-(i*125//2)
            hi = 40 if f[i//8] & (128 >> (i & 7)) else 20
            bits.extend(b'\1'*hi+b'\0'*(n-hi))
        bits.extend(bytes(7500))                 # complete LOW before main decodes the trailer
        self.drive(bits)
        flags = f[-6] & 15
        self.call('ud_header', [flags, now_ms*1000, 0, 0])
        action = 0
        if flags & 8:
            self.poke(0x20018000, f)
            self.call('upd_app_frame', [self.addr('upd'), 0x20018000, now_ms])
            action = self.ret
        self.drive(bytes(25000))
        return action

    def idle(self, now_ms):
        self.poke(self.addr('ms_now'), struct.pack('<I', now_ms))
        self.call('upd_service')
        self.drive(bytes(25000))


class Node:
    def __init__(self, app, boot):
        self.app_elf, self.boot_elf = app, boot
        self.app = App(app, boot)
        self.boot = None; self.takes = 0; self.operations = []; self.blackouts = []
        self.installed = self.app.installed

    @property
    def forwarding(self): return self.app.forwarding if self.app else self.boot.ocm == 5

    def receive(self, f, now_ms):
        if self.app:
            before = bytes(self.app.flash)
            action = self.app.receive(f, now_ms)
            assert bytes(self.app.flash) == before, 'application dispatch must not write flash'
            if action == 1:
                self.takes += 1
                self.boot = Forward(self.boot_elf)
                self.boot.boot_forward()
                self.blackouts.append(self.boot.opened[0])
                self.boot.installed(self.installed)
                self.boot.init_session()
                self.app = None
        else:
            self.boot.send(f)

    def idle(self, now_ms):
        if self.app: self.app.idle(now_ms)
        else:
            while self.boot.state == 3: self.boot.run_due()

    def finish(self, img):
        assert self.boot is not None
        assert self.boot.peek(APP, 64) == img[:64] and self.boot.peek(BODY, len(img)-64) == img[64:]
        self.operations += self.boot.ops
        self.installed = img
        self.app = App(self.app_elf, self.boot_elf, img)
        self.boot = None


def ordinary():
    f = bytearray(40)
    f[-8:-5] = b'\x5a\xd2\x30'
    f[-1] = v203.crc(f[-8:-1], 8, 7, 0)
    return bytes(f)


def close_tests(app, boot):
    rows = []
    for gap in (False, True):
        a = App(app, boot)
        assert not a.forwarding
        assert a.receive(frame(0x85, 7), 0) == 0 and not a.forwarding
        a.receive(frame(0x85, 7), 10)
        assert a.forwarding
        if gap: a.idle(2013)
        passed = []
        for k in range(2):
            passed.append(a.forwarding)
            a.receive(ordinary(), 2020+k*10 if gap else 20+k*10)
        assert passed == ([False, False] if gap else [True, False]), passed
        rows.append(dict(gap=gap, ordinary_passed=passed, ok=True))
    # The transit hold is the ANNOUNCE's declared erase lead (test_upd_transit_hold_v407.py); a half that names no
    # known layout gets the largest known one, 224 x 510 + 100 = 114340 ms; the 2 s idle close counts from its end.
    fallback = 224*510+100
    a = App(app, boot)
    for t in (0, 10): a.receive(frame(0x85, 7), t)
    a.receive(frame(0x81, 7, data=bytes(32)), 20)
    a.receive(frame(0x84, 8), 30)            # a stale session cannot cancel another family's erase hold
    a.idle(10000); assert a.forwarding, 'foreign erase silence must not close the gate'
    a.idle(20+fallback+2000-1); assert a.forwarding
    a.idle(20+fallback+2000+1); assert not a.forwarding
    rows.append(dict(erase_hold_ms=fallback, ok=True))
    a = App(app, boot)
    for t in (0, 10): a.receive(frame(0x85, 7), t)
    start = (1 << 32)-25000
    a.receive(frame(0x81, 7, data=bytes(32)), start)
    a.idle((start+10000) & 0xffffffff); assert a.forwarding
    a.idle((start+fallback+2000-1) & 0xffffffff); assert a.forwarding
    a.idle((start+fallback+2000+1) & 0xffffffff); assert not a.forwarding
    rows.append(dict(case='erase-hold-zero-deadline-wrap', ok=True))
    return rows


def chain(app, boot, oldboot, reverse):
    old = [v203.MCU(str(oldboot)) for _ in range(2)]
    for m in old:
        m.installed(v203.image(m.target, version=0x30000)); m.init_session()
    nodes = [Node(app, boot), Node(app, boot)]
    target407 = nodes[0].app.target
    images = {203:v203.image(old[0].target), 407:image(target407)}
    now = 0; dropped = 0
    def send(f):
        nonlocal now, dropped
        for m in old: m.send(f)
        for n in nodes:
            forward = n.forwarding
            n.receive(f, now)
            if not forward:
                dropped += 1
                break
        now += 10
    order = (407, 203) if reverse else (203, 407)
    for session, family in enumerate(order, 7):
        img = images[family]
        before_old = [bytes(m.flash) for m in old]
        before407 = [n.installed for n in nodes]
        for _ in range(101): send(frame(0x85, session))   # authorized nonzero-session pre-entry preamble
        assert all(n.forwarding for n in nodes)
        for half in (0, 1): send(frame(0x81, session, half, img[half*32:(half+1)*32]))
        now += 50
        for _ in range(101): send(frame(0x85, session))   # entry wait, training, then boot ANNOUNCE
        for half in (0, 1): send(frame(0x81, session, half, img[half*32:(half+1)*32]))
        for m in old: m.drain_erase()
        for n in nodes: n.idle(now)
        now += 500
        for i in range((len(img)-64+31)//32):
            send(frame(0x82, session, i, img[64+32*i:96+32*i].ljust(32, b'\0')))
            if i % 8 == 7: now += 61
        send(frame(0x83, session, data=img[24:28]+img[20:24]+bytes(24)))
        if family == 407:
            for n in nodes: n.finish(img)
            assert all(bytes(m.flash) == b for m, b in zip(old, before_old))
        else:
            for m in old:
                assert m.peek(v203.APP, 64) == img[:64] and m.peek(v203.BODY, len(img)-64) == img[64:]
            assert [n.installed for n in nodes] == before407 and all(n.takes == int(reverse) for n in nodes)
        now += 210
    return dict(order=order, v203_operations=[len(m.ops) for m in old], v407_operations=[len(n.operations) for n in nodes],
                app_entries=[n.takes for n in nodes], initial_closed_frames=dropped,
                boot_blackout_us=max(t for n in nodes for t in n.blackouts), ok=True)


def group_cut_recovery(boot):
    nodes = [Forward(boot), Forward(boot)]
    for m in nodes:
        m.boot_forward(); m.installed(image(m.target, version=0x30000)); m.init_session()
    img = image(nodes[0].target)
    for m in nodes: announce(m, img)
    nodes[0].cut_at = 2                     # middle node loses power during its first body erase
    try: nodes[0].run_due()
    except PowerCut: pass
    else: raise AssertionError('cut did not happen')
    retained = bytes(nodes[0].flash)
    nodes[0] = Forward(boot); nodes[0].flash[:] = retained
    nodes[0].boot_forward(); nodes[0].init_session()
    while nodes[1].state == 3: nodes[1].run_due()
    for m in nodes: m.send(frame(0x84, 7))    # cancel the surviving transaction before a fresh group session
    for m in nodes: announce(m, img, 8)
    for m in nodes:
        transfer(m, img, 8)
        assert committed(m, img) and m.ocm == 5
    return dict(case='middle-node-power-cut-group-retry', both_committed=True, ok=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('app', type=Path); p.add_argument('boot', type=Path); p.add_argument('v203', type=Path)
    p.add_argument('--json', type=Path)
    p.add_argument('--close-only', action='store_true')
    a = p.parse_args()
    rows = close_tests(a.app, a.boot)
    if not a.close_only:
        rows += [chain(a.app, a.boot, a.v203, r) for r in (False, True)]
        rows.append(group_cut_recovery(a.boot))
    print(json.dumps(rows, indent=2))
    if a.json: a.json.write_text(json.dumps(rows, indent=2))
