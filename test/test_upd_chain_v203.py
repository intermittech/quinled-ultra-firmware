# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Group updates through mixed chains with the V203 Digital-M (UD) first, middle and last (model evidence).

Nodes, each fed the previous node's port-2 wire = AND(its input, its FWD_EN), frame by frame in time order:
  - the UD app: an API image of main_ur_digital_m.c (LTO off, model-only big slot). Its real 20 kHz tick (poll_isr ->
    dl_tick) runs over the sampled line, with PA6 / TIM4 read live inside the tick; ud_header / upd_app_frame /
    upd_service are called as the main loop calls them, about 300 us after each frame's last edge (the decode). Long
    silences are compressed (ms_now jumps; 3 ms of LOW are sampled);
  - after its TAKE, the production V203 BOARD 3 bootloader on the sampled wire (whole main, DMA ring, production decoder,
    TIM2 CH2 on FWD_EN: bootloader/test/test_bl_forward_v203.py's model), then the new app;
  - V203 BOARD 1 / 2 bootloaders already in their stay (the same model), for their own or a foreign image;
  - the V407 Digital-M: the models of test_upd_group_chain_v407.py (API app, production bootloader), frame-level.
A downstream node gets a frame only if FWD_EN was open over all of it; a frame cut by FWD_EN fails the run.
Host schedule per subtransfer (docs/update.md port job): >= 1000 ms nonzero-session PING preamble (10 ms apart), the entry
ANNOUNCE pair, the whole-port entry wait, >= 1000 ms training, the boot pair, the image's declared erase lead (its erase
units x 2 x erase bound + 100 ms: V203 20 ms, V407 200 ms), three DATA passes with the block pauses (V203 21 ms, V407
61 ms), END x3, the commit / reboot wait, >= 2002 ms quiet, then ordinary frames. Checked: every matching node commits
its image, a node that skips it (same version) stays in its bootloader through every pass and forwards them, nonmatching
flash is untouched, ANNOUNCE / DATA / END reach every node behind each forwarder whole, transit apps hold through the
foreign lead (the longest holds: test_update.c), nothing is cut. Usage: python test_upd_chain_v203.py <v203 api fw.elf> <v203 bl folder> <v407 api fw.elf> <v407 bl.elf>
[--json out] [--cases a,b] [--workers n]"""
import argparse
from bisect import bisect_right
from concurrent.futures import ProcessPoolExecutor
import json
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / 'tools'), str(ROOT / 'bootloader/test'), str(ROOT / 'test')]
import rv32emu as rv  # noqa: E402
import abi3_model as v203  # noqa: E402
from rv32emu import COST  # noqa: E402
rv.FLASH_SIZE = max(rv.FLASH_SIZE, 0x90000)              # (the model-only API image's big slot; the V407 models' flash)
rv.RAM_SIZE = max(rv.RAM_SIZE, 200 * 1024)

S = 36                                                   # samples per us
BIT, H0, H1 = 45, 11, 34                                 # profile 0 at 36 MHz
UD_GATE_MODE = 0x40000018                                # TIM2 CHCTLR1: OC2M drives FWD_EN (PA1): 4 forced LOW, 5 forced HIGH


def bits_of(f):
    out = bytearray()
    for byte in f:
        for k in range(8):
            h = H1 if byte >> (7 - k) & 1 else H0
            out += b'\1' * h + b'\0' * (BIT - h)
    return out


class UDApp(rv.Emu):
    """the V203 Digital-M app (API image) with its sampled line; FWD_EN changes logged in its own sample count"""

    def __init__(self, elf, bl_bin, desc, O):
        super().__init__(str(elf))
        self.samples = bytearray()
        self.tick = self.words = self.falls = self.edge_base = 0
        self.fwd = 0
        self.log = []                                    # (sample, open)
        self.cyc = 0
        self.in_isr = False
        self.flash[:len(bl_bin)] = bl_bin
        self.poke(0x08002000, desc[:64] + b'\xb7\xa5\xc3\xa5')
        self.call('upd_boot')
        self.call('dline_init', [self.addr('dl'), 0, self.addr('dl_cap')])
        self.poke(self.addr('dl_on'), b'\1')
        self.poke(self.addr('adc_ready'), b'\1')
        self.poke(self.addr('inj_run'), b'\1')
        self.mmio[0x40012400] = 4
        self.call('dline_ch_start', [self.addr('dl'), 1])  # a CHAIN line: the chain scanner runs (never a marker port)
        self.drive(bytes(36000))
        assert self.peek(self.addr('dl') + O['dl_ch_on'], 1) == b'\1', 'chain scanner not running'

    def _i(self, ins, pc, x):
        r = super()._i(ins, pc, x)
        self.cyc += COST[r[1]]
        return r

    def _c(self, ins, pc, x):
        r = super()._c(ins, pc, x)
        self.cyc += COST[r[1]]
        return r

    def now_sample(self):
        return self.tick * 1800 + (self.cyc // 4 if self.in_isr else 0)

    def rd(self, a, n):
        if self.in_isr and a == 0x40000824:              # TIM4: falling edges up to now, inside the tick too
            s1 = min(self.now_sample(), len(self.samples))
            s0 = (self.tick) * 1800
            extra = self.samples[max(0, s0 - 1):s1].count(b'\x01\x00')
            return (self.mmio.get(a, 0) + extra) & 0xffff
        if self.in_isr and a == 0x40010808:              # PA6 now
            s = self.now_sample()
            return 64 if s < len(self.samples) and self.samples[s] else 0
        return super().rd(a, n)

    def wr(self, a, n, v):
        new = self.fwd
        if a == UD_GATE_MODE:
            m = (v >> 12) & 7
            assert m in (4, 5), f'gate mode {m}: the slot plan (3) needs CHAIN decided, which this model never does'
            new = int(m == 5)
        elif a == 0x40000824:
            self.edge_base = self.falls - v
        if new != self.fwd:
            self.log.append((self.now_sample(), new))
            self.fwd = new
        super().wr(a, n, v)

    def drive(self, bits):
        self.samples.extend(bits)
        while (self.tick + 1) * 1800 <= len(self.samples):
            end = (self.tick + 1) * 1800 // 16
            for w in range(self.words, end):
                part = self.samples[w * 16:w * 16 + 16]
                self.poke(self.addr('dl_ring') + 2 * (w & 511), struct.pack('<H', sum(v << (15 - i) for i, v in enumerate(part))))
                at = w * 16
                self.falls += self.samples[max(0, at - 1):at + 16].count(b'\x01\x00')
            self.words = end
            self.tick += 1
            self.mmio.update({0x40020020: 512 - (end & 511), 0x40020034: 512 - ((end + 2) & 511),
                              0x40000824: (self.falls - self.edge_base) & 65535})
            self.cyc, self.in_isr = 0, True
            self.call('poll_isr')
            self.in_isr = False

    def level(self, s):
        i = bisect_right([x[0] for x in self.log], s) - 1
        return self.log[i][1] if i >= 0 else 0

    def receive(self, f, now_ms, gap_us):
        """one frame and the LOW after it (the decode about 300 us in); returns (state over the frame, action)"""
        self.poke(self.addr('ms_now'), struct.pack('<I', now_ms & 0xffffffff))
        self.call('upd_service')
        s0 = len(self.samples)
        b = bits_of(f)
        self.drive(b)
        s1 = s0 + len(b) - (BIT - (H1 if f[-1] & 1 else H0))
        self.drive(bytes(300 * S))
        flags = f[-6] & 15
        self.call('ud_header', [flags, 0, 0, 0])
        action = 0
        if len(f) == 48 and flags & 8 and not flags & 1:
            self.poke(0x20004c00, f)
            self.call('upd_app_frame', [self.addr('upd'), 0x20004c00, now_ms & 0xffffffff])
            action = self.ret
        self.drive(bytes(int(min(max(gap_us - 300, 0), 3000) * S)))
        lv = {self.level(x) for x in (s0, s1)} | {k for x, k in self.log if s0 < x < s1}
        state = 'whole' if lv == {1} else 'none' if lv == {0} else 'partial'
        return state, action

    def idle(self, now_ms):
        self.poke(self.addr('ms_now'), struct.pack('<I', now_ms & 0xffffffff))
        self.call('upd_service')
        self.drive(bytes(600 * S))


class Frame:
    def __init__(self, t_us, data, kind):
        self.t, self.data, self.kind = t_us, bytes(data), kind
        self.end = t_us + len(data) * 8 * 1.25

    def __repr__(self):
        return f'{self.kind}@{self.t / 1000:.1f}ms'


def ordinary():
    f = bytearray(40)
    f[2] = 0x55
    f[30:32] = v203.crc(f[:30], 16, 0x1021, 0xffff).to_bytes(2, 'big')
    f[-8:-5] = b'\x5a\xd2\x30'
    f[-1] = v203.crc(f[-8:-1], 8, 7, 0)
    return bytes(f)


def subtransfer(t, session, img, lead_ms, pause_ms, entry_wait_ms, data_gap_ms=0):
    """the host's frames for one image (frame() of the matching family's model: same wire format)"""
    fr = []

    def at(us, data, kind):
        fr.append(Frame(us, data, kind))
        return fr[-1].end + 500

    for _ in range(101):                                 # >= 1000 ms nonzero-session PING preamble
        at(t, v203.frame(0x85, session), 'ping')
        t += 10000
    for half in (0, 1):
        t = at(t, v203.frame(0x81, session, half, img[32 * half:32 * half + 32]), 'entry announce')
    t += entry_wait_ms * 1000
    for _ in range(101):                                 # >= 1000 ms training
        at(t, v203.frame(0x85, session), 'training')
        t += 10000
    for half in (0, 1):
        t = at(t, v203.frame(0x81, session, half, img[32 * half:32 * half + 32]), 'boot announce')
    t += lead_ms * 1000
    body = img[64:]
    n = (len(body) + 31) // 32
    for p in range(3):
        for i in range(n):
            t = at(t, v203.frame(0x82, session, i, body[32 * i:32 * i + 32].ljust(32, b'\0'), pass_no=p), 'data')
            t += data_gap_ms * 1000                      # (a host may pace DATA slower than the declared pauses)
            if i % 8 == 7 or i == n - 1:
                t += pause_ms * 1000
    for _ in range(3):
        t = at(t, v203.frame(0x83, session, data=img[24:28] + img[20:24] + bytes(24)), 'end')
    t += 2500 * 1000                                     # commit / reboot and >= 2002 ms quiet
    for _ in range(3):
        t = at(t, ordinary(), 'ordinary')
    return fr, t + 20000


class UDNode:
    """the V203 Digital-M: app (API), after a TAKE its production bootloader, after the commit the new app"""

    def __init__(self, name, api, bl_elf, O):
        self.name, self.api, self.bl_elf, self.O = name, api, bl_elf, O
        self.bl_bin = Path(bl_elf).with_suffix('.bin').read_bytes()
        self.target = self.bl_bin[21:37]
        self.installed = v203.image(self.target, 772, 0x00030000)
        self.app = UDApp(api, self.bl_bin, self.installed, O)
        self.takes, self.boots, self.problems = 0, [], []

    def run(self, frames):
        """all of its input frames, in order; returns the frames it forwarded whole"""
        out, i = [], 0
        while i < len(frames):
            f = frames[i]
            if self.app is not None:
                gap = (frames[i + 1].t - f.end) if i + 1 < len(frames) else 5000
                if gap > 4000:                           # a long silence: upd_service through it, 100 ms apart
                    state, action = self.app.receive(f.data, int(f.end // 1000), 3000)
                    k = f.end + 3000
                    while k + 100000 < frames[i + 1].t if i + 1 < len(frames) else False:
                        k += 100000
                        self.app.idle(int(k // 1000))
                else:
                    state, action = self.app.receive(f.data, int(f.end // 1000), gap)
                if state == 'partial':
                    self.problems.append(f'{self.name} app cut {f}')
                if state == 'whole':
                    out.append(f)
                i += 1
                if action == 1:                          # TAKE: black latch, then the reset into the bootloader
                    self.takes += 1
                    t_reset = f.end + 10000
                    rest = [g for g in frames[i:] if g.t >= t_reset]
                    assert len(rest) == len(frames) - i, f'{self.name}: frames inside the reset handover'
                    passed, committed_at = self.boot(rest, t_reset)
                    out += passed
                    i = len(frames) if committed_at is None else next(
                        (k for k in range(i, len(frames)) if frames[k].t >= committed_at), len(frames))
                    self.app = UDApp(self.api, self.bl_bin, self.installed, self.O)
            else:
                i += 1
        return out

    def boot(self, frames, t_reset):
        from test_bl_forward_v203 import Fwd, Wire
        w = Wire()
        for g in frames:
            w.frame(g.data, g.kind, 0.0, at=g.t - t_reset)
        m = Fwd(self.bl_elf, w)
        m.installed(self.installed)
        try:
            m.call('main', max_steps=200000000)
            how = 'returned'
        except v203.Reset:
            how = 'reset'
        except AssertionError as e:
            how = 'end of wire' if 'wire stream completed' in str(e) else 'error: ' + str(e)
        self.problems += [f'{self.name} boot: {p}' for p in m.problems]
        fw = m.forwarded()
        passed = []
        for g, (name, s, _) in zip(frames, fw):
            if s == 'partial':
                self.problems.append(f'{self.name} boot cut {g}')
            elif s == 'whole' and g.t - t_reset < m.now_us:
                passed.append(g)
        committed_at = t_reset + m.now_us if how == 'reset' else None
        img_ok = None
        self.boots.append(dict(exit=how, blackout_us=m.opened_at(), flash_ops=len(m.flash_checks),
                               at_ms=round(t_reset / 1000)))
        if how == 'reset':
            desc = m.peek(v203.APP, 64)
            body_len = struct.unpack('>I', desc[20:24])[0]
            self.installed = desc + m.peek(v203.BODY, body_len)
        return passed, committed_at


class BLNode:
    """a V203 BOARD 1 / 2 / 3 bootloader already in its stay (an earlier entry): the production main on the wire"""

    def __init__(self, name, bl_elf, version=0x00030000):
        self.name, self.bl_elf = name, bl_elf
        self.target = Path(bl_elf).with_suffix('.bin').read_bytes()[21:37]
        self.installed = v203.image(self.target, 772, version)
        self.problems, self.result = [], {}

    def run(self, frames):
        from test_bl_forward_v203 import Fwd, Wire
        if not frames:                                   # nothing reached it
            self.m, self.how, self.before = None, 'no input', None
            self.result = dict(exit='no input')
            self.problems.append(f'{self.name}: no frame reached it')
            return []
        w = Wire()
        for g in frames:
            w.frame(g.data, g.kind, 0.0, at=g.t)
        m = Fwd(self.bl_elf, w)
        m.installed(self.installed)
        before = bytes(m.flash)
        try:
            m.call('main', max_steps=300000000)
            how = 'returned'
        except v203.Reset:
            how = 'reset'
        except AssertionError as e:
            how = 'end of wire' if 'wire stream completed' in str(e) else 'error: ' + str(e)
        self.problems += [f'{self.name}: {p}' for p in m.problems]
        fw = m.forwarded()
        out = []
        for g, (name, s, _) in zip(frames, fw):
            if s == 'partial':
                self.problems.append(f'{self.name} cut {g}')
            elif s == 'whole' and g.t < m.now_us:
                out.append(g)
        self.m, self.how, self.before = m, how, before
        self.result = dict(exit=how, flash_ops=len(m.flash_checks), opened_us=m.opened_at())
        return out


class V407Node:
    """the V407 Digital-M (frame-level model): forwards a frame if its gate was open before it"""

    def __init__(self, name, api, bl):
        from test_upd_group_chain_v407 import Node
        self.name = name
        self.node = Node(api, bl)
        self.problems = []

    def run(self, frames, images):
        out = []
        took = False
        for i, f in enumerate(frames):
            fwd = self.node.forwarding
            self.node.receive(f.data, int(f.end // 1000))
            if fwd:
                out.append(f)
            if f.kind == 'boot announce' and f.data[3] == 1 and self.node.boot is not None:
                self.node.idle(int(f.end // 1000))
                took = True
            if f.kind == 'end' and self.node.boot is not None and (i + 1 == len(frames) or frames[i + 1].kind != 'end'):
                self.node.finish(images[f.data[1]])
            if self.node.app is not None and (i + 1 == len(frames) or frames[i + 1].t - f.end > 4000):
                self.node.idle(int((f.end + 3000) // 1000))
        return out


def run_case(args):
    name, layout, order, length, data_gap, paths = args
    api, bldir, v407_api, v407_bl = paths
    from ur_digital_m_target import offsets
    import abi3_model_v407 as v407
    O = offsets(api)
    bl3 = str(Path(bldir) / 'bl3-wch/bl.elf')
    nodes = []
    for kind in layout:
        if kind == 'UD':
            nodes.append(UDNode(f'UD{len(nodes)}', api, bl3, O))
        elif kind == 'V407':
            nodes.append(V407Node(f'V407-{len(nodes)}', v407_api, v407_bl))
        else:                                            # BL<n>, or BL<n>s: the image's own version installed (it skips)
            b = kind[2]
            nodes.append(BLNode(f'{kind}-{len(nodes)}', str(Path(bldir) / f'bl{b}-wch/bl.elf'),
                                0x00030100 if kind.endswith('s') else 0x00030000))
    t_v203 = Path(bl3).with_suffix('.bin').read_bytes()[21:37]
    t_v407 = Path(v407_bl).with_suffix('.bin').read_bytes()[21:37]
    images = {}
    frames, t = [], 50000.0
    entry_wait = 60
    for session, fam in enumerate(order, 7):
        if fam == 'V407':                                # (lead: units x 2 x 200 ms + 100, layout 4)
            img = v407.image(t_v407, length)
            fr, t = subtransfer(t, session, img, (1 + (length + 4095) // 4096) * 400 + 100, 61, entry_wait, data_gap)
        else:                                            # (lead: units x 2 x 20 ms + 100, layout 1)
            tb = t_v203 if fam == 'V203' else Path(bldir, f'bl{fam[-1]}-wch/bl.bin').read_bytes()[21:37]
            img = v203.image(tb, length)
            fr, t = subtransfer(t, session, img, (1 + (length + 255) // 256) * 40 + 100, 21, entry_wait, data_gap)
        images[session] = img
        frames += fr
    sent = frames
    reached = [len(frames)]
    problems = []
    for n in nodes:
        if isinstance(n, V407Node):
            frames = n.run(frames, images)
        else:
            frames = n.run(frames)
        reached.append(len(frames))
        problems += n.problems
    # every UPD frame of every subtransfer reached the end of the chain unless a node behind a taker / a BL stopped it
    res = dict(case=name, layout=layout, order=order, frames_sent=len(sent), frames_after_each_node=reached)
    for n in nodes:
        if isinstance(n, UDNode):
            res[n.name] = dict(takes=n.takes, boots=n.boots, version=hex(struct.unpack('>I', n.installed[16:20])[0]))
        elif isinstance(n, BLNode):
            res[n.name] = n.result
        else:
            res[n.name] = dict(takes=n.node.takes, version=hex(struct.unpack('>I', n.node.installed[16:20])[0]))
    # expectations
    for n in nodes:
        if isinstance(n, UDNode):
            want = order.count('V203')
            if n.takes != want or len(n.boots) != want or any(b['exit'] != 'reset' for b in n.boots):
                problems.append(f'{n.name}: takes {n.takes}, boots {n.boots}, want {want} committed')
            if 'V203' in order and n.installed[:64] != images[7 + order.index('V203')][:64]:
                problems.append(f'{n.name}: V203 image not installed')
        elif isinstance(n, V407Node):
            if 'V407' in order and n.node.installed != images[7 + order.index('V407')]:
                problems.append(f'{n.name}: V407 image not installed')
        else:
            fam = n.name.split('-')[0]
            if n.m is None:
                problems.append(f'{n.name}: no input')
            elif fam.endswith('s'):                      # it skips: no flash work, it stayed through END and left then
                img = images[7 + order.index(fam[:-1])]
                ends = [f.end for f in sent if f.kind == 'end']
                if n.m.flash_checks or n.how != 'reset' or n.m.now_us < ends[0]:
                    problems.append(f'{n.name}: skip node: {len(n.m.flash_checks)} flash ops, {n.how} at '
                                    f'{n.m.now_us / 1000:.0f} ms (first END at {ends[0] / 1000:.0f} ms)')
            elif fam in order:
                img = images[7 + order.index(fam)]
                if n.how != 'reset' or not (n.m.peek(v203.APP, 64) == img[:64] and
                                            n.m.peek(v203.BODY, len(img) - 64) == img[64:]):
                    problems.append(f'{n.name}: its image not committed ({n.how})')
            elif bytes(n.m.flash) != n.before:
                problems.append(f'{n.name}: nonmatching flash changed')
    # the last node saw every UPD frame the host sent before its own last subtransfer's END (ordinary ones may stop)
    upd_sent = [f for f in sent if f.kind != 'ordinary']
    res.update(ok=not problems, failures=problems[:12], upd_frames_sent=len(upd_sent))
    return res


CASES = [
    ('UD first: V407 image first (a transit hold over its lead), then the UD image', ['UD', 'V407'], ['V407', 'V203']),
    ('UD first: UD image first, then V407 (its bootloader forwards to the V407)', ['UD', 'V407'], ['V203', 'V407']),
    ('UD middle: V407 first', ['V407', 'UD', 'V407'], ['V407', 'V203']),
    ('UD middle: UD first', ['V407', 'UD', 'V407'], ['V203', 'V407']),
    ('UD last: V407 first', ['V407', 'UD'], ['V407', 'V203']),
    ('UD last: UD first', ['V407', 'UD'], ['V203', 'V407']),
    ('two UDs: the first one\'s bootloader forwards to the second', ['UD', 'UD'], ['V203']),
    ('UD transit for a BOARD 1 image to a BOARD 1 bootloader; BOARD 2 untouched', ['UD', 'BL1', 'BL2'], ['BL1']),
    ('UD bootloader forwards to BOARD 1 / 2 bootloaders (foreign: untouched)', ['UD', 'BL1', 'BL2'], ['V203']),
    ('a skipping BOARD 1 bootloader stays through three passes paced over 13.8 s, forwarding to an older one',
     ['BL1s', 'BL1'], ['BL1'], 772, 200),
]


def main():
    p = argparse.ArgumentParser()
    p.add_argument('api'); p.add_argument('bldir'); p.add_argument('v407_api'); p.add_argument('v407_bl')
    p.add_argument('--json', type=Path)
    p.add_argument('--cases', default='')
    p.add_argument('--workers', type=int, default=5)
    a = p.parse_args()
    paths = (a.api, a.bldir, a.v407_api, a.v407_bl)
    jobs = [(c[0], c[1], c[2], c[3] if len(c) > 3 else 772, c[4] if len(c) > 4 else 0, paths) for i, c in enumerate(CASES)
            if not a.cases or str(i) in a.cases.split(',')]
    rows, fails = [], 0
    with ProcessPoolExecutor(max_workers=max(1, min(a.workers, len(jobs)))) as pool:
        for r in pool.map(run_case, jobs):
            rows.append(r)
            fails += not r['ok']
            print(f"  {r['case']}: {'ok' if r['ok'] else 'FAIL'}; frames after each node {r['frames_after_each_node']}; "
                  + json.dumps({k: v for k, v in r.items() if k.startswith(('UD', 'V407', 'BL'))})
                  + ''.join('\n      ' + f for f in r['failures']), flush=True)
    print('upd chain v203: %s' % (f'FAIL, {fails} of {len(rows)}' if fails else f'ok, {len(rows)} cases'))
    if a.json:
        a.json.write_text(json.dumps(rows, indent=2))
    return 1 if fails else 0


if __name__ == '__main__':
    raise SystemExit(main())
