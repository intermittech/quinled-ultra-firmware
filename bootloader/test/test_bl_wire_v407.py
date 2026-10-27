# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The V407 bootloader's whole main loop on the wire: DIN sampled at 50 MHz into the DMA ring (SPI1 RX, DMA1 ch2
CNTR), the real decoder (bl_rx.c), the frame end in local ticks (bl_main.c: frame_t0 from the ring position, the
unfed words and the decoder's LOW run), the rate lock from the trailer TIMEs, and the status answer on the answer
UART (hw_reply). The host sends a SELECT, 1.1 s of keepalive PINGs and an UPD_STATUS poll, each behind its marker,
at 800 kHz, on a host clock off the board's HCLK by the HSI error. Checked: every decoded frame's frame_t0 within
2 us of its true last falling edge, and the answer burst on the host clock: enable 270..900 us after the poll's end,
the lead byte and 32 answer bytes, the burst over by 900 + pitch - 30 us.

    python bootloader/test/test_bl_wire_v407.py <bl.elf> <out dir>
"""
import argparse
from bisect import bisect_right
from itertools import product
import json
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import MCU, UID, frame, reseal          # noqa: E402

DMA2_CNTR, TIM2_CNT, SPI1_CTLR1, GPIOB_INDR = 0x40020020, 0x40000024, 0x40013000, 0x40010C08   # TIM2: the gate's edge count
USART7, GPIOA_BSHR, GPIOA_BCR = 0x40001C00, 0x40010810, 0x40010814
RING = 4096


class Done(Exception):
    pass


class Wire(MCU):
    def __init__(self, elf, true_mhz, release_us=80, marker=True):
        super().__init__(elf, options=None)
        self.true_mhz = true_mhz
        self.release_us = release_us
        self.marker = marker
        self.base = None                                   # local tick of sample 0 (SPI enabled)
        self.pulses = []                                   # (start tick, end tick) HIGH intervals
        self.fall_ticks = []
        self.frames = []                                   # (true t0 tick, host TIME, frame bytes)
        self.filled = 0
        self.edge_base = 0
        self.t0_seen = []
        self.tx_end, self.buffered, self.bytes, self.de = 0, None, [], []

    # ------------------------------------------------------------------------------------------- the line
    def bits(self, start_tick, f):
        """a marker (20 us HIGH, 10 us LOW), then 384 bits at 800 kHz (1.25 us; 0: 0.4 us, 1: 0.8 us HIGH)"""
        us = self.true_mhz
        out = [(start_tick, start_tick + round(20 * us))] if self.marker else []
        for i in range(384):
            t = start_tick + round(((30 if self.marker else 0) + 1.25 * i) * us)
            bit = (f[i // 8] >> (7 - i % 8)) & 1
            out.append((t, t + round((0.8 if bit else 0.4) * us)))
        return out

    def add_frame(self, start_tick, f, host_of, release=False):
        """the frame with its TIME = the host time of its own last falling edge (which depends on its last bit)"""
        for last in (0, 1, 0):
            end = self.bits(start_tick, bytes(47) + bytes([last]))[-1][1]
            ff = reseal(f, host_of(end))
            if ff[47] & 1 == last:
                break
        pulses = self.bits(start_tick, ff)
        assert pulses[-1][1] == end
        self.pulses += pulses
        if release:                                        # RW: 60..150 us LOW, reference sender aims at 80 us
            self.pulses.append((end + round(self.release_us * self.true_mhz), end + round(4000 * self.true_mhz)))
        self.frames.append((end, host_of(end), ff))
        return end

    def level(self, tick):
        k = bisect_right(self.pulses, (tick, 1 << 62)) - 1
        return 1 if k >= 0 and self.pulses[k][0] <= tick < self.pulses[k][1] else 0

    def refresh(self):
        word = (int(self.timer) - self.base) // 64
        start = max(self.filled, word - RING)
        ring = self.addr("hw_ring")
        for w in range(start, word):
            v = 0
            t0 = self.base + w * 64
            k = bisect_right(self.pulses, (t0 + 64, 1 << 62))
            j = max(0, bisect_right(self.pulses, (t0 - 40000, 0)) - 1)
            for a, b in self.pulses[j:k]:
                if b <= t0 or a >= t0 + 64:
                    continue
                for s in range(16):
                    ts = t0 + 4 * s
                    if a <= ts < b:
                        v |= 0x8000 >> s
            self.poke(ring + 2 * (w % RING), v.to_bytes(2, "little"))
        self.filled = word
        return word % RING

    # ------------------------------------------------------------------------------------------- peripherals
    def bit_ticks(self):
        return 2 * (self.mmio.get(USART7 + 8, 1) or 1)

    def shift(self):
        t = int(self.timer)
        if self.buffered is not None and t >= self.tx_end:
            self.bytes.append((self.tx_end, self.buffered))
            self.tx_end += 10 * self.bit_ticks()
            self.buffered = None

    def _check_app(self, pc, registers):
        if pc == self.addr("bl_sess_frame"):
            t0 = int.from_bytes(self.peek(self.addr("sess") + self.off["frame_t0"], 4), "little")
            true = max((fr[0] for fr in self.frames if fr[0] <= int(self.timer)), default=None)
            self.t0_seen.append((t0, true))
        super()._check_app(pc, registers)

    def rd(self, a, n):
        if self.frames and self.frames[-1][2][0] == 0x86 and int(self.timer) > self.frames[-1][0] + 5000*self.true_mhz:
            raise RuntimeError('no answer before the marker listen deadline')
        if a == DMA2_CNTR and self.base is not None:
            return RING - self.refresh()
        if a == TIM2_CNT and self.base is not None:
            falls = bisect_right(self.falls(), int(self.timer))
            return (falls - self.edge_base) & 0xffff
        if a == GPIOB_INDR:
            return self.level(int(self.timer)) << 4 if self.base is not None else 0
        if a == USART7:
            self.shift()
            t = int(self.timer)
            return (0x80 if self.buffered is None else 0) | (0x40 if self.buffered is None and t >= self.tx_end else 0)
        if a == self.STK + 8 and self.base is not None and self.pulses:
            # fast forward through quiet time (the ring drained): up to 250 us before the next marker
            now = int(self.timer)
            k = bisect_right(self.pulses, (now, 1 << 62))
            if k < len(self.pulses):
                nxt = self.pulses[k][0]
                if nxt - now > int(3000 * self.true_mhz) and now - self.last_edge() > int(1500 * self.true_mhz):
                    self.advance((nxt - int(250 * self.true_mhz)) / self.mhz)
        return super().rd(a, n)

    def falls(self):
        if len(self.fall_ticks) != len(self.pulses):
            self.fall_ticks = [b for _, b in self.pulses]
        return self.fall_ticks

    def last_edge(self):
        ends = self.falls()
        k = bisect_right(ends, int(self.timer))
        return ends[k-1] if k else -10 ** 12

    def wr(self, a, n, v):
        t = int(self.timer)
        if a == SPI1_CTLR1 and v & 0x40 and self.base is None:
            self.base = t
            self.filled = 0
        if a == TIM2_CNT:
            self.edge_base = bisect_right(self.falls(), t) - v
        if a == USART7 + 4:
            self.shift()
            if self.buffered is None and t >= self.tx_end:
                self.bytes.append((t, v & 255))
                self.tx_end = t + 10 * self.bit_ticks()
            else:
                assert self.buffered is None, "UART overrun"
                self.buffered = v & 255
            return
        if a == GPIOA_BSHR and v & (1 << 11):
            self.de.append(("on", t))
        if a == GPIOA_BCR and v & (1 << 11):
            self.de.append(("off", t))
            if self.bytes:
                raise Done()
        super().wr(a, n, v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("elf", type=Path)
    ap.add_argument("out", type=Path)
    ap.add_argument('--release-us', type=int, nargs='+', default=[60, 80, 132, 150])
    ap.add_argument('--hsi', type=float, nargs='+', default=[0.0, -0.045, 0.022])
    ap.add_argument('--unmarked', action='store_true', help='CHAIN status polls must remain silent')
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)
    rows, bad = [], 0
    for hsi, release_us in product(a.hsi, a.release_us):
        true_mhz = 200.0 * (1 + hsi) * (1 - 0.024)          # the boot-trimmed HSI (12 steps low) and its error
        w = Wire(a.elf, true_mhz, release_us, marker=not a.unmarked)
        w.STK = 0xE000F000
        w.mmio[0x40021000 + 0x24] = 1 << 27                    # power-on reset; no app: recovery listens
        w.mhz = 20.0
        w.setword("clock_khz", 20000)
        w.setword("options_trusted", 0, 1)
        # schedule once the sampler runs: 3 ms after the start, then the session
        state = dict(scheduled=False)
        orig_wr = w.wr

        def wr(addr, n, v, w=w, state=state, true_mhz=true_mhz, orig=orig_wr):
            orig(addr, n, v)
            if w.base is not None and not state["scheduled"]:
                state["scheduled"] = True
                t = w.base + int(3000 * true_mhz)
                host0 = 10_000_000
                target = bytes(w.flash[0x15:0x25])
                seq = [frame(0x87, 9, 0, UID + target + bytes(4))]
                seq += [frame(0x85, 9, 0, bytes(32))] * 12
                seq += [frame(0x86, 9, 0, UID + bytes([0, 1, 0]) + bytes(17))]
                for k, f in enumerate(seq):
                    w.add_frame(t, f, lambda tick: (host0 + int((tick - w.base) / true_mhz)) & 0xffffffff,
                                release=f[0] == 0x86)
                    t += int(100_000 * true_mhz)
        w.wr = wr
        try:
            w.call("main", max_steps=60_000_000)
            outcome = "main returned"
        except Done:
            outcome = "answered"
        except Exception as e:                                 # noqa: BLE001
            outcome = f"{type(e).__name__}: {e}"
        # A silent receiver continues decoding after the release drops again;
        # that overlong HIGH can produce an additional bad frame. Compare the
        # scheduled frames, while independently requiring no UART activity.
        seen = w.t0_seen[:len(w.frames)] if a.unmarked else w.t0_seen
        t0_err = [abs(t0 - (true if true is not None else t0)) / true_mhz for t0, true in seen]
        poll_t0 = w.frames[-1][0]
        row = dict(hsi=hsi, release_us=release_us, outcome=outcome, frames_decoded=len(w.t0_seen),
                   worst_t0_error_us=round(max(t0_err), 3) if t0_err else None)
        ok = outcome == "answered" and len(w.t0_seen) == len(w.frames) and max(t0_err) <= 2.0
        if a.unmarked:
            ok = ('no answer before' in outcome and len(w.t0_seen) >= len(w.frames) and
                  max(t0_err) <= 2.0 and not w.bytes and not any(k == 'on' for k, _ in w.de))
        if w.de and w.bytes:
            en = (max(t for k, t in w.de if k == "on" and t <= w.bytes[0][0]) - poll_t0) / true_mhz
            end = (w.de[-1][1] - poll_t0) / true_mhz
            row.update(enable_us=round(en, 2), burst_end_us=round(end, 2), bytes=len(w.bytes))
            ok = ok and 270 <= en <= 900 and end <= 900 + 500 - 30 and len(w.bytes) == 33 and w.bytes[0][1] == 0xff
        row["ok"] = ok
        row["de_events"] = [(k, round((t - poll_t0) / true_mhz, 1)) for k, t in w.de]
        bad += not ok
        rows.append(row)
        print(json.dumps(row), flush=True)
    (a.out / "wire_v407.json").write_text(json.dumps(rows, indent=2))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
