# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Peripheral model around the actual rv32emu production bootloader ELF.

Flash erase/program latency is modeled explicitly. Interrupting a model operation
retains only its flash effects; restarting constructs a new MCU/RAM instance.
"""
from bisect import bisect_right
from pathlib import Path
import struct
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
from rv32emu import COST, RAM_BASE, RAM_SIZE, Emu

FLASH, RCC, STK, CRC = 0x40022000, 0x40021000, 0xE000F000, 0x40023000
APP, BODY, END = 0x08002000, 0x08002100, 0x08010000
WORK, FRAME = 0x20004500, 0x20004600


def crc32(data, initial=0xffffffff):
    c = initial
    for (v,) in struct.iter_unpack("<I", data):
        c ^= v
        for _ in range(32):
            c = ((c << 1) ^ (0x04c11db7 if c & 0x80000000 else 0)) & 0xffffffff
    return c


def crc(data, bits, poly, initial):
    c = initial
    for v in data:
        c ^= v << (bits-8)
        for _ in range(8):
            c = ((c << 1) ^ (poly if c & (1 << (bits-1)) else 0)) & ((1 << bits)-1)
    return c


def seal(desc):
    desc[60:64] = crc32(desc[:60]).to_bytes(4, "big")
    return bytes(desc)


def image(target, length=772, version=0x00030100, body=None):
    if body is None:
        body = bytes((i * 37 + i // 256 + 11) & 255 for i in range(length))
    length = len(body)
    desc = bytearray(64)
    desc[:5] = b"QDU6\x02"
    desc[6] = desc[7] = target[7]
    desc[8:15] = target[:7]
    desc[15] = 3
    struct.pack_into(">IIIII", desc, 16, version, length, crc32(body), BODY, BODY)
    desc[36:39] = bytes([2, 3, 1])
    return seal(desc) + body


def frame(op, session=7, index=0, data=bytes(32), flags=0, pass_no=0, time=0):
    b = bytearray(48)
    b[0:6] = bytes([op, session, index >> 8, index & 255, pass_no, flags])
    b[6:38] = data
    return reseal(b, time)


def reseal(b, time=0):
    b = bytearray(b)
    b[38:40] = crc(b[:38], 16, 0x1021, 0xffff).to_bytes(2, "big")
    b[40:43] = b"\x5a\xd2" + bytes([0x39 if b[0] == 0x86 else 0x38])
    b[43:47] = (time & 0xffffffff).to_bytes(4, "big")
    b[47] = crc(b[40:47], 8, 7, 0)
    return bytes(b)


class Reset(Exception):
    pass


class PowerCut(Exception):
    pass


class Booted(Exception):
    pass


class Recovery(Exception):
    pass


class AppMain(Exception):
    """The installed app's startup reached its main: its registers then."""
    def __init__(self, registers, pc):
        super().__init__(hex(pc))
        self.registers, self.pc = registers, pc


class MCU(Emu):
    def __init__(self, elf):
        super().__init__(str(elf))
        self.elf = str(elf)
        self.flash[0x2000:] = bytes([255]) * (len(self.flash)-0x2000)
        self.target = bytes(self.flash[21:37])
        self.now_us = 0.0
        self.timer = 0.0
        self.mhz = 144
        self.ops = []
        self.fail_at = self.cut_at = None
        self.fault = "eop"
        self.erase_us, self.program_us, self.half_us = 20000, 10000, 100
        self.cut_fraction = 0.5
        self.busy_us = None
        self.fast_buffer = {}
        self.options = [0xa5, 0x3f, 0x12, 0x34, 0xfc, 255, 255, 255]
        self.mmio.update({FLASH+16: 0x8080, FLASH+12: 0, FLASH+32: 0xfffffffc,
                          STK: 5, RCC+0x24: 0x08000000, RCC+4: 2})
        self.stopping_boot = False
        self.app_main = None                     # set: the jump runs the installed app's startup to its main
        self.app_entry = None
        self.app_mepc = 0
        self.app_stray = []
        self.keys = {}
        self.setword("clock_mhz", 144)
        self.setword("options_trusted", 1, 1)

    def setword(self, symbol, value, size=4):
        self.poke(self.addr(symbol), value.to_bytes(size, "little"))

    def advance(self, us):
        assert us >= self.now_us
        self.timer += (us-self.now_us) * self.mhz
        self.now_us = float(us)

    def _i(self, instruction, pc, registers):
        if self.app_main is not None and pc in (self.app_main, self.app_main + 0x08000000):
            raise AppMain(list(registers), pc)
        if self.app_entry is not None:           # the app's startup enters main by mepc + mret
            if instruction & 0x707f == 0x1073 and instruction >> 20 == 0x341:
                self.app_mepc = registers[(instruction >> 15) & 31]
                registers[(instruction >> 7) & 31] = 0
                return pc + 4, "alu"
            if instruction == 0x30200073:
                return self.app_mepc, "jump"
        self.last_pc = pc
        result = super()._i(instruction, pc, registers)
        self.advance(self.now_us + COST[result[1]] / self.mhz)
        return result

    def _c(self, instruction, pc, registers):
        if self.app_main is not None and pc in (self.app_main, self.app_main + 0x08000000):
            raise AppMain(list(registers), pc)
        self.last_pc = pc
        result = super()._c(instruction, pc, registers)
        self.advance(self.now_us + COST[result[1]] / self.mhz)
        return result

    def rd(self, a, n):
        if self.busy_us is not None and self.now_us < self.busy_us:
            assert not (a < len(self.flash) or 0x08000000 <= a < 0x08000000+len(self.flash)), \
                "flash fetch/read while busy; deadline path must execute in SRAM"
        if self.stopping_boot:
            if a in (BODY, BODY-0x08000000):
                if self.app_main is None:
                    raise Booted()
                if self.app_entry is None:
                    self.app_entry = a           # the jump's target: the app's startup runs on
            if a == self.addr("bl_sess_init"):
                raise Recovery()
        if a == STK+8:
            return int(self.timer) & 0xffffffff
        if a == FLASH+12 and self.busy_us is not None:
            if self.busy_us == float("inf"):
                self.advance(self.now_us + 100)
                return 1
            self.advance(max(self.now_us, self.busy_us))
            self.busy_us = None
        if 0x1ffff800 <= a < 0x1ffff810:
            v = self.options[(a-0x1ffff800)//2]
            return v | ((v ^ 255) << 8)
        if a == RCC:
            return self.mmio.get(a, 0) | 0x02000000
        if a == RCC+4:
            v = self.mmio.get(a, 0)
            return (v & ~12) | ((v & 3) << 2)
        return super().rd(a, n)

    def operation(self, kind, address, payload, latency):
        assert APP <= address and address + len(payload) <= END, (kind, hex(address), len(payload))
        if kind != "erase":
            assert all((old & new) == new for old, new in zip(self.peek(address, len(payload)), payload)), \
                "programming attempts a zero-to-one flash transition"
        self.ops.append((kind, address, self.now_us))
        count = len(self.ops)
        if count == self.cut_at:
            n = int(len(payload)*self.cut_fraction)
            self.poke(address, payload[:n])
            raise PowerCut()
        if count == self.fail_at:
            self.mmio[FLASH+12] = 0x10 if self.fault == "wrp" else 0
            if self.fault == "stuck":
                self.busy_us = float("inf")
            return
        self.poke(address, payload)
        self.mmio[FLASH+12] = 0x20
        self.busy_us = self.now_us + latency

    def wr(self, a, n, v):
        if self.app_entry is not None and not (RAM_BASE <= a < RAM_BASE + RAM_SIZE or 0x40000000 <= a < 0x60000000
                                               or a >= 0xe0000000):
            self.app_stray.append(a)             # the app's startup storing outside SRAM and peripherals
        if a < len(self.flash) or 0x08000000 <= a < 0x08000000+len(self.flash):
            assert APP <= a and a+n <= END, "CPU write outside the permitted metadata/body slot"
        if a == 0xe000e048 and v == 0xbeef0080:
            raise Reset()
        if a == CRC+8:
            self.mmio[CRC] = 0xffffffff
        elif a == CRC:
            self.mmio[a] = crc32(v.to_bytes(4, "little"), self.mmio.get(a, 0xffffffff))
        elif a == FLASH+12:
            self.mmio[a] &= ~(v & 0x30)
        elif a in (FLASH+4, FLASH+8, FLASH+0x24):
            if self.keys.get(a) == 0x45670123 and v == 0xcdef89ab:
                self.mmio[FLASH+16] &= ~(0x80 if a == FLASH+4 else 0x8000)
            self.keys[a] = v
        elif a == FLASH+16:
            if v & 0x40 and v & 0x20000:
                self.operation("erase", self.mmio[FLASH+20], bytes([255])*256, self.erase_us)
                v &= ~0x40
            if v & 0x200000 and v & 0x10000:
                base = min(self.fast_buffer)
                payload = b"".join(self.fast_buffer[base+i] for i in range(0, 256, 4))
                self.operation("page", base, payload, self.program_us)
                self.fast_buffer.clear()
                v &= ~0x200000
            self.mmio[a] = v
        elif APP <= a < END:
            control = self.mmio[FLASH+16]
            assert not control & 0x80, "write while locked"
            if control & 1:
                assert n == 2
                self.operation("half", a, v.to_bytes(2, "little"), self.half_us)
            else:
                assert control & 0x10000 and n == 4
                self.fast_buffer[a] = v.to_bytes(4, "little")
        elif a == RCC+4:
            self.mmio[a] = v
            self.mhz = (144 if v & 3 == 2 else 8) // (2 if v & 0xf0 else 1)
        elif a == STK and v & 32:
            self.mmio[a] = v
            self.timer = 0
        else:
            self.mmio[a] = v
            super().wr(a, n, v)

    def word(self, symbol):
        return int.from_bytes(self.peek(self.addr(symbol), 4), "little")

    def installed(self, img, confirmed=True):
        self.poke(APP, img[:64] + b"\xb7\xa5" + (b"\xc3\xa5" if confirmed else b"\xff\xff") + bytes([255])*188)
        self.poke(BODY, img[64:])

    def init_session(self):
        self.call("bl_app_read", [WORK])
        self.call("bl_sess_init", [self.addr("sess"), 3, WORK, int(self.now_us//1000)])

    @property
    def state(self):
        return int.from_bytes(self.peek(self.addr("sess")+4, 4), "little")

    def send(self, f, time_us=None):
        if time_us is not None:
            self.advance(max(self.now_us, time_us))
        self.poke(FRAME, f)
        self.call("bl_sess_frame", [self.addr("sess"), 384, FRAME, int(self.now_us//1000)])

    def poll(self):
        self.call("bl_sess_poll", [self.addr("sess"), int(self.now_us//1000)])
        return self.ret

    def drain_erase(self):
        while self.state == 3:
            self.poll()

    def start(self, img, session=7):
        self.send(frame(0x81, session, 0, img[:32]))
        self.send(frame(0x81, session, 1, img[32:64]))
        self.drain_erase()

    def transfer(self, img, session=7, skip=()):
        body = img[64:]
        for index in range((len(body)+31)//32):
            if index in skip:
                continue
            self.send(frame(0x82, session, index, body[index*32:(index+1)*32].ljust(32, b"\0")))
        end = img[24:28] + img[20:24] + bytes(24)
        self.send(frame(0x83, session, data=end))

    def boot_path(self):
        new = MCU(self.elf)
        new.flash[:] = self.flash
        new.setword("clock_mhz", 8)
        new.setword("options_trusted", 0, 1)
        new.mhz = 8
        new.stopping_boot = True
        try:
            new.call("main", max_steps=1000000)
        except Booted:
            return "app"
        except Recovery:
            return "bootloader"
        raise AssertionError("no boot decision")

    def boot_app(self, app_main):
        """The boot path into the installed (real) app, through its startup to its main: (entry, registers, stray
        stores). The startup runs on this model's peripherals; the check is where it put gp, sp and its RAM."""
        new = MCU(self.elf)
        new.flash[:] = self.flash
        new.setword("clock_mhz", 8)
        new.setword("options_trusted", 0, 1)
        new.mhz = 8
        new.stopping_boot = True
        new.app_main = app_main
        try:
            new.call("main", max_steps=3000000)
        except AppMain as reached:
            return new.app_entry, reached.registers, new.app_stray
        except (Booted, Recovery):
            raise AssertionError("the boot path did not run the installed app")
        raise AssertionError("the app's startup never reached its main")


class WireMCU(MCU):
    """Execute main, DMA-ring reception, decoder, session and flash in one MCU.

    Quiet intervals over 2 ms may be fast-forwarded only after the ring has
    drained. Every active receive instruction still advances the cycle estimate.
    Sampling during flash's divided clock is discarded by production resync.
    This model is not a silicon timing or electrical qualification.
    """
    def __init__(self, elf, rows, marker=False, drop=()):
        super().__init__(elf)
        self.wire_words = {}
        self.starts, self.ends, self.falling = [], [], []
        for row in rows:
            now, clock, period, zero, one, encoded = row.split()
            now, clock, period, zero, one = map(int, (now, clock, period, zero, one))
            data = bytes.fromhex(encoded)
            if data[0] == 0x82 and data[4] >> 4 == 0 and int.from_bytes(data[2:4], "big") in drop:
                continue
            self.starts.append(now-40 if marker else now)
            self.ends.append(now+(383*period+(one if data[-1]&1 else zero))*1000000/clock)
            pulses = [(now-40, now-20)] if marker else []
            for index in range(384):
                high = one if data[index//8] & (128 >> (index%8)) else zero
                pulses.append((now+index*period*1000000/clock, now+(index*period+high)*1000000/clock))
            for first, last in pulses:
                self.falling.append(last)
                for sample in range(int(first*36), int(last*36)):
                    word = sample // 16
                    self.wire_words[word] = self.wire_words.get(word, 0) | (1 << (15-sample%16))
        self.sample_word = int(self.now_us*36)//16
        self.edge_base = 0
        self.received = []
        self.main_first, self.main_size = self.sym("main")
        self.sampler_enabled = False
        self.mmio[RCC+0x24] = 0x10000000
        self.mmio[RCC+4] = 0
        self.setword("clock_mhz", 8)
        self.mhz = 8
        magic, enter = 0x51444f48, 0x55504421
        self.poke(0x20000000, struct.pack("<8I", magic, enter, 0, 0, 0, 0, 0,
                                        ~(magic ^ enter) & 0xffffffff))

    def refresh_ring(self):
        end = int(self.now_us*36)//16
        start = max(self.sample_word, end-2048)
        for word in range(start, end):
            value = self.wire_words.get(word, 0)
            self.poke(self.addr("hw_ring")+2*(word & 2047), value.to_bytes(2, "little"))
        self.sample_word = end
        return end & 2047

    def rd(self, a, n):
        if n == 2 and a == self.addr("bl_sess_frame"):
            received = self.peek(self.addr("frame"), 56)
            self.received.append((round(self.now_us), received[4], received[8:14].hex()))
        if self.sampler_enabled:
            if (a == STK+8 and self.state != 3 and
                    self.main_first <= self.last_pc < self.main_first+self.main_size):
                next_frame = bisect_right(self.ends, self.now_us)
                previous = self.ends[next_frame-1] if next_frame else -1000000
                if next_frame < len(self.starts) and self.now_us-previous > 1500:
                    start = self.starts[next_frame]
                    if start-self.now_us > 2000:
                        self.advance(start-250)
                if self.now_us > self.ends[-1]+300000:
                    raise AssertionError("wire stream completed without a committed reset")
            if a == 0x40020020:  # DMA1 channel 2 CNTR
                return 2048-self.refresh_ring()
            if a == 0x40000024:  # TIM2 CNT, DIN falling-edge count
                return (bisect_right(self.falling, self.now_us)-self.edge_base) & 65535
        return super().rd(a, n)

    def wr(self, a, n, v):
        if a == 0x40013000 and v & 64:  # SPI1 SPE
            self.sampler_enabled = True
            self.sample_word = int(self.now_us*36)//16
        if a == 0x40000024:
            self.edge_base = bisect_right(self.falling, self.now_us)-v
        super().wr(a, n, v)

    def run(self, max_steps=None):
        if max_steps is None:
            # Runaway guard: whole wire schedule + 300 ms drain, at <=144/min(COST) instructions/us, with 2x margin.
            max_steps = int((self.ends[-1] + 300000) * 144 / min(COST.values()) * 2) + 1
        try:
            self.call("main", max_steps=max_steps)
        except Reset:
            return
        except Exception as error:
            raise RuntimeError(
                f"whole-main failure: pc={self.last_pc:08x}, us={self.now_us:.1f}, "
                f"state={self.state}, operations={len(self.ops)}, sampler={self.sampler_enabled}, "
                f"received={len(self.received)}, last={self.received[-5:]}"
            ) from error
        raise AssertionError("production main returned")
