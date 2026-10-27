# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""The Diff-Power-UR-Digital-M's answers and status LED (BOARD=3) on the compiled firmware.

tools/rv32emu.py runs the ELF's own ud_frame (the header, the POLL, ud_rc_page, rc_answer, rc_brr) on frames built
here, and checks every answer (SYNC, PAGE, index, SEQ, UID16, CRC and each field) against the state poked into the ELF
and the protocol header's offsets (src/core/ultra_protocol.h, read here); then IDENTIFY 5 and an update's SAME / NACK
on the status LED (led_service). The state's field offsets come from test/ur_digital_m_offsets.c, compiled here with the target
compiler ($RISCV_GCC, else the xPack GCC in %LOCALAPPDATA%\\xPacks) and read back from the object.

Build (the functions as written: no inlining or IPA clones, so they keep their arguments):
  XW=0 LTO=0 BOARD=3 EXTRA_CFLAGS="-fno-inline -fno-ipa-cp -fno-ipa-sra" OUT=build/ur_digital_m-api bash tools/build_target.sh
Run: python test/test_ur_digital_m_pages.py build/ur_digital_m-api/fw.elf  (tools/build_release.sh runs it for UR_DIGITAL_M=1)
"""
import re
import struct
import sys
from pathlib import Path

FW = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(FW / "tools"))
from rv32emu import Emu  # noqa: E402
from ur_digital_m_target import offsets  # noqa: E402

checks = fails = 0


def check(ok, what):
    global checks, fails
    checks += 1
    if not ok:
        fails += 1
        print("FAIL:", what)


# ---------------------------------------------------------------------------------------------- the header's values
H = {}
HDR = (FW / "src/core/ultra_protocol.h").read_text(encoding="utf-8")
for line in HDR.splitlines():
    m = re.match(r"\s*#define\s+(ULTRA_\w+)\s+\(?(-?(?:0x[0-9A-Fa-f]+|\d+))u?\)?(?:\s|$)", line)
    if m:
        H[m.group(1)] = int(m.group(2), 0)
for m in re.finditer(r"\b(ULTRA_\w+)\s*=\s*(0x[0-9A-Fa-f]+|\d+)\s*[,}]", HDR):   # the enums' members
    H[m.group(1)] = int(m.group(2), 0)


def crc16(b):
    c = 0xFFFF
    for x in b:
        c ^= x << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021 if c & 0x8000 else c << 1) & 0xFFFF
    return c


def crc8(b):
    c = 0
    for x in b:
        c ^= x
        for _ in range(8):
            c = ((c << 1) ^ 0x07 if c & 0x80 else c << 1) & 0xFF
    return c


def frame(slot_type, payload, seq, flags, host_us=123456):
    """one slot (light 0xF, the board) + the header: 40 bytes"""
    s = bytearray(H["ULTRA_SLOT_BYTES"])
    s[0], s[1], s[2] = slot_type, (0 if slot_type in (H["ULTRA_CONFIG"],H["ULTRA_NOP"]) else 15 if slot_type==H["ULTRA_PIXEL_FX"] else 255), seq
    s[H["ULTRA_PAYLOAD"]:H["ULTRA_PAYLOAD"] + len(payload)] = payload
    s[30:32] = crc16(s[:30]).to_bytes(2, "big")
    h = bytearray(H["ULTRA_MAGIC"].to_bytes(2, "big") + bytes([H["ULTRA_VERSION"] << 4 | flags]) +
                  host_us.to_bytes(4, "big"))
    h.append(crc8(h))
    return bytes(s + h)


def poll(page, arg=b"", rate=0, seq=0x31, index=0, flags=None):
    p = bytes([index, 1, page, rate]) + arg     # slot bytes 3..: index, R, page, rate, arguments (byte 7 on)
    return frame(H["ULTRA_POLL"], p, seq, H["ULTRA_HDR_RW"] if flags is None else flags)


# ---------------------------------------------------------------------------------------------- the state's offsets
def every_symbol(elf, name):
    """addresses of every symbol called name (local statics of several files share names)"""
    d = open(elf, "rb").read()
    shoff, = struct.unpack_from("<I", d, 32)
    shentsize, shnum = struct.unpack_from("<HH", d, 46)
    secs = [struct.unpack_from("<IIIIIIIIII", d, shoff + i * shentsize) for i in range(shnum)]
    out = []
    for s in secs:
        if s[1] == 2:
            strtab = secs[s[6]][4]
            for k in range(s[5] // 16):
                st_name, st_value = struct.unpack_from("<II", d, s[4] + k * 16)
                if d[strtab + st_name:d.index(b"\0", strtab + st_name)] == name.encode():
                    out.append(st_value)
    return out


# ---------------------------------------------------------------------------------------------- the board's state
elf = sys.argv[1] if len(sys.argv) > 1 else str(FW / "build/ur_digital_m-api/fw.elf")
O = offsets(elf)
e = Emu(elf)
A = e.addr
w = e.wr

FW_WORD = 0x000701FF                           # 0.7.1, a release (pre-release 0xFF)
UP = (1 << 24) + 5                             # STATUS's u24 uptime wraps
UID = bytes(range(0x11, 0x1D))
LT = dict(on_s=3600 * 1234 + 359, lit_s=3600 * 456 + 1, ups=70000, dips=0x123456, writes=0x00ABCDEF)
w(A("fw_version"), 4, FW_WORD)
w(A("up_s"), 4, UP)
for a in every_symbol(elf, "reset_flags"):
    w(a, 4, 0x0C000000)                        # power-on + pin: STATUS 7 = 0x03
upd = A("upd") + O["upd_info"]
w(upd + O["info_hw_id"], 1, 3)
w(upd + O["info_hw_rev"], 1, 0)
w(upd + O["info_bl"], 1, 2)
lt = A("en") + O["en_lt"]
for k, v in LT.items():
    w(lt + O[k], 4, v)
w(lt + O["trips"], 2, 3)
w(lt + O["trips"] + 2, 2, 65535)
w(lt + O["updates"], 2, 2)
w(lt + O["wd"], 1, 255)
w(lt + O["peak_t"], 1, 150)
w(lt + O["peak_a"], 1, 42)
w(A("vs_bus") + O["vs_valid"], 1, 1)
w(A("vs_bus") + O["vs_mv"], 4, 24123)
w(A("mt") + O["mt_c10"], 2, (-125) & 0xFFFF)  # -12.5 C
w(A("mt") + O["mt_vdda"], 2, 3301)
w(A("i_avg"), 4, 1234)
w(A("rc_uid16"), 2, 0xBEEF)
st = A("dl") + O["dl_st"]
w(st + O["st_frames"], 4, 0x01234567)
w(st + O["st_bits_last"], 4, 14400)
w(st + O["st_bits_min"], 4, 0xFFFFFFFF)
w(st + O["st_bits_max"], 4, 65535)
w(st + O["st_long_high"], 2, 3)
w(st + O["st_answered"], 2, 7)
w(st + O["st_marks"], 2, 11)
w(st + O["st_cap_drops"], 1, 2)
w(A("dl") + O["dl_wpos"], 4, 2250 * 5 + 7)  # 5 ms since the last edge
w(A("dl") + O["dl_edge_w"], 4, 7)
w(A("dl") + O["dl_direct"], 1, 0)
uds = A("uds")
w(uds + O["uds_fps10"], 2, 600)
w(uds + O["uds_faults"], 1, 2)
w(uds + O["uds_polls"], 2, 25)
for i, b in enumerate(UID):
    e.mmio[0x1FFFF7E8 + i] = b

FBUF, ANS = A("ud_fbuf"), A("ud_ans")
tick = [1000]


def run(f, nbits=320):
    e.poke(FBUF, f)
    e.poke(ANS, bytes(32))
    tick[0] += 20 * 50                          # 50 ms of 20 kHz ticks between frames
    e.call("ud_frame", [FBUF, nbits, tick[0]])
    return e.ret, e.peek(ANS, 32)


def u(b, o, n, signed=False):
    return int.from_bytes(b[o:o + n], "big", signed=signed)


def answer(page, arg=b"", rate=0, expect=None, seq=0x31):
    """a POLL answered: the framing checked; returns DATA"""
    what = "page 0x%02X arg %s rate %d" % (page, arg[:1].hex() or "-", rate)
    r, a = run(poll(page, arg, rate, seq))
    check(r == 1, what + ": ud_frame %r (1 = an answer armed)" % r)
    check(a[0] == H["ULTRA_RSP_SYNC"] and u(a, 30, 2) == crc16(a[:30]), what + ": SYNC / CRC")
    check(a[1] == (page if expect is None else expect), what + ": answered page 0x%02X" % a[1])
    check(a[2] >> 4 == 0 and a[3] == seq and u(a, 4, 2) == 0xBEEF, what + ": index / SEQ / UID16")
    return a[H["ULTRA_RSP_DATA"]:H["ULTRA_RSP_DATA"] + H["ULTRA_RSP_DATA_LEN"]], e.rd(A("ud_brr"), 2)


# Wire-3 pages: actual dispatcher, UART framing, banks, and descriptor bytes.
ux=A("ux");e.call("udfx_init",[ux])
brr=[]
for rate in range(3):
    d,div=answer(H["ULTRA_PAGE_STATUS"],rate=rate);brr.append(div)
    check(u(d,0,3)==FW_WORD>>8 and d[3]==3 and u(d,4,3)==5,"STATUS identity/uptime")
    check(d[H["ULTRA_ST_STATE"]]&0x0F==0,"STATUS 8 bits 0-3: 0 (no POWER events)")
    check(u(d,13,2)==2412 and u(d,17,2)==1234,"STATUS 10mV and mA units")
    d,_=answer(H["ULTRA_PAGE_LIFETIME"],rate=rate)
    check(u(d,0,3)==LT['on_s']//360 and d[17]==42,"LIFETIME 0.5A peak")
desc=e.peek(A('board_descriptor'),e.sym('board_descriptor')[1])
assembled=bytearray()
for part in range((len(desc)+23)//24):
    d,_=answer(H['ULTRA_PAGE_DESCRIPTOR'],bytes([part]));assembled.extend(d)
check(assembled[:len(desc)]==desc and not any(assembled[len(desc):]),'descriptor paging and zero padding')
answer(H['ULTRA_PAGE_DESCRIPTOR'],bytes([(len(desc)+23)//24]),expect=255)
d,_=answer(H['ULTRA_PAGE_IDENTITY'])
check(d[:12]==UID and d[19]==3 and u(d,20,2)==len(desc) and u(d,22,2)==crc16(desc[6:]),'IDENTITY wire/descriptor length/CRC')
d,_=answer(H['ULTRA_PAGE_IDENTITY'],bytes([1]))
FEAT=u(d,0,4)
check(not FEAT&(H['ULTRA_FEAT_POWER_EVENTS']|H['ULTRA_FEAT_SUPPLY_DIPS']|H['ULTRA_FEAT_DIAG']) and
      FEAT&H['ULTRA_FEAT_ENERGY'] and FEAT&H['ULTRA_FEAT_LIFETIME'] and not any(d[4:]),
      'IDENTITY part 1: the V203 binding (lean_telemetry): ENERGY + LIFETIME, no POWER events, SUPPLY or DIAG (0x%08X)'%FEAT)
answer(H['ULTRA_PAGE_IDENTITY'],bytes([2]),expect=255) # no immutable boot info fixture yet
for page in (0x01,0x03,0x05,0x07,0x0A,0x12,0x80,0x81,0x82):answer(page,expect=255)
answer(H['ULTRA_PAGE_LINK_STATS'])
# the V203 binding (board.json lean_telemetry): no POWER, SUPPLY or DIAG page; the supply impedance on ENERGY
for page in (H['ULTRA_PAGE_POWER'],H['ULTRA_PAGE_SUPPLY'],H['ULTRA_PAGE_DIAG']):
    for part in (0,1):answer(page,bytes([part]),expect=255)
vd=A("vdip")
for znum,zden,steps,want,what in ((1030,2500,4,1030*125//(2500>>3),"4 steps"),(1030,2500,3,0,"3 steps: not known yet"),
                                  (600000,8,9,0xFFFF,"saturated"),(-50,2500,9,0,"negative: 0")):
    w(vd+O["vdip_znum"],4,znum&0xFFFFFFFF);w(vd+O["vdip_zden"],4,zden);w(vd+O["vdip_zsteps"],2,steps)
    d,_=answer(H['ULTRA_PAGE_ENERGY'])
    check(u(d,H['ULTRA_EN_SUPPLY_Z'],2)==want and not any(d[21:24]),
          "ENERGY supply impedance, %s: %d mOhm (want %d)"%(what,u(d,H['ULTRA_EN_SUPPLY_Z'],2),want))
# the dips are still detected on the lean board: a log of the newest dip (the fault memory's, STATUS's supply reason)
check(O["sz_vdip_log"]==12,"lean: a dip log of one (%d B)"%O["sz_vdip_log"])
e.poke(vd,bytes(e.sym("vdip")[1]))
w(A("up_s"),4,100)
now=[5000]
def dip_ms(level,n):
    for _ in range(n):
        e.call("vdip_ms",[vd,now[0],e.rd(A("up_s"),4),level,1,1500]);now[0]+=1
dip_ms(2000,300)                                # the hold-off, then the reference 2000
dip_ms(1700,8)                                  # 85 %: a dip from the 3rd ms
inside=e.rd(vd+O["vdip_in_dip"],1)
dip_ms(2000,5)
check(inside==1 and e.rd(vd+O["vdip_in_dip"],1)==0 and e.rd(vd+O["vdip_count"],2)==1 and e.rd(vd+O["vdip_n_log"],1)==1 and
      e.rd(vd+O["vdip_log"],4)==100 and e.rd(vd+O["vdip_log"]+4,2)==1700,
      "lean: a dip detected, counted and kept (in %d, count %d, logged %d)"%(inside,e.rd(vd+O["vdip_count"],2),e.rd(vd+O["vdip_n_log"],1)))
d,_=answer(H["ULTRA_PAGE_STATUS"])
check(d[H["ULTRA_ST_STATE"]]>>4==H["ULTRA_ST_WHY_SUPPLY"],"lean: STATUS's reason a supply dip within 30 s (%d)"%(d[8]>>4))
w(A("up_s"),4,131)
d,_=answer(H["ULTRA_PAGE_STATUS"])
check(d[H["ULTRA_ST_STATE"]]>>4==H["ULTRA_ST_WHY_NONE"],"lean: no reason 31 s after it (%d)"%(d[8]>>4))
w(A("up_s"),4,UP)
for part in (0,1,2): answer(0x06,bytes([part,1]))
answer(0x06,bytes([0,0]),expect=255)
for part in (0,1):answer(0x10,bytes([part]))
answer(0x10,bytes([2]),expect=255)
for part in (0,1,2):answer(0x11,bytes([part]))
answer(0x11,bytes([0,1]),expect=255)
# Output settings and explicit stream route. Unsupported output and ingress stay unchanged.
cfg=b''.join(bytes([cid])+val.to_bytes(2,'big') for cid,val in ((64,3),(66,40),(67,0),(68,20),(75,1),(77,1)))
r,_=run(frame(H['ULTRA_CONFIG'],cfg,9,0))
e.call('udfx_service',[ux,A('dl'),5,0,256])
vals=[e.rd(ux+O['ux_cfg']+2*k,2) for k in range(14)]
check(vals[0]==3 and vals[2]==40 and vals[4]==20 and vals[11]==1 and vals[13]==1,'scoped CONFIG 64..77')
d,_=answer(0x10,bytes([1]));check(d[0]==3 and d[1]==4 and u(d,4,2)==40 and d[8]==1,'PIXEL output config')
fx=bytearray(H['ULTRA_PX_CUSTOM2']+1-H['ULTRA_PAYLOAD']);fx[0]=3
fx[H['ULTRA_PX_BRIGHTNESS']-3]=200;fx[H['ULTRA_PX_FLAGS']-3]=H['ULTRA_PXF_STORE']
run(frame(H['ULTRA_PIXEL_FX'],fx,4,0))
e.wr(ux+O['ux_local'],1,1)
for _ in range(10):e.call('udfx_render',[ux])
seg0=ux+O['ux_seg'];check(e.rd(seg0+O['seg_run'],1)==1,'PIXEL_FX dispatch')
d,_=answer(0x02,bytes([11,11,11]));check(d[:18]==bytes(18) and u(d,18,3)==0x3FFFF,'PIXEL_FX ACKS always NEVER')
d,_=answer(0x11);check(d[13:17]==bytes([4]*4),'whole-output segment ACKs')
fx[0]=0;run(frame(H['ULTRA_PIXEL_FX'],fx,5,0))
e.call('ux_frame_begin',[ux]) # consumer has reached the next complete frame boundary
check(e.rd(ux+O['ux_want_stream'],1)==1 and e.rd(ux+O['ux_stored_have'],1)==0,'STOP returns configured stream')
d,_=answer(0x11);check(d[13:17]==bytes([5]*4),'whole-output STOP updates all segment SEQs')

# LIFE_SYNC: the ledger 2 h and 5 power-ups ahead; a wrong UID16, a wrong RECORDS byte, the merge, then once only
lt16 = bytearray(16)
lt16[0:3] = (LT["on_s"] // 360 + 20).to_bytes(3, "big")
lt16[3:6] = (LT["lit_s"] // 360).to_bytes(3, "big")
lt16[6:9] = (70005).to_bytes(3, "big")
lt16[9:11] = (3).to_bytes(2, "big")
lt16[11:13] = (65535).to_bytes(2, "big")
lt16[13:16] = (0x123456).to_bytes(3, "big")
# the energy step from the descriptor the board serves (domain 0: rating mA x maximum voltage 10 mV, 0xFFFF unknown):
# ceil(P x (the powered 0.1 h raised + 10) / 10^6) Wh; unknown: the energy never merges
P = 0
k = 11
while k < len(desc):
    if desc[k] == 2 and desc[k + 2] >> 4 == 0:
        vmax, ma = u(desc, k + 2 + 4, 2), u(desc, k + 2 + 6, 2)
        P = 0 if 65535 in (vmax, ma) else vmax * ma
    k += 2 + desc[k + 1]
def ls_bound(dt):
    return (P * (dt + 10) + 999999) // 1000000
own16 = answer(H["ULTRA_PAGE_LIFETIME"])[0][:16]
wh0 = u(answer(H["ULTRA_PAGE_LIFE_SYNC"], (0x1234).to_bytes(2, "big") + bytes(own16) + bytes(5))[0], H["ULTRA_LS_A_TOTAL_WH"], 4)
check(P == 2250 * 5500 and ls_bound(0) == 124 and ls_bound(20) == 372,
      "LIFE_SYNC energy step from the served descriptor: %d x 10 uW, %d Wh (+0 h), %d Wh (+2 h)" % (P, ls_bound(0), ls_bound(20)))
for uid16, rec, lts, wh, want, flags in (
        (0x1234, 0xEF, lt16, wh0 + 1, "ULTRA_LS_R_UID", 0), (0xBEEF, 0x00, lt16, wh0 + 1, "ULTRA_LS_R_RECORDS", 0),
        (0xBEEF, 0xEF, own16, wh0 + ls_bound(0) + 1, "ULTRA_LS_R_CAPPED", 0),   # (none applied: the merge not used up)
        (0xBEEF, 0xEF, lt16, wh0 + ls_bound(20), "ULTRA_LS_R_APPLIED",
         H["ULTRA_LS_F_POWERED"] | H["ULTRA_LS_F_UPS"] | H["ULTRA_LS_F_WH"]),
        (0xBEEF, 0xEF, lt16, wh0 + ls_bound(20) + 1, "ULTRA_LS_R_ONCE", 0)):
    d, _ = answer(H["ULTRA_PAGE_LIFE_SYNC"], uid16.to_bytes(2, "big") + bytes(lts) + wh.to_bytes(4, "big") + bytes([rec]))
    check(d[H["ULTRA_LS_A_RESULT"]] == H[want] and d[H["ULTRA_LS_A_APPLIED"]] == flags and
          u(d, H["ULTRA_LS_A_TOTAL_WH"], 4) == (wh0 + ls_bound(20) if want in ("ULTRA_LS_R_APPLIED", "ULTRA_LS_R_ONCE")
                                                else wh0),
          "LIFE_SYNC %s: result %d applied 0x%02X, %d Wh (own %d)" % (want, d[20], d[21], u(d, 16, 4), wh0))
check(e.rd(lt + O["ups"], 4) == 70005 and e.rd(lt + O["on_s"], 4) // 360 == LT["on_s"] // 360 + 20,
      "LIFE_SYNC: merged into the record")

# not for this board: RELAY_ONLY, another index, a frame without RW, a bad slot CRC, another length
r, _ = run(poll(H["ULTRA_PAGE_STATUS"], index=H["ULTRA_POLL_RELAY_ONLY"]))
check(r == 0, "RELAY_ONLY: no answer")
r, _ = run(poll(H["ULTRA_PAGE_STATUS"], index=1))
check(r == 0, "index 1: no answer (one receiver per port)")
r, _ = run(poll(H["ULTRA_PAGE_STATUS"], flags=0))
check(r == 0, "a POLL slot without RW: no answer")
f = bytearray(poll(H["ULTRA_PAGE_STATUS"]))
f[10] ^= 1
derr = e.rd(uds + O["uds_dec_err"], 2)
r, _ = run(bytes(f))
check(r == 0 and e.rd(uds + O["uds_dec_err"], 2) == derr + 1, "a bad slot CRC: no answer, a decode error")


# the capture path (ud_decode): the bits counted to the frame's real end, exactly 320 or 384
def wave(data, nbits):
    s = [0] * 360                               # 10 us LOW after the marker's falling edge
    for i in range(nbits):
        hi = 25 if data[i // 8] & (0x80 >> (i % 8)) else 9
        s += [1] * hi + [0] * (48 - hi)
    s += [0] * 1664                             # the frame end's 46 us LOW
    s += [0] * (-len(s) % 16)
    return [int("".join(map(str, s[k:k + 16])), 2) for k in range(0, len(s), 16)]


def decode(data, nbits):
    words = wave(data, nbits)
    cap = A("dl_cap")
    e.poke(cap, struct.pack("<%dH" % len(words), *words))
    tick[0] += 20 * 50
    e.call("ud_decode", [cap, len(words), tick[0]], max_steps=5_000_000)
    return e.ret


blk = bytearray(40)                             # an UPD PING: the 40-byte block (ultra_update.h) + its header
blk[0] = 0x85
blk[38:40] = crc16(blk[:38]).to_bytes(2, "big")
hdr = bytearray(H["ULTRA_MAGIC"].to_bytes(2, "big") + bytes([3 << 4 | H["ULTRA_HDR_UPD"]]) + (777).to_bytes(4, "big"))
hdr.append(crc8(hdr))
upd_f = bytes(blk) + bytes(hdr)
n0 = e.rd(A("upd") + O["upd_frames"], 4)
check(decode(poll(H["ULTRA_PAGE_STATUS"]), 320) == 1, "ud_decode: a 320-bit POLL answered")
check(decode(upd_f, 384) == 0 and e.rd(A("upd") + O["upd_frames"], 4) == n0 + 1, "ud_decode: a 384-bit UPD frame taken")
for n, f in ((321, poll(H["ULTRA_PAGE_STATUS"]) + bytes(8)), (328, poll(H["ULTRA_PAGE_STATUS"]) + bytes(8)),
             (383, upd_f), (385, upd_f + bytes(1))):
    derr = e.rd(uds + O["uds_dec_err"], 2)
    check(decode(f, n) == 0 and e.rd(uds + O["uds_dec_err"], 2) == derr + 1 and
          e.rd(A("upd") + O["upd_frames"], 4) == n0 + 1, "ud_decode: %d bits refused" % n)


# ---------------------------------------------------------------------------------------------- CHAIN frames (ud_chain_decode)
DL = A("dl")
e.poke(DL + O["dl_cap"], struct.pack("<I", A("dl_cap")))
UC = A("uc")


def poll_r(index, r, page, rate, seq):
    return frame(H["ULTRA_POLL"], bytes([index, r, page, rate]), seq, H["ULTRA_HDR_RW"])


chain_n = [0]
decode_cycles = []


def chain_decode(data, bits, end_w, phase=5):
    # Feed the current acquisition mailbox incrementally, exactly as the
    # scanner publishes its raw ring to udc_stream; the old linear head and
    # snapshot layout is no longer a firmware input.
    samples = [0] * phase
    for i in range(bits):
        hi = 34 if data[i // 8] & (0x80 >> (i % 8)) else 11
        samples += [1] * hi + [0] * (45-hi)
    samples += [0] * (160 + (-len(samples) % 16))
    words = [int("".join(map(str, samples[i:i+16])), 2) for i in range(0,len(samples),16)]
    cap = A("dl_cap")
    js = 10000 + chain_n[0] * 4096
    w(DL + O["dl_ch_cjs"], 4, js)
    w(DL + O["dl_ch_profile"], 1, 255)
    for start in range(0, len(words), 128):
        stop = min(start+128, len(words))
        for i in range(start, stop):
            w(cap + 2*((js-1+i)&(O['dl_ch_tail']-1)), 2, words[i])
        w(DL + O["dl_ch_hn"], 2, stop)
        e.call("udc_stream", [DL, A("ucp")])
    w(DL + O["dl_ch_bits"], 2, bits)
    w(DL + O["dl_ch_end"], 4, end_w)
    w(DL + O["dl_ch_ready"], 1, 1)
    w(A("dl_ch_ready"), 1, 1)
    w(A("dl_ch_tick"), 4, 1000 + 1000 * chain_n[0])
    w(A("ms_now"), 4, 50 * chain_n[0])
    chain_n[0] += 1
    e.poke(ANS, bytes(32))
    e.call("ud_chain_decode", [], max_steps=8_000_000)
    decode_cycles.append(e.cycles())


def ucv(k, n=2):
    return e.rd(UC + O[k], n)


e.call("udfx_init", [ux])
e.poke(UC, bytes(e.sym("uc")[1]))               # the line mode state zero: AUTO, undecided
e.poke(DL + O["dl_ch_req"], bytes(1))
h0 = e.rd(uds + O["uds_headers"], 2)
d0 = e.rd(uds + O["uds_decoded"], 2)
t0 = e.rd(uds + O["uds_tail_err"], 2)
nop = frame(H["ULTRA_NOP"], b"", 1, 0, 10000)
chain_decode(nop, 320, 120000)
check(ucv("uc_frames") == 1 and ucv("uc_mode", 1) == 0 and e.rd(DL + O["dl_ch_ready"], 1) == 0 and e.rd(DL + O["dl_ch_arm"], 1) == 0 and
      e.rd(uds + O["uds_headers"], 2) == h0 and e.rd(uds + O["uds_decoded"], 2) == d0,
      "CHAIN: a NOP frame: counted, undecided after one, the capture released, no answer")
check(e.rd(DL + O["dl_ch_req"], 1) == 0 and e.rd(ux + O["ux_chain"], 1) == 0, "CHAIN: nothing started by one frame")
chain_decode(nop, 320, 130000)
check(ucv("uc_mode", 1) == 2 and e.rd(DL + O["dl_ch_req"], 1) == 5 and e.rd(ux + O["ux_chain"], 1) == 1,
      "CHAIN: the second Ultra frame decides CHAIN (dline_ch_start enter, the encoder asked for): mode %d req %d" %
      (ucv("uc_mode", 1), e.rd(DL + O["dl_ch_req"], 1)))
chain_decode(nop, 320, 135000)  # third CRC trailer qualifies command execution
chain_decode(frame(H["ULTRA_NOP"], b"", 1, 0, 10040) + bytes(85), 1000, 140000)
check(ucv("uc_nonultra") == 1 and e.rd(uds + O["uds_tail_err"], 2) == t0 + 1 and e.rd(DL + O["dl_ch_arm"], 1) == 0 and ucv("uc_mode", 1) == 2,
      "CHAIN: 1000 bits is no Ultra frame: counted, nothing armed")
bad = bytearray(nop)
bad[36] ^= 0x40
chain_decode(bytes(bad), 320, 150000)
check(ucv("uc_nonultra") == 2, "CHAIN: a broken header is no Ultra frame")

# a POLL for answer index 1 of 2 at 500 kbaud: the answer's start word and the window's end from t0 and the clock
e.poke(A("line_lock"), struct.pack("<IIBB", 200, 0xFFFFF800, 0, 3))  # prior 1 s qualification
w(ux + O["ux_r_have"], 1, 1)                    # the clock's rate is measured (an unmeasured one answers no POLL: below)
wpu24 = e.rd(ux + O["ux_wpu24"], 4)
known = e.rd(ux + O["ux_r_have"], 1)
pitch = 335 * 2 + 165
end_w = 200000
chain_decode(poll_r(1, 2, H["ULTRA_PAGE_STATUS"], 1, 0x37), 320, end_w)
a = e.peek(ANS, 32)
tx_us = 1000 + pitch * 1 + (0 if known else 10)
check(e.rd(DL + O["dl_ch_arm"], 1) == 1 and e.rd(DL + O["dl_ch_tx"], 4) == end_w + (tx_us * wpu24 >> 24) and
      e.rd(DL + O["dl_ch_win"], 4) == end_w + ((1000 + pitch * 2 + 200) * wpu24 >> 24),
      "CHAIN: a POLL for index 1 of 2: armed, starts at t0 + %d us (word %d), window to t0 + %d us" % (tx_us, e.rd(DL + O["dl_ch_tx"], 4) - end_w,
                                                                                                    1000 + pitch * 2 + 200))
check(a[0] == H["ULTRA_RSP_SYNC"] and a[1] == H["ULTRA_PAGE_STATUS"] and a[2] >> 4 == 1 and a[3] == 0x37 and u(a, 4, 2) == 0xBEEF and
      u(a, 30, 2) == crc16(a[:30]), "CHAIN: the answer built: SYNC / page / index 1 / SEQ / UID16 / CRC")
check(e.rd(A("ud_brr"), 2) == brr[1] and ucv("uc_windows") == 1, "CHAIN: the divider for 500 kbaud, one window counted")
# RELAY_ONLY: the window is flagged, nothing armed
chain_decode(poll_r(H["ULTRA_POLL_RELAY_ONLY"], 1, H["ULTRA_PAGE_STATUS"], 0, 0x38), 320, 210000)
check(e.rd(DL + O["dl_ch_arm"], 1) == 0 and e.rd(DL + O["dl_ch_win"], 4) == 210000 + ((1000 + 500 * 1 + 200) * wpu24 >> 24) and ucv("uc_windows") == 2,
      "CHAIN: RELAY_ONLY: a window, no answer")
# a frame without RW: a CONFIG slot is acted on (CONFIG 75 = 1 forces STREAM), the window stays 0
chain_decode(frame(H["ULTRA_CONFIG"], bytes([75, 0, 1]), 12, 0, 10100), 320, 220000)
check(e.rd(ux + O["ux_cfg"] + 2 * 11, 2) == 1 and e.rd(DL + O["dl_ch_win"], 4) == 0, "CHAIN: CONFIG 75 taken from a chain frame")
# a 3-slot frame: slot 0 acted on, counted as not the last board
three = frame(H["ULTRA_NOP"], b"", 2, 0, 10200)[:32] + frame(H["ULTRA_NOP"], b"", 3, 0)[:32] + frame(H["ULTRA_NOP"], b"", 4, 0)[:32] + \
    frame(H["ULTRA_NOP"], b"", 4, 0, 10200)[32:]
chain_decode(three, 3 * 256 + 64, 230000)
check(ucv("uc_longs") == 1 and ucv("uc_slots", 1) == 3 and ucv("uc_flags", 1) & H["ULTRA_PXL_F_LONG"], "CHAIN: a 3-slot frame is counted as not the last board")
# an UPD frame: 384 bits, header only
d, _ = answer(H["ULTRA_PAGE_PIXEL"], bytes([0]), 0)
check(d[H["ULTRA_PXL_MODE"]] == 2 and u(d, H["ULTRA_PXL_FRAMES"], 2) == ucv("uc_frames") and u(d, H["ULTRA_PXL_NONULTRA"], 2) == 2 and
      u(d, H["ULTRA_PXL_LONG"], 2) == 1 and u(d, H["ULTRA_PXL_WINDOWS"], 2) == 2 and d[H["ULTRA_PXL_FLAGS"]] & H["ULTRA_PXL_F_LONG"] and
      d[H["ULTRA_PXL_FLAGS"]] & H["ULTRA_PXL_F_NONULTRA"] and u(d, H["ULTRA_PXL_BITS"] , 3) == 832 and d[H["ULTRA_PXL_SLOTS"]] == 3,
      "CHAIN: page 0x81 part 3 shows it: mode CHAIN, frames %d, non-Ultra 2, long 1, windows 2, bits 832" % u(d, H["ULTRA_PXL_FRAMES"], 2))
# a POLL before the clock's rate is measured: the window is counted, the answer is not armed (the HSI would shift its start out of the window), a miss is counted
w(ux + O["ux_r_have"], 1, 0)
m0 = ucv("uc_missed")
chain_decode(poll_r(0, 1, H["ULTRA_PAGE_STATUS"], 0, 0x39), 320, 240000)
check(e.rd(DL + O["dl_ch_arm"], 1) == 0 and ucv("uc_missed") == m0 + 1 and ucv("uc_windows") == 3 and
      e.rd(DL + O["dl_ch_win"], 4) == 240000 + ((1000 + 500 * 1 + 200) * wpu24 >> 24),
      "CHAIN: a POLL before the rate is measured: window counted, no answer armed, counted as missed")
w(ux + O["ux_r_have"], 1, 1)


# ---------------------------------------------------------------------------------------------- the status LED
def led_levels(ms, step=10):
    """led_service for ms, in steps: [(t, level)] at each change of PB15 (GPIOB BSHR: bit 15 on, bit 31 off)"""
    out, t = [], 0
    for _ in range(ms // step):
        e.writes.clear()
        e.call("led_service", [step])
        t += step
        for reg, val in e.writes:
            if reg == 0x40010C10 and val & (1 << 15):
                out.append((t, 1))
            elif reg == 0x40010C10 and val & (1 << 31):
                out.append((t, 0))
    return out


w(A("led_lvl"), 1, 1)                           # (main sets it with the LED on)
f = frame(H["ULTRA_IDENTIFY"], bytes([H["ULTRA_IDENT_VERSION"]]), 0x40, 0)
r, _ = run(f)
check(r == 0 and e.rd(A("led_lead"), 2) == 1000, "IDENTIFY 5: the version's code starts (1 s off first)")
run(f)                                          # a copy: dropped (the same pattern runs on)
ch = led_levels(16000)
check(ch[:1] == [(10, 0)] and (1010, 1) in ch, "IDENTIFY 5: LED off for 1 s, then the pattern: %s" % ch[:3])
n_on = sum(1 for _, lv in ch if lv == 1)
check(n_on >= 3 + 1 + 1 + 2 and ch[-1][1] == 1 and e.rd(A("led_lead"), 2) == 0,
      "IDENTIFY 5: 0.7.1 shown (3 fast, 1, 2, 2 blinks), the LED on again: %d on-edges" % n_on)
# CHAIN: frames that are no Ultra frame (a pixel stream on a chain board) blink the status LED twice (a fault pattern, at most every 30 s)
w(A("led_lvl"), 1, 1)
e.poke(UC, bytes(e.sym("uc")[1]))
w(UC + O["uc_mode"], 1, 2)
w(UC + O["uc_nonultra"], 2, 5)
w(A("wrong_seen"), 2, 0)
w(A("wrong_s"), 4, 0)
w(A("up_s"), 4, 100)
w(A("led_lead"), 2, 0)
e.call("second_service", [], max_steps=2_000_000)
check(e.rd(A("led_lead"), 2) == 1000 and e.rd(A("wrong_seen"), 2) == 5, "CHAIN: non-Ultra frames start the LED's fault code (1 s off first)")
ch = led_levels(9000)
n_on = sum(1 for _, lv in ch if lv == 1)
check(ch[:1] == [(10, 0)] and n_on >= 4 and ch[-1][1] == 1, "CHAIN: the 2-blink fault code runs and ends with the LED on: %s" % ch[:8])
w(UC + O["uc_nonultra"], 2, 6)
w(A("up_s"), 4, 110)
e.call("second_service", [], max_steps=2_000_000)
check(e.rd(A("led_lead"), 2) == 0, "CHAIN: the code is not repeated within 30 s")
for act, name in ((3, "SAME"), (4, "NACK")):     # UPD_APP_SHOW_SAME / _REJECTED
    e.call("upd_action", [act])
    ch = led_levels(5000)
    check(ch[:2] == [(10, 0), (1010, 1)] and ch[-1][1] == 1 and len(ch) == 4,
          "an update's %s: 1 s off, one flash, on again: %s" % (name, ch))

print("a chain frame decoded in one go by ud_chain_decode (udc_classify: the rising-edge decode of the head and of the snapshot's tail, the CRCs, the slot's action, the answer built; the board spreads the head over the frame): %s cycles = %s us at 144 MHz"
      % (sorted(decode_cycles)[len(decode_cycles) // 2], sorted(decode_cycles)[len(decode_cycles) // 2] // 144))
# Every unsupported output/bank is answered without using bank-zero state.
for page in range(0x12):
    if page in (H['ULTRA_PAGE_POWER'], H['ULTRA_PAGE_LIFE_SYNC']): continue
    for bank in (1, 255):
        answer(page, bytes([0,0,0,bank]), expect=H['ULTRA_PAGE_UNSUPPORTED'])

print("ud_pages: %d checks, %d failed" % (checks, fails))
sys.exit(1 if fails else 0)
