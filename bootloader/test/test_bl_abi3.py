# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""ABI-3 production ELF tests. No legacy descriptors or synthetic engine clone."""
import argparse
import json
from pathlib import Path
import struct
import subprocess
import sys

from abi3_model import APP, BODY, END, FRAME, WORK, MCU, WireMCU, PowerCut, Reset, crc32, frame, image, reseal, seal
from rv32emu import Emu

ROOT = Path(__file__).resolve().parents[2]
CHECKS = 0


def check(condition, label):
    global CHECKS
    CHECKS += 1
    assert condition, label


def fresh(elf, installed=True):
    m = MCU(elf)
    old = image(m.target, 772, 0x00030000)
    if installed:
        m.installed(old)
    m.init_session()
    return m, old


def committed(m, img):
    return m.peek(APP, 64) == img[:64] and m.peek(BODY, len(img)-64) == img[64:]


def recover(m, img):
    new, _ = fresh(m.elf, False)
    new.flash[:] = m.flash
    new.init_session()
    new.start(img, 19)
    new.transfer(img, 19)
    check(committed(new, img), "fresh ANNOUNCE recovers retained flash")


def transactions(elf):
    m, old = fresh(elf)
    img = image(m.target)
    check(m.boot_path() == "app", "confirmed old app boot path")
    m.start(img)
    check(m.state == 1, "cursor reaches RECEIVE")
    check([a for k, a, _ in m.ops if k == "erase"] == list(range(APP, BODY+1024, 256)),
          "minimum-unit cursor covers metadata and complete rounded body")
    check(m.boot_path() == "bootloader", "revoked image cannot run")
    m.transfer(img)
    check(committed(m, img), "complete body then metadata")
    check(m.peek(APP+64, 2) == b"\xb7\xa5", "VERIFIED readback")
    check(m.ops[-1][0:2] == ("half", APP), "descriptor magic is the last mutation")
    operation_count = len(m.ops)
    m.send(frame(0x83, data=img[24:28]+img[20:24]+bytes(24)))
    check(len(m.ops) == operation_count, "duplicate END idempotent")

    # Rejection before revoke: exact tuple, revision, boot version, bounds,
    # reserved fields, malformed flags/session and both independent CRCs.
    for position, value in ((8, 254), (9, 4), (10, 5), (11, 4), (12, 2),
                            (14, 255), (6, 17), (7, 0), (15, 4), (13, 1),
                            (4, 1), (5, 1), (39, 1), (56, 1)):
        m, old = fresh(elf)
        d = bytearray(img[:64]); d[position] = value; d = seal(d)
        before = bytes(m.flash)
        m.start(d+img[64:])
        check(bytes(m.flash) == before and not m.ops, f"descriptor reject byte {position}")
    for offset, value in ((20, 0), (20, 3), (20, 57092), (20, 0xfffffffc),
                          (28, 0x08000000), (32, 0x08002101), (32, END)):
        m, old = fresh(elf)
        d = bytearray(img[:64]); struct.pack_into(">I", d, offset, value); d = seal(d)
        before = bytes(m.flash)
        m.start(d+img[64:])
        check(bytes(m.flash) == before and not m.ops, f"descriptor bounds {offset}/{value}")
    for session, flags, pass_no in ((0, 0, 0), (7, 2, 0), (7, 0, 1)):
        m, old = fresh(elf); before = bytes(m.flash)
        for half in (0, 1):
            m.send(frame(0x81, session, half, img[half*32:(half+1)*32], flags, pass_no))
        check(bytes(m.flash) == before and not m.ops, "semantic ANNOUNCE reject")
    for supply, protection in ((False, True), (True, False)):
        m, old = fresh(elf); before = bytes(m.flash)
        if not supply: m.mmio[0x40007004] = 4
        if not protection: m.mmio[0x40022020] = 0xffffffff
        m.start(img)
        check(bytes(m.flash) == before and not m.ops, "supply/protection reject")

    # Missing chunks survive END and later passes repair them.
    m, _ = fresh(elf); m.start(img); m.transfer(img, skip=(2, 9))
    check(not committed(m, img), "missing chunks never commit")
    m.transfer(img)
    check(committed(m, img), "later full pass completes missing chunks")
    # Identical programmed repeats do not erase/program again; conflicts fail.
    m, _ = fresh(elf); m.start(img)
    for k in range(8):
        m.send(frame(0x82, index=k, data=img[64+32*k:96+32*k]))
    ops = len(m.ops)
    m.send(frame(0x82, index=0, data=img[64:96]))
    check(len(m.ops) == ops, "identical programmed repeat")
    m.send(frame(0x82, index=0, data=bytes(32)))
    check(m.state == 4, "conflicting programmed repeat fails session")
    recover(m, img)
    m, _ = fresh(elf)
    m.send(frame(0x81, index=0, data=img[:32]))
    m.send(frame(0x81, index=0, data=bytes(32)))
    m.send(frame(0x81, index=1, data=img[32:64]))
    check(not m.ops, "conflicting ANNOUNCE never revokes")
    m, _ = fresh(elf); m.start(img)
    m.send(frame(0x82, index=0, data=img[64:96]))
    m.send(frame(0x82, index=0, data=bytes(32)))
    check(m.state == 4, "conflicting RAM chunk fails session")
    recover(m, img)

    # Stale and malformed traffic cannot touch flash or complete a transfer.
    m, _ = fresh(elf); m.start(img); before = bytes(m.flash); ops = len(m.ops)
    controls = [frame(0x84, 8), frame(0x82, 8, 0), frame(0x82, 7, 65535),
                frame(0x83, 7, data=bytes(32)), frame(0x86, 7), frame(0x87, 7)]
    for f in controls: m.send(f)
    bad = bytearray(frame(0x82)); bad[38] ^= 1; m.send(bad)
    bad = bytearray(frame(0x82)); bad[47] ^= 1; m.send(bad)
    check(bytes(m.flash) == before and len(m.ops) == ops, "stale/malformed controls")
    m.transfer(img); check(committed(m, img), "valid stream after malformed frames")

    # END is required; inactivity may never commit complete pages on its own.
    m, _ = fresh(elf); m.start(img)
    for k in range((len(img)-64+31)//32):
        m.send(frame(0x82, index=k, data=img[64+32*k:96+32*k].ljust(32, b"\0")))
    m.advance(m.now_us+10001000); m.poll()
    check(m.state == 4 and not committed(m, img), "timeout is not END")
    recover(m, img)
    return img


def interruption(elf, img):
    # Every individual flash mutation, including each metadata halfword.
    probe, _ = fresh(elf); probe.start(img); probe.transfer(img)
    for fraction in (0.0, 0.5, 1.0):
        for operation in range(1, len(probe.ops)+1):
            m, old = fresh(elf)
            m.cut_at, m.cut_fraction = operation, fraction
            try:
                m.start(img); m.transfer(img)
                raise AssertionError("cut was not reached")
            except PowerCut:
                pass
            if m.boot_path() == "app":
                check(committed(m, old) or committed(m, img), "only complete image may run after cut")
            else:
                check(True, "interrupted image stays in bootloader")
            recover(m, img)
    # Before announce, half announce, after revoke, during cursor erase,
    # after erase, partial RAM, programmed page and body-complete before END.
    for phase in range(8):
        m, old = fresh(elf)
        if phase:
            m.send(frame(0x81, index=0, data=img[:32]))
        if phase >= 2:
            m.send(frame(0x81, index=1, data=img[32:64]))
        if phase >= 3: m.poll()
        if phase >= 4: m.drain_erase()
        chunks = 1 if phase == 5 else 8 if phase == 6 else (len(img)-64+31)//32 if phase == 7 else 0
        for k in range(chunks):
            m.send(frame(0x82, index=k, data=img[64+32*k:96+32*k].ljust(32, b"\0")))
        m.send(frame(0x84))
        check(not committed(m, img), f"abort phase {phase} cannot commit")
        if m.boot_path() == "app": check(committed(m, old), "abort preserves old app only before revoke")
        recover(m, img)


def faults(elf, img):
    for mode in ("eop", "wrp", "stuck"):
        for op in (1, 2, 4, 7, 10):
            m, _ = fresh(elf)
            m.fault, m.fail_at = mode, op
            try:
                m.start(img); m.transfer(img)
            except Reset:
                check(mode == "stuck", "only stuck deadline resets")
            check(not committed(m, img), f"flash failure {mode}/{op} cannot commit")
            recover(m, img)
    for which, duration in (("erase_us", 41000), ("program_us", 21000), ("half_us", 21000)):
        m, _ = fresh(elf); setattr(m, which, duration)
        try: m.start(img); m.transfer(img)
        except Reset: pass
        check(not committed(m, img), f"{which} over declared deadline")
        recover(m, img)
    m, _ = fresh(elf); m.start(img)
    for k in range((len(img)-64+31)//32):
        m.send(frame(0x82, index=k, data=img[64+32*k:96+32*k].ljust(32, b"\0")))
    m.flash[BODY-0x08000000+17] ^= 1
    m.send(frame(0x83, data=img[24:28]+img[20:24]+bytes(24)))
    check(not committed(m, img) and m.state == 4, "full-body CRC precedes metadata")
    recover(m, img)


def decode(m, f, clock=40000000, bit=50, zero=12, one=37, phase=0.0, marker=False):
    samples = []
    edge = phase
    def run(level, ticks):
        nonlocal edge
        start = int(edge)
        edge += ticks*36000000/clock
        samples.extend([level]*(int(edge)-start))
    run(0, clock//2000)
    if marker:
        run(1, clock//50000); run(0, clock//50000)
    for byte in f:
        for shift in range(7, -1, -1):
            high = one if byte & (1 << shift) else zero
            run(1, high); run(0, bit-high)
    run(0, clock//2000)
    samples.extend([0]*((-len(samples)) % 16))
    words = [sum(samples[k+j] << (15-j) for j in range(16)) for k in range(0, len(samples), 16)]
    m.poke(WORK, struct.pack("<HHI", 22, 160, 3600))
    m.call("bl_rx_init", [m.addr("rx"), WORK])
    found = []
    peak = 0
    for k in range(0, len(words), 128):
        batch = words[k:k+128]
        m.poke(m.addr("hw_ring"), struct.pack("<"+"H"*len(batch), *batch))
        m.call("bl_rx_feed", [m.addr("rx"), m.addr("hw_ring"), len(batch), FRAME])
        peak = max(peak, m.cycles()/len(batch))
        if m.ret:
            result = m.peek(FRAME, 56)
            found.append((int.from_bytes(result[:4], "little"), result[4], result[8:56]))
    check(len(found) == 1 and found[0] == (384, 0, f), "production decoder frame equality")
    return peak


def senders(elf, full_slot=False, wire=False, line_profile=None):
    sender_root = ROOT / "build/bl-abi3/senders"
    folder = ROOT / "build/bl-abi3/streams"
    if line_profile is not None:
        folder /= f"profile-{line_profile}"
    folder.mkdir(parents=True, exist_ok=True)
    m, _ = fresh(elf)
    length = 57088 if full_slot else 772
    img = image(m.target, length)
    image_file = folder / f"board{m.target[0]}-{length}.bin"
    image_file.write_bytes(img)
    stats = []
    for kind in ("c99", "esphome"):
        for profile in range(4) if line_profile is None else (line_profile,):
            m, _ = fresh(elf)
            command = [str(sender_root / kind / "trace.exe"), str(image_file),
                       str(Path(elf).with_suffix(".bin")), str(profile), "3", "7"]
            raw = subprocess.check_output(command, text=True)
            lines = raw.splitlines()
            if wire:
                receiver = WireMCU(elf, lines, marker=m.target[0] == 3, drop=(2, 9))
                receiver.installed(image(receiver.target, 772, 0x00030000))
                receiver.run()
                check(committed(receiver, img), f"production main {kind} profile {profile}")
                check(all(APP <= a < END for _, a, _ in receiver.ops), "whole-MCU flash bounds")
                check(receiver.peek(0x20000004, 4) == b"TOOB", "commit requests verified reboot")
                stats.append(dict(sender=kind, profile=profile, frames=len(lines), body_bytes=length,
                                  production_main=True, elapsed_us=round(receiver.now_us)))
                print(json.dumps(stats[-1]), flush=True)
                continue
            decoded = 0
            peak = 0
            for row in lines:
                now, clock, period, zero, one, hex_frame = row.split()
                f = bytes.fromhex(hex_frame)
                now, clock, period, zero, one = map(int, (now, clock, period, zero, one))
                edge_us = (383*period+(one if f[-1]&1 else zero))*1000000/clock
                if now < m.now_us:
                    continue  # Receiver is deaf during an advertised flash operation.
                # Lose first-pass chunks. The sender's second pass must repair them.
                if f[0] == 0x82 and f[4] >> 4 == 0 and int.from_bytes(f[2:4], "big") in (2, 9):
                    continue
                # Decode complete short transfers; full-slot tests cover every DATA
                # through the same production session, with sampled pulse decoding.
                if not full_slot or f[0] != 0x82 or int.from_bytes(f[2:4], "big") % 223 == 0:
                    # Decoder execution has its own modeled MCU; it must not alter
                    # the transport clock used by the flash scheduling assertions.
                    decoder = MCU(elf)
                    peak = max(peak, decode(decoder, f, clock, period, zero, one,
                                           phase=0.5, marker=m.target[0] == 3))
                    decoded += 1
                m.send(f, now+edge_us+100)
                m.drain_erase()
            check(committed(m, img), f"{kind} profile {profile} full transfer")
            check(all(APP <= a < END for _, a, _ in m.ops), "sender never touches bootloader/records")
            stats.append(dict(sender=kind, profile=profile, frames=len(lines), pulse_frames=decoded,
                              peak_decoder_cycles_per_word=round(peak, 3), body_bytes=length))
            print(json.dumps(stats[-1]), flush=True)
    suffix = "-wire" if wire else ""
    (folder / f"board{m.target[0]}-{length}{suffix}.json").write_text(json.dumps(stats, indent=2)+"\n")


def vectors(elf):
    data = json.loads((ROOT / "build/bl-abi3/senders/vectors.json").read_text())
    for vector in data["vectors"]:
        if vector["kind"] != "update":
            continue
        m, _ = fresh(elf)
        f = bytes.fromhex(vector["hex"])
        before = bytes(m.flash)
        check(reseal(f, int.from_bytes(f[43:47], "big")) == f, "original protocol vector codec")
        m.send(f)
        check(bytes(m.flash) == before and not m.ops, "vector without matching descriptor cannot write")


def app_startup(elf, app_elf):
    """The boot path into a real app: the bootloader jumps through the app's link alias, and the app's startup puts
    gp and sp (and its RAM) where its link put them (an app linked at the zero alias entered at 0x0800xxxx computes
    them 0x08000000 too high: PC-relative)."""
    app = Emu(str(app_elf))
    end = app.addr("_data_lma") + (app.addr("_edata") - app.addr("_data_vma"))
    body = bytes(app.flash[BODY - 0x08000000:end])
    body += bytes((-len(body)) % 4)
    m = MCU(elf)
    img = image(m.target, body=body)
    m.installed(img)
    entry, registers, stray = m.boot_app(app.addr("main"))
    print(json.dumps(dict(app=str(app_elf), entry=hex(entry), sp=hex(registers[2]), gp=hex(registers[3]),
                          link_sp=hex(app.addr("_eusrstack")), link_gp=hex(app.gp), stray=len(stray))), flush=True)
    check(entry == BODY - 0x08000000, f"the jump goes to the link alias ({hex(entry)})")
    check(registers[2] == app.addr("_eusrstack"), f"app sp {hex(registers[2])} (link {hex(app.addr('_eusrstack'))})")
    check(registers[3] == app.gp, f"app gp {hex(registers[3])} (link {hex(app.gp)})")
    check(not stray, f"app startup stores outside SRAM ({[hex(a) for a in stray[:3]]})")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("elf", type=Path)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--senders", action="store_true")
    parser.add_argument("--line-profile", type=int, choices=range(4), help="one sender profile; default exercises all four")
    parser.add_argument("--full-slot", action="store_true")
    parser.add_argument("--wire", action="store_true")
    parser.add_argument("--app", type=Path, help="a real app ELF: the boot path through its startup (gp / sp)")
    args = parser.parse_args()
    if args.app:
        app_startup(args.elf, args.app)
        print(json.dumps(dict(elf=str(args.elf), checks=CHECKS, status="PASS")))
        return
    img = transactions(args.elf)
    if args.senders or args.wire:
        vectors(args.elf)
        senders(args.elf, args.full_slot, args.wire, args.line_profile)
    elif not args.smoke:
        interruption(args.elf, img)
        faults(args.elf, img)
    print(json.dumps(dict(elf=str(args.elf), checks=CHECKS, status="PASS")))


if __name__ == "__main__":
    main()
