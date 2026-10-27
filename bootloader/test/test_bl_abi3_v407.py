# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""ABI-3 production ELF tests of the CH32V407 bootloader (layout 4) in its status profile (docs/update.md), after
test_bl_abi3.py: the same transactions, interruptions and faults on the V407's flash (abi3_model_v407.py: 0xE339
erased, 4 KB sector erase, 256-byte fast page program, halfword program on erased cells only, deadlines), driven the
status way (SELECT, ANNOUNCE, polls, each operation after its BUSY answer, blocks within credit, END, the ACK), plus
the status rules (no mutation and no answer without the selection, a changed selection discards the transfer, BUSY
before every operation and the same reservation on a repeated poll, every answer consistent as the sender decodes
it), the first-boot option write cut at every step, and the boot path through a real app's startup (--app). The wire
timing and the rate training: test_bl_status_v407.py.

    python bootloader/test/test_bl_abi3_v407.py <bl.elf> [--app <V407 app fw.elf>] [--smoke]
"""
import argparse
import json
from pathlib import Path
import struct
import sys

sys.path.insert(0, str(Path(__file__).parent))
from abi3_model_v407 import (APP, BODY, END, SECTOR, MCU, PROVISIONED, PowerCut, Recovery, Reset,  # noqa: E402
                             erased, frame, image, seal)
from rv32emu import Emu                                         # noqa: E402

CHECKS = 0
BUSY, READY, RECEIVE, COMMITTED, FAILED = 2, 3, 4, 6, 7


def check(condition, label):
    global CHECKS
    CHECKS += 1
    assert condition, label


def answer_ok(st, length):
    """the sender's decode rules (protocol sender/src/ultra_sender_update.c ultra_update_status_decode)"""
    if st is None:
        return True
    pop = bin(st["mask"]).count("1")
    block = st["committed"] // 256 * 256
    remaining = min(256, length - block)
    contiguous = 0
    while contiguous < 8 and st["mask"] & (1 << contiguous):
        contiguous += 1
    rules = [st["version"] == 1, st["state"] <= 8, st["credit"] <= 8, st["mask"] <= 255,
             st["committed"] <= st["accepted"] <= length,
             st["committed"] == length or st["committed"] % 256 == 0,
             st["state"] != BUSY or (not st["credit"] and st["retry"]),
             st["state"] not in (FAILED, 8) or not st["credit"],
             st["state"] != COMMITTED or (st["committed"] == length and not st["error"]),
             all(k * 32 < remaining for k in range(8) if st["mask"] & (1 << k)),
             st["committed"] >= length or (st["accepted"] == block + min(contiguous * 32, remaining) and
                                           st["credit"] <= 8 - pop)]
    return all(rules)


def fresh(elf, installed=True):
    m = MCU(elf)
    old = image(m.target, 772, 0x00030000)
    if installed:
        m.installed(old)
    m.init_session()
    m.lock()
    return m, old


def committed(m, img):
    return m.peek(APP, 64) == img[:64] and m.peek(BODY, len(img) - 64) == img[64:]


def recover(m, img):
    new = MCU(m.elf, m.options, bytes(m.flash))
    new.init_session()
    new.lock()
    new.start(img, 19)
    st = new.transfer(img, 19)
    check(committed(new, img) and (st["state"] == COMMITTED or (st["state"] == FAILED and st["error"] == 1)),
          "a fresh session recovers retained flash (or finds the image already there: same version)")


def sectors(img):
    return list(range(APP, BODY + (len(img) - 64 + SECTOR - 1) // SECTOR * SECTOR, SECTOR))


def transactions(elf):
    m, old = fresh(elf)
    img = image(m.target)
    check(m.boot_path() == "app", "confirmed old app boot path")
    check(m.poll() is None, "no answer before a selection")
    st = m.start(img)
    check(st is not None and st["state"] == READY and st["credit"] == 8, f"erased: READY with 8 credits {st}")
    check([a for k, a, _ in m.ops if k == "erase"] == sectors(img), "4 KB cursor covers metadata and rounded body")
    check(m.boot_path() == "bootloader", "revoked image cannot run")
    st = m.transfer(img)
    check(committed(m, img) and st["state"] == COMMITTED and answer_ok(st, len(img) - 64), "complete body then metadata")
    check(m.peek(APP + 64, 2) == b"\xb7\xa5", "VERIFIED readback")
    check(m.ops[-1][0:2] == ("half", APP), "descriptor magic is the last mutation")
    ops = len(m.ops)
    m.send(frame(0x83, 7, data=img[24:28] + img[20:24] + bytes(24)))
    check(len(m.ops) == ops and m.poll()["state"] == COMMITTED, "duplicate END idempotent, COMMITTED holds")
    m.ack()
    check(m.stv("frame_t0") is not None and m.run_due() == 1, "the session's PING acknowledges: reboot requested")

    # every BUSY announces one operation; a repeated poll before it starts keeps the same reservation
    m, old = fresh(elf)
    m.select(7)
    m.send(frame(0x81, 7, 0, img[:32]))
    m.send(frame(0x81, 7, 1, img[32:64]))
    check(not m.ops, "the descriptor alone revokes and erases nothing")
    a = m.poll(run=False)
    at = m.stv("op_at")
    b = m.poll(run=False)
    check(a["state"] == BUSY and b["state"] == BUSY and a["retry"] == 200 and not m.ops and m.stv("op_at") == at,
          f"BUSY 200 ms before the erase, the same reservation on a repeat ({len(m.ops)} ops)")
    m.run_due()
    check(len(m.ops) == 1 and m.poll(run=False)["state"] == BUSY, "the erase after the window; BUSY for the next")

    # rejection before revoke: exact tuple, revision, boot version, bounds, reserved fields, flags, both CRCs
    for position, value in [(p, img[p] ^ 0x01) for p in (8, 9, 10, 11, 12, 14)] + [(13, 0x08), (14, 255), (6, 17),
                                                                                    (7, 0), (15, 4), (4, 1),
                                                                        (5, 1), (39, 1), (56, 1)]:
        m, old = fresh(elf)
        d = bytearray(img[:64]); d[position] = value; d = seal(d)
        before = bytes(m.flash)
        st = m.start(d + img[64:])
        check(bytes(m.flash) == before and not m.ops and st is not None and st["state"] == FAILED and st["error"],
              f"descriptor reject byte {position}: {st and (st['state'], st['error'])}")
    for offset, value in ((20, 0), (20, 3), (20, END - BODY + 4), (20, 0xfffffffc), (28, 0x08000000),
                          (32, BODY + 1), (32, END)):
        m, old = fresh(elf)
        d = bytearray(img[:64]); struct.pack_into(">I", d, offset, value); d = seal(d)
        before = bytes(m.flash)
        m.start(d + img[64:])
        check(bytes(m.flash) == before and not m.ops, f"descriptor bounds {offset}/{value:#x}")
    for session, flags, pass_no in ((0, 0, 0), (7, 2, 0), (7, 0, 1)):
        m, old = fresh(elf); before = bytes(m.flash)
        m.select(7)
        for half in (0, 1):
            m.send(frame(0x81, session, half, img[half * 32:(half + 1) * 32], flags, pass_no))
        m.drain(7)
        check(bytes(m.flash) == before and not m.ops, "semantic ANNOUNCE reject")
    for name, setup, err in (("supply", lambda x: x.mmio.__setitem__(0x40007004, 4), 4),
                             ("protection", lambda x: x.mmio.__setitem__(0x40022020, 0xffffffff), 2),
                             ("memory split", lambda x: x.mmio.__setitem__(0x4002201C, 0x100), 2)):
        m, old = fresh(elf); before = bytes(m.flash)
        setup(m)
        st = m.start(img)
        check(bytes(m.flash) == before and not m.ops and st["state"] == FAILED and st["error"] == err,
              f"{name} reject {st and (st['state'], st['error'])}")

    # the selection: another UID / target never mutates or answers; a changed selection discards the transfer
    for uid, target in ((bytes(12), None), (None, bytes(16))):
        m, old = fresh(elf); before = bytes(m.flash)
        m.select(7, uid=uid, target=target)
        m.send(frame(0x81, 7, 0, img[:32]))
        m.send(frame(0x81, 7, 1, img[32:64]))
        check(m.poll() is None and m.drain(7) is None and bytes(m.flash) == before and not m.ops,
              "another node's selection: no answer, no mutation")
    m, old = fresh(elf)
    m.start(img)
    m.send(frame(0x82, 7, 0, img[64:96]))
    m.select(8)
    check(m.poll(7) is None and m.poll(8)["state"] == 0, "a new selection: the old session is gone, IDLE")
    m.select(8, uid=bytes(12))
    check(m.poll(8) is None, "deselected: silence")
    check(m.poll(7, uid=bytes(12)) is None, "a poll for another UID: silence")

    # incomplete blocks never program; a later pass completes them; repeats and conflicts
    m, _ = fresh(elf); m.start(img)
    st = m.transfer(img, skip=(2, 9))
    check(not committed(m, img) and st["committed"] == 0 and st["mask"] == 0xfb, f"missing chunks never commit {st}")
    st = m.transfer(img)
    check(committed(m, img) and st["state"] == COMMITTED, "a later pass completes missing chunks")
    m, _ = fresh(elf); m.start(img)
    for k in range(3):
        m.send(frame(0x82, 7, k, img[64 + 32 * k:96 + 32 * k]))
    st = m.poll()
    check(st["state"] == RECEIVE and st["mask"] == 7 and st["credit"] == 5 and st["accepted"] == 96 and
          answer_ok(st, len(img) - 64), f"RECEIVE: mask, credit, accepted {st}")
    m.send(frame(0x82, 7, 0, img[64:96]))
    check(m.poll()["credit"] == 5, "an identical repeat takes no credit")
    m.send(frame(0x82, 7, 0, bytes(32)))
    st = m.poll()
    check(st["state"] == FAILED and st["error"] == 7, "a conflicting repeat fails the session (CONFLICT)")
    recover(m, img)
    m, _ = fresh(elf)
    m.select(7)
    m.send(frame(0x81, 7, 0, img[:32]))
    m.send(frame(0x81, 7, 0, bytes(32)))
    m.send(frame(0x81, 7, 1, img[32:64]))
    m.drain(7)
    check(not m.ops, "conflicting ANNOUNCE never revokes")

    # stale and malformed traffic cannot touch flash or complete a transfer
    m, _ = fresh(elf); m.start(img); before = bytes(m.flash); ops = len(m.ops)
    for f in [frame(0x84, 8), frame(0x82, 8, 0), frame(0x82, 7, 65535), frame(0x83, 7, data=bytes(32)),
              frame(0x86, 7)]:
        m.send(f)
    m.send(frame(0x82), corrupt=38)
    m.send(frame(0x82), corrupt=47)
    check(bytes(m.flash) == before and len(m.ops) == ops, "stale/malformed controls")
    st = m.transfer(img)
    check(committed(m, img) and st["state"] == COMMITTED, "valid stream after malformed frames")

    # END is required; inactivity never commits
    m, _ = fresh(elf); m.start(img)
    m.transfer(img, end=False)
    m.advance(m.now_us + 10001000)
    m.run_due()
    check(not committed(m, img), "timeout is not END")
    recover(m, img)

    big = image(m.target, 9000)                                  # three body sectors, 36 blocks
    m, _ = fresh(elf); m.start(big)
    st = m.transfer(big)
    check(committed(m, big) and st["state"] == COMMITTED, "multi-sector body")
    check([a for k, a, _ in m.ops if k == "erase"] == sectors(big), "every sector erased once")
    check(sum(1 for k, _, _ in m.ops if k == "page") == 36, "every block programmed once")
    return img


def interruption(elf, img):
    probe, _ = fresh(elf); probe.start(img); probe.transfer(img)
    for fraction in (0.0, 0.5, 1.0):
        for operation in range(1, len(probe.ops) + 1):
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
    # before SELECT, selected, half / whole descriptor, after the revoke, after the erase, a partial block, a
    # programmed block, the whole body before END: ABORT
    for phase in range(9):
        m, old = fresh(elf)
        if phase >= 1: m.select(7)
        if phase >= 2: m.send(frame(0x81, 7, 0, img[:32]))
        if phase >= 3: m.send(frame(0x81, 7, 1, img[32:64]))
        if phase == 4:
            m.poll(); m.advance(m.now_us + 201000); m.run_due()     # the revoke only
        if phase >= 5: m.drain(7)
        if phase == 6: m.send(frame(0x82, 7, 0, img[64:96]))
        if phase == 7:
            for k in range(8): m.send(frame(0x82, 7, k, img[64 + 32 * k:96 + 32 * k]))
            m.drain(7)
        if phase == 8: m.transfer(img, end=False)
        m.send(frame(0x84, 7))
        check(not committed(m, img), f"abort phase {phase} cannot commit")
        if m.boot_path() == "app": check(committed(m, old), "abort preserves old app only before revoke")
        recover(m, img)


def faults(elf, img):
    for mode in ("eop", "wrp", "stuck"):
        for op in (1, 2, 3, 5, 8):
            m, _ = fresh(elf)
            m.fault, m.fail_at = mode, op
            try:
                m.start(img)
                st = m.transfer(img)
                check(mode != "stuck" and st["state"] == FAILED and st["error"] == 6, f"FAILED FLASH {mode}/{op}")
            except Reset:
                check(mode == "stuck", "only stuck deadline resets")
            check(not committed(m, img), f"flash failure {mode}/{op} cannot commit")
            recover(m, img)
    for which, duration in (("erase_us", 210000), ("program_us", 61000), ("half_us", 61000)):
        m, _ = fresh(elf); setattr(m, which, duration)
        try: m.start(img); m.transfer(img)
        except Reset: pass
        check(not committed(m, img), f"{which} over declared deadline")
        recover(m, img)
    m, _ = fresh(elf); m.start(img)
    st = m.transfer(img, end=False)
    m.flash[BODY - 0x08000000 + 17] ^= 1
    m.send(frame(0x83, 7, data=img[24:28] + img[20:24] + bytes(24)))
    st = m.drain(7)
    check(not committed(m, img) and st["state"] == FAILED and st["error"] == 8, "full-body CRC precedes metadata")
    recover(m, img)


def options(elf):
    """first boot: open sectors (WRPR0 0xFF) and RAM_CODE_MOD 1 (USER bit 6): the bootloader writes the options; a cut
    at every step of that write must never let an app run on options other than the intended ones"""
    want = list(PROVISIONED)
    for user in (0xff, 0xbf):
        start = list(PROVISIONED); start[1] = user; start[4] = 0xff
        m, old = fresh(elf)
        m.options = start
        m.load_options()
        result = m.boot_path()
        b = m.last_boot
        check(b.options == want, f"first boot writes the protection and mode 0 (USER {user:#x}): {b.options}")
        check(result == "app", "then the confirmed app runs")
        writes = [k for mcu in m.boot_chain for k, _, _ in mcu.ops if k.startswith("ob")]
        check(writes == ["ob-erase"] + ["ob-half"] * 8, f"option write sequence {writes}")
        for cut in range(1, 10):
            m, old = fresh(elf)
            m.options = list(start)
            m.load_options()
            nxt = m.restarted()
            nxt.cut_at = cut
            nxt.setword("clock_khz", 20000)
            nxt.setword("options_trusted", 0, 1)
            nxt.stopping_boot = True
            try:
                nxt.call("main", max_steps=3000000)
                raise AssertionError("cut not reached")
            except PowerCut:
                pass
            torn = nxt.restarted()
            outcome = torn.boot_path()
            final = torn.last_boot.options
            if outcome == "app":
                check(final == want, f"an app runs only on the intended options (cut {cut}): {final}")
            else:
                check(True, f"torn options (cut {cut}) stay in the bootloader")
        m, old = fresh(elf)
        m.options = list(start)
        m.load_options()
        nxt = m.restarted()
        nxt.fail_at, nxt.fault = 3, "wrp"
        nxt.setword("clock_khz", 20000)
        nxt.setword("options_trusted", 0, 1)
        nxt.stopping_boot = True
        try:
            nxt.call("main", max_steps=3000000)
            raise AssertionError("no boot decision")
        except Recovery:
            check(True, "a refused option write stays in the bootloader")
        check(nxt.restarted().boot_path() == "bootloader", "and after a power cycle too")
    start = list(PROVISIONED); start[1] = 0xff
    m, old = fresh(elf)
    m.options = start
    m.load_options()
    check(m.mmio[0x4002201C] & 0x100, "model: OBR bit 8 from USER bit 6")
    check(m.boot_path() == "app" and m.last_boot.options == want and not m.last_boot.mmio[0x4002201C] & 0x100,
          "mode 1 with protected sectors: mode 0 written, then the app (OBR bit 8 clear)")


def app_startup(elf, app_elf):
    app = Emu(str(app_elf))
    end = app.addr("_data_lma") + (app.addr("_edata") - app.addr("_data_vma"))
    body = bytes(app.flash[BODY - 0x08000000:end])
    body += erased((-len(body)) % 4)
    m = MCU(elf)
    img = image(m.target, body=body)
    m.installed(img)
    entry, registers, stray = m.boot_path(app.addr("main"))
    print(json.dumps(dict(app=str(app_elf), entry=hex(entry), sp=hex(registers[2]), gp=hex(registers[3]),
                          link_sp=hex(app.addr("_eusrstack")), link_gp=hex(app.gp), stray=len(stray))), flush=True)
    check(entry == BODY - 0x08000000, f"the jump goes to the link alias ({hex(entry)})")
    check(registers[2] == app.addr("_eusrstack"), "app sp")
    check(registers[3] == app.gp, "app gp")
    check(not stray, "app startup stores outside SRAM")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("elf", type=Path)
    parser.add_argument("--smoke", action="store_true")
    parser.add_argument("--app", type=Path)
    args = parser.parse_args()
    if args.app:
        app_startup(args.elf, args.app)
    else:
        img = transactions(args.elf)
        if not args.smoke:
            interruption(args.elf, img)
            faults(args.elf, img)
            options(args.elf)
    print(json.dumps(dict(elf=str(args.elf), checks=CHECKS, status="PASS")))


if __name__ == "__main__":
    main()
