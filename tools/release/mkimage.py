# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Make an update image (descriptor + body) from an app built with LDSCRIPT=src/mcu/ch32v20x/Link_app.ld.

    python tools/release/mkimage.py --elf <out>/fw.elf --bin <out>/fw.bin --version 0.7.0 --rev-mask 0x8 \
        --build-id <git hash> --out quinled-dig2analog-plus-5ch-0.7.0-rev04.bin

Writes the image (what the ESP32 sends: ANNOUNCE = its first 64 bytes, DATA = the rest in 32-byte chunks) and a
.json next to it (version, revisions, size, SHA-256, ...) for make_manifest.py.
"""
import argparse
import hashlib
import json
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import qdu  # noqa: E402


def elf_entry(path):
    with open(path, "rb") as f:
        h = f.read(32)
    if h[:4] != b"\x7fELF" or h[4] != 1:
        raise SystemExit(f"{path}: not a 32-bit ELF")
    return struct.unpack_from("<I", h, 24)[0]


def elf_symbol(path, name):
    """the value of a symbol in a 32-bit little-endian ELF's .symtab, or None"""
    e = open(path, "rb").read()
    shoff, = struct.unpack_from("<I", e, 32)
    shentsize, shnum = struct.unpack_from("<HH", e, 46)
    secs = [struct.unpack_from("<IIIIIIIIII", e, shoff + i * shentsize) for i in range(shnum)]
    for s in secs:
        if s[1] != 2:                                   # SHT_SYMTAB
            continue
        strtab = secs[s[6]]                             # sh_link: its string table
        for k in range(s[5] // 16):                     # sh_size / sizeof(Elf32_Sym)
            st_name, st_value = struct.unpack_from("<II", e, s[4] + 16 * k)
            end = e.index(b"\0", strtab[4] + st_name)
            if e[strtab[4] + st_name:end] == name.encode("ascii"):
                return st_value
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", required=True, help="fw.bin of the app linked at 0x00002100")
    ap.add_argument("--elf", help="fw.elf: checks the entry point")
    ap.add_argument("--version", required=True, help="0.7.0 or 0.7.0-alpha.N / -beta.N / -rc.N")
    ap.add_argument("--rev-mask", required=True, type=lambda s: int(s, 0),
                    help="board revisions it runs on (bit 0 v0.1 PB15 only, 1 PB11, 2 current sense)")
    ap.add_argument("--hw-id", type=int, help="the board's hardware id (1 dig2analog-plus-5ch, 2 the Diff-Power "
                    "Diff-Power-UR-Analog-M); default: the ELF's fw_hw_id (tools/build_target.sh, from BOARD), never a "
                    "guess")
    ap.add_argument("--min-bl", type=int, default=qdu.MIN_BL, help="default: 2")
    ap.add_argument("--build-id", default="", help="git hash or similar (<= 16 characters)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    if a.elf and elf_entry(a.elf) != qdu.APP_ENTRY:
        raise SystemExit(f"{a.elf}: entry 0x{elf_entry(a.elf):08X}, expected 0x{qdu.APP_ENTRY:08X} "
                         "(build with LDSCRIPT=src/mcu/ch32v20x/Link_app.ld)")
    # the hardware id from the build itself: a receiver image (BOARD=2) never carries 1
    elf_hw = elf_symbol(a.elf, "fw_hw_id") if a.elf else None
    if a.hw_id is None:
        if elf_hw is None:
            raise SystemExit("no hardware id: give --elf of a build with fw_hw_id (tools/build_target.sh) or --hw-id")
        a.hw_id = elf_hw
    elif elf_hw is not None and elf_hw != a.hw_id:
        raise SystemExit(f"--hw-id {a.hw_id}, but {a.elf} was built for hardware {elf_hw} (BOARD)")
    if a.hw_id not in qdu.HW_IDS:
        raise SystemExit(f"hardware id {a.hw_id} unknown (known: {sorted(qdu.HW_IDS)})")
    body = open(a.bin, "rb").read()
    body += b"\xFF" * (-len(body) % 4)
    if len(body) > qdu.APP_BODY_MAX:
        raise SystemExit(f"app is {len(body)} bytes, the slot holds {qdu.APP_BODY_MAX}")
    ver = qdu.parse_version(a.version)
    desc = qdu.make_descriptor(body, ver, a.rev_mask, a.hw_id, a.min_bl, a.build_id.encode("ascii"))
    img = desc + body
    info = qdu.check_image(img)
    with open(a.out, "wb") as f:
        f.write(img)
    meta = {
        "file": os.path.basename(a.out), "version": qdu.format_version(ver), "version_word": ver,
        "hw_id": a.hw_id, "hw_rev_mask": a.rev_mask, "min_bootloader": a.min_bl, "size": len(img),
        "body_len": len(body), "body_crc": f"{info['body_crc']:08x}", "sha256": hashlib.sha256(img).hexdigest(),
        "build_id": a.build_id,
    }
    with open(os.path.splitext(a.out)[0] + ".json", "w", encoding="utf-8", newline="\n") as f:
        json.dump(meta, f, indent=2, sort_keys=True)
        f.write("\n")
    print(f"{a.out}: {meta['version']}, revisions 0x{a.rev_mask:x}, {len(body)} B body "
          f"({100 * len(body) / qdu.APP_BODY_MAX:.0f} % of the slot), sha256 {meta['sha256'][:16]}...")


if __name__ == "__main__":
    main()
