# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Make an update image (descriptor + body) from an app built with LDSCRIPT=vendor/wch/Link_app.ld.

    python tools/release/mkimage.py --elf <out>/fw.elf --bin <out>/fw.bin --version 0.5.0 --rev-mask 0x6 \
        --build-id <git hash> --out quinled-dig2analog-ultra-5ch-0.5.0-hw1.bin

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


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bin", required=True, help="fw.bin of the app linked at 0x00002100")
    ap.add_argument("--elf", help="fw.elf: checks the entry point")
    ap.add_argument("--version", required=True, help="0.6.0 or 0.6.0-alpha.N / -beta.N / -rc.N")
    ap.add_argument("--rev-mask", required=True, type=lambda s: int(s, 0),
                    help="board revisions it runs on (bit 0 v0.1 PB15 only, 1 PB11, 2 current sense)")
    ap.add_argument("--hw-id", type=int, default=qdu.HW_ID_D2A_ULTRA_5CH)
    ap.add_argument("--min-bl", type=int, default=1)
    ap.add_argument("--build-id", default="", help="git hash or similar (<= 16 characters)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    if a.elf and elf_entry(a.elf) != qdu.APP_ENTRY:
        raise SystemExit(f"{a.elf}: entry 0x{elf_entry(a.elf):08X}, expected 0x{qdu.APP_ENTRY:08X} "
                         "(build with LDSCRIPT=vendor/wch/Link_app.ld)")
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
