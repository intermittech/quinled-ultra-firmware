# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Make the production hex: bootloader + app image, marked verified and confirmed (boots without a listen window).

    python tools/release/mkfactory.py --bootloader <bl>/bl.bin --image <image>.bin --out factory.hex

Addresses are 0x0000-based like the build's fw.hex (WCH-LinkUtility / wlink map them to 0x08000000). Programming:
erase the whole chip, program the hex, then set write protection on sectors 0-1 (the bootloader) in the option
bytes (WRPR0 bits 0 and 1 = 0). Bytes not in the hex (the rest of the status area) must stay erased.
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import qdu  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bootloader", required=True, help="bl.bin")
    ap.add_argument("--image", required=True, help="update image from mkimage.py")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()

    bl = open(a.bootloader, "rb").read()
    if len(bl) > qdu.BL_SIZE or bl[0x10:0x14] != b"QDBL":
        raise SystemExit(f"{a.bootloader}: not a bootloader (size {len(bl)}, info magic {bl[0x10:0x14]!r})")
    bl_version, hw_id, hw_rev = bl[0x14], bl[0x15], bl[0x16]
    img = open(a.image, "rb").read()
    info = qdu.check_image(img)
    if info["hw_id"] != hw_id or not (info["hw_rev_mask"] >> hw_rev) & 1 or info["min_bl"] > bl_version:
        raise SystemExit(f"image (hw {info['hw_id']}, revisions 0x{info['hw_rev_mask']:x}, needs bootloader "
                         f"{info['min_bl']}) does not fit this bootloader (hw {hw_id}, revision {hw_rev}, version {bl_version})")
    status = struct.pack("<HH", qdu.ST_VERIFIED, qdu.ST_CONFIRMED)
    segs = [(0x0000, bl),
            (qdu.APP_DESC - 0x08000000, img[:qdu.DESC_BYTES] + status),
            (qdu.APP_BODY - 0x08000000, img[qdu.DESC_BYTES:])]
    with open(a.out, "w", encoding="ascii", newline="\n") as f:
        f.write(qdu.intel_hex(segs))
    print(f"{a.out}: bootloader {bl_version} (hw {hw_id} rev {hw_rev}, {len(bl)} B) + app {info['version']} "
          f"({info['body_len']} B), verified + confirmed")


if __name__ == "__main__":
    main()
