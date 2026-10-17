# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Shared helpers for the QuinLED dig2analog-ultra-5ch release tools (formats: src/core/ultra_update.h).

Version words, the descriptor, CRC-32/MPEG-2 over little-endian words (= the CH32 CRC unit), Intel HEX output.
"""
import os
import re
import struct

APP_DESC = 0x08002000
APP_BODY = 0x08002100
# UPPER_RECORDS (0.6.5, as tools/build_target.sh): 1 (default) = the records in the upper CodeFlash, the app slot to
# 0x08010000, bootloader 2; 0 = 0.6.4's layout: the bootloader's slot ends at 0xF700, 0xF600 holds the calibration
UPPER_RECORDS = os.environ.get("UPPER_RECORDS", "1") != "0"
APP_END = 0x08010000 if UPPER_RECORDS else 0x0800F600
APP_ENTRY = 0x00002100
APP_BODY_MAX = APP_END - APP_BODY          # 57,088 bytes (0.6.4's layout: 54,528, the app's link)
MIN_BL = 2 if UPPER_RECORDS else 1         # the oldest bootloader that takes such an image (ultra_update.h)
BL_SIZE = 0x2000
DESC_BYTES = 64
HW_ID_D2A_ULTRA_5CH = 1
HW_ID_DP_ULTRA_RX = 2                      # (round 11, B2) the QuinLED Diff-Power Ultra-Analog receiver
HW_ID_DP_ULTRA_DIGITAL = 3                 # the QuinLED Diff-Power Ultra-Digital receiver (BOARD=3; protocol 0.2.7 proposal)
HW_IDS = {HW_ID_D2A_ULTRA_5CH: "dig2analog-ultra-5ch", HW_ID_DP_ULTRA_RX: "diff-power-ultra-analog",
          HW_ID_DP_ULTRA_DIGITAL: "diff-power-ultra-digital"}
REV_V01_PB15, REV_PB11, REV_ISENSE = 0, 1, 2
ST_VERIFIED, ST_CONFIRMED = 0xA5B7, 0xA5C3
STAGES = {"alpha": 0x00, "beta": 0x40, "rc": 0x80}
CHANNEL_RANK = {"alpha": 0, "beta": 1, "stable": 2}

_SEMVER = re.compile(r"^v?(\d+)\.(\d+)\.(\d+)(?:-(alpha|beta|rc)\.(\d+))?$")


def parse_version(s):
    """'0.6.0-beta.1' / 'v0.5.0' -> 32-bit version word (major, minor, patch, pre-release byte)"""
    m = _SEMVER.match(s.strip())
    if not m:
        raise ValueError(f"not a version: {s!r} (want e.g. 0.6.0 or 0.6.0-alpha.N / -beta.N / -rc.N)")
    ma, mi, pa = int(m.group(1)), int(m.group(2)), int(m.group(3))
    if max(ma, mi, pa) > 255:
        raise ValueError(f"{s}: major / minor / patch must be <= 255")
    if m.group(4):
        n = int(m.group(5))
        if n > 63:
            raise ValueError(f"{s}: pre-release number must be <= 63")
        pre = STAGES[m.group(4)] + n
    else:
        pre = 0xFF
    return (ma << 24) | (mi << 16) | (pa << 8) | pre


def format_version(w):
    ma, mi, pa, pre = (w >> 24) & 255, (w >> 16) & 255, (w >> 8) & 255, w & 255
    base = f"{ma}.{mi}.{pa}"
    if pre == 0xFF:
        return base
    for name, off in sorted(STAGES.items(), key=lambda kv: -kv[1]):
        if pre >= off:
            return f"{base}-{name}.{pre - off}"
    return base


def stage_of(w):
    """'alpha', 'beta' or 'stable' (an rc counts as beta)"""
    pre = w & 255
    return "stable" if pre == 0xFF else ("alpha" if pre < 0x40 else "beta")


def crc32_words(data, crc=0xFFFFFFFF):
    """CRC-32/MPEG-2 over little-endian 32-bit words, MSB first (the CH32 CRC unit); len(data) % 4 == 0"""
    if len(data) % 4:
        raise ValueError("length must be a multiple of 4")
    for (w,) in struct.iter_unpack("<I", data):
        crc ^= w
        for _ in range(32):
            crc = ((crc << 1) ^ 0x04C11DB7) & 0xFFFFFFFF if crc & 0x80000000 else (crc << 1) & 0xFFFFFFFF
    return crc


def make_descriptor(body, version_word, hw_rev_mask, hw_id=HW_ID_D2A_ULTRA_5CH, min_bl=1, build_id=b""):
    if len(body) % 4 or not 0 < len(body) <= APP_BODY_MAX:
        raise ValueError(f"body length {len(body)}: must be a multiple of 4, 1..{APP_BODY_MAX}")
    d = bytearray(DESC_BYTES)
    d[0:4] = b"QDU5"
    d[4] = 1
    d[5] = hw_id
    struct.pack_into(">HIIIII", d, 6, hw_rev_mask, version_word, len(body), crc32_words(body), APP_BODY, APP_ENTRY)
    d[28] = min_bl
    d[32:48] = build_id[:16].ljust(16, b"\0")
    struct.pack_into(">I", d, 60, crc32_words(bytes(d[:60])))
    return bytes(d)


def parse_descriptor(d):
    if len(d) < DESC_BYTES or d[0:4] != b"QDU5":
        raise ValueError("no descriptor (magic)")
    if crc32_words(bytes(d[:60])) != struct.unpack_from(">I", d, 60)[0]:
        raise ValueError("descriptor CRC")
    hw_rev_mask, ver, blen, bcrc, load, entry = struct.unpack_from(">HIIIII", d, 6)
    return {"desc_version": d[4], "hw_id": d[5], "hw_rev_mask": hw_rev_mask, "version_word": ver,
            "version": format_version(ver), "body_len": blen, "body_crc": bcrc, "load": load, "entry": entry,
            "min_bl": d[28], "build_id": d[32:48].rstrip(b"\0").decode("ascii", "replace")}


def check_image(img):
    """an update image (descriptor + body): returns the parsed descriptor or raises"""
    info = parse_descriptor(img[:DESC_BYTES])
    body = img[DESC_BYTES:]
    if info["desc_version"] != 1 or info["load"] != APP_BODY or info["entry"] != APP_ENTRY:
        raise ValueError("descriptor layout / load address")
    if info["body_len"] != len(body) or crc32_words(body) != info["body_crc"]:
        raise ValueError("body length / CRC")
    if info["body_len"] % 4 or not 0 < info["body_len"] <= APP_BODY_MAX:
        # 0.6.4's layout: the bootloader's slot runs to 0xF700 (ultra_update.h), 256 bytes past the app's link: an
        # image that long would erase the calibration page at 0xF600 (src/hal/board.h). Refused here too.
        raise ValueError(f"body length {info['body_len']}: must be a multiple of 4, 1..{APP_BODY_MAX}")
    return info


def intel_hex(segments):
    """segments: [(address, bytes)] with 0x0000-based addresses (like the build's fw.hex) -> Intel HEX text"""
    out = []
    upper = None
    for addr, data in sorted(segments):
        for off in range(0, len(data), 16):
            a = addr + off
            chunk = data[off:off + 16]
            if (a >> 16) != upper:
                upper = a >> 16
                rec = bytes([2, 0, 0, 4, upper >> 8, upper & 255])
                out.append(":" + rec.hex().upper() + f"{(-sum(rec)) & 255:02X}")
            rec = bytes([len(chunk), (a >> 8) & 255, a & 255, 0]) + chunk
            out.append(":" + rec.hex().upper() + f"{(-sum(rec)) & 255:02X}")
    out.append(":00000001FF")
    return "\n".join(out) + "\n"
