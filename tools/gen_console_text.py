# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Compress the console's text at build time (byte-pair tokens), for core/cz.h's streaming decoder.

    python tools/gen_console_text.py <out dir> [--perf] [--upper] [--no-jig] [--full] [source.c ...]   (default: the sources
    in SOURCES)

Writes <out dir>/cz_text.h (rewritten only when it changes); tools/build_target.sh, test/build_host.cmd and
test/build_robust.cmd run it and add <out dir> to the include path.

The sources mark each console string with Z(name, "text" ...) (core/cz.h): the C compiler sees only the name, an
offset into the compressed blob; this script reads the text from the same call. One name, one text (a name may be
used again with the same text). ZP(name, "text") is the same for a PERF build's texts (core/perf.h, the bench's
`perf`): read only with --perf (build_target.sh with PERF=1 / 2), so a release's blob holds none of them. ZU(...):
the texts of the monitoring only the upper record layout has room for (core/config.h MON_EXTRA): read with --upper
(build_target.sh unless UPPER_RECORDS=0; the host tests' builds); ZL(...) the same text for 0.6.4's layout, read
only without --upper. ZJ(...): the test jig's texts, left out with --no-jig. (0.6.7) ZC(...): the full console's
texts (core/config.h CONSOLE_FULL: commands, help, the keys, their replies), read only with --full (build_target.sh
with CONSOLE_FULL=1, the host tests' builds): the release console takes no input and holds none of them. The marks
combine: Z and any of P, U, L, J, C, each a condition the text needs (ZCU: --full and --upper, ZCL: --full without
--upper).

Format (decoded by cz_next in core/cli.c, character by character, no buffer):
  - every string is stored once (identical texts share it), compressed, and ends with a NUL byte;
  - a byte 0x80 + i is token i: the two bytes cz_pair[i][0], cz_pair[i][1], each a character or again a token
    (byte-pair encoding: the most frequent adjacent pair becomes the next token, at most 128 tokens);
  - bytes below 0x20 other than LF (the NUL ends and separators of the name lists, the settings keys' command
    bytes) are never part of a token, CR only as CR LF, so the console walks the lists and the keys on the
    compressed bytes (nth / skip / key_cmd);
  - a token expands with at most CZ_DEPTH pending right halves (the decoder's stack, a few bytes);
  - the offsets are biased by CZ_BIAS (the blob stays below 4 KB): every Z(...) is a 12-bit signed immediate.
Standard library only; deterministic (ties broken by the pair's byte values)."""
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, ".."))
SOURCES = ["src/core/cli.c", "src/app/main.c"]                   # (a new file with Z(...) texts: add it here;
                                                                 # main.c's units come with it, its #include "x.c")
MAX_TOKENS = 128
MAX_DEPTH = 8
BIAS = 2048


# ------------------------------------------------------------------------------------------------ C scanning
TOKEN = re.compile(r'''
    (?P<ws>\s+|//[^\n]*|/\*.*?\*/|\\\r?\n)
  | (?P<str>"(?:\\.|[^"\\\n])*")
  | (?P<chr>'(?:\\.|[^'\\\n])+')
  | (?P<id>[A-Za-z_]\w*)
  | (?P<num>\.?\d[\w.]*)
  | (?P<punct>.)
''', re.S | re.X)

ESC = {"n": 10, "r": 13, "t": 9, "a": 7, "b": 8, "f": 12, "v": 11, "\\": 92, '"': 34, "'": 39, "?": 63}


def unescape(body, where):
    out, i = bytearray(), 0
    while i < len(body):
        ch = body[i]
        if ch != "\\":
            out += ch.encode("ascii")
            i += 1
            continue
        e = body[i + 1]
        if e in ESC:
            out.append(ESC[e])
            i += 2
        elif e == "x":
            m = re.match(r"[0-9A-Fa-f]+", body[i + 2:])
            out.append(int(m.group(0), 16) & 0xFF)
            i += 2 + len(m.group(0))
        elif e in "01234567":
            m = re.match(r"[0-7]{1,3}", body[i + 1:])
            out.append(int(m.group(0), 8) & 0xFF)
            i += 1 + len(m.group(0))
        else:
            raise SystemExit("%s: unknown escape \\%s" % (where, e))
    return bytes(out)


def tokens(path):
    """(kind, text, line, path) of path's tokens; a local #include "x.c" in place (round 12: src/app/main.c's units,
    one translation unit: the texts keep their order)"""
    src = open(path, encoding="utf-8").read()
    line, last = 1, []
    for m in TOKEN.finditer(src):
        kind = m.lastgroup
        if kind != "ws":
            if kind == "str" and last == ["#", "include"] and m.group().endswith('.c"'):
                yield from tokens(os.path.join(os.path.dirname(path), m.group()[1:-1]))
            else:
                yield kind, m.group(), line, path
            last = (last + [m.group()])[-2:]
        line += m.group().count("\n")


def shown(path):
    """path relative to the firmware folder (as given when it is elsewhere, e.g. another drive)"""
    try:
        return os.path.relpath(path, FW).replace("\\", "/")
    except ValueError:
        return path.replace("\\", "/")


def wanted(mark, perf, upper, no_jig, full):
    """a mark Z + conditions: P perf, U upper, L not upper, J not no_jig, C full"""
    c = mark[1:]
    return (re.fullmatch(r"Z[PULJC]*", mark) is not None and ("P" not in c or perf) and ("U" not in c or upper) and
            ("L" not in c or not upper) and ("J" not in c or not no_jig) and ("C" not in c or full))


def scan(path, perf=False, upper=False, no_jig=False, full=False):
    """[(name, text bytes, 'file:line')] for every Z(name, "..." ...) in path whose mark's conditions hold (ZP(...)
    with perf, ZU(...) with upper, ZL(...) without, ZJ(...) unless no_jig, ZC(...) with full; combined: all of them)"""
    found, toks = [], list(tokens(path))
    for i in range(len(toks) - 4):
        if toks[i][0] != "id" or not wanted(toks[i][1], perf, upper, no_jig, full) or toks[i + 1][1] != "(" or \
                toks[i + 2][0] != "id" or toks[i + 3][1] != ",":
            continue
        j, text = i + 4, b""
        if toks[j][0] != "str":
            continue                       # e.g. the macro's own definition
        where = "%s:%d" % (shown(toks[i][3]), toks[i][2])
        while toks[j][0] == "str":
            text += unescape(toks[j][1][1:-1], where)
            j += 1
        if toks[j][1] != ")":
            raise SystemExit("%s: Z(%s, ...): only string literals may follow the name" % (where, toks[i + 2][1]))
        if any(b >= 0x80 for b in text):
            raise SystemExit("%s: Z(%s): only 7-bit text (bytes >= 0x80 are tokens)" % (where, toks[i + 2][1]))
        if re.search(rb"(^|[^%])(%%)*%$", text):       # cf would read the code after the string's end
            raise SystemExit("%s: Z(%s): ends in a single '%%' (write \"%%%%\")" % (where, toks[i + 2][1]))
        found.append((toks[i + 2][1], text, where))
    return found


# ------------------------------------------------------------------------------------------------ compression
def mergeable(a, b):
    """a pair may become a token: no NUL / control byte (list separators, the keys' command bytes), and a CR only
    as CR LF (a CR alone is a command byte: CMD_MIN_PULSE 0x0d)"""
    if (a < 0x20 and a not in (10, 13)) or (b < 0x20 and b not in (10, 13)):
        return False
    return a != 13 or b == 10


def replace(seq, a, b, t):
    out, i, n = [], 0, 0
    while i < len(seq):
        if i + 1 < len(seq) and seq[i] == a and seq[i + 1] == b:
            out.append(t)
            i += 2
            n += 1
        else:
            out.append(seq[i])
            i += 1
    return out, n


def compress(texts):
    seqs = [list(t) + [0] for t in texts]
    pairs, need = [], {}
    nd = lambda x: need.get(x, 0)
    for t in range(0x80, 0x80 + MAX_TOKENS):
        cnt = {}
        for s in seqs:
            for a, b in zip(s, s[1:]):
                if mergeable(a, b):
                    cnt[(a, b)] = cnt.get((a, b), 0) + 1
        best, best_n = None, 2
        for (a, b), n in sorted(cnt.items(), key=lambda kv: (-kv[1], kv[0])):
            if n <= best_n:
                break
            if max(nd(a) + 1, nd(b)) > MAX_DEPTH:
                continue
            if a == b:                     # runs: the replacement takes them two by two
                n = sum(replace(s, a, b, t)[1] for s in seqs)
            if n > best_n or (n == best_n and best is not None and (a, b) < best):
                best, best_n = (a, b), n
        if best is None:
            break
        a, b = best
        seqs = [replace(s, a, b, t)[0] for s in seqs]
        pairs.append(best)
        need[t] = max(nd(a) + 1, nd(b))
    return seqs, pairs, max([0] + list(need.values()))


def expand(x, pairs):
    if x < 0x80:
        return bytes([x])
    a, b = pairs[x - 0x80]
    return expand(a, pairs) + expand(b, pairs)


def c_text(bs):
    s = ""
    for i, b in enumerate(bs):
        nxt = bs[i + 1] if i + 1 < len(bs) else 0
        if b == 0:
            s += "\\000" if 48 <= nxt <= 55 else "\\0"     # (a digit after it: no octal escape that runs on)
        elif 32 <= b < 127 and b not in (34, 92) and not (b in (42, 47) and nxt in (42, 47)):
            s += chr(b)                                     # (no "*/" or "/*" inside the comment)
        else:
            s += {10: "\\n", 13: "\\r", 34: '\\"', 92: "\\\\"}.get(b, "\\%03o" % b)
    return s


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    out_dir = sys.argv[1]
    perf, upper, no_jig = "--perf" in sys.argv[2:], "--upper" in sys.argv[2:], "--no-jig" in sys.argv[2:]
    full = "--full" in sys.argv[2:]
    srcs = [a for a in sys.argv[2:] if a not in ("--perf", "--upper", "--no-jig", "--full")] or \
        [os.path.join(FW, s) for s in SOURCES]
    names, texts, order = {}, [], {}
    for path in srcs:
        for name, text, where in scan(path, perf, upper, no_jig, full):
            if name in names and names[name][0] != text:
                raise SystemExit("%s: Z(%s) has another text at %s" % (where, name, names[name][1]))
            names.setdefault(name, (text, where))
            if text not in order:
                order[text] = len(texts)
                texts.append(text)
    seqs, pairs, depth = compress(texts)
    for t, s in zip(texts, seqs):                          # the decoder's view, checked here once more
        assert b"".join(expand(x, pairs) for x in s) == t + b"\0"
    offs, blob = [], []
    for s in seqs:
        offs.append(len(blob))
        blob += s
    if len(blob) > 2 * BIAS - 1:
        raise SystemExit("compressed text %d B: more than the 12-bit offsets reach" % len(blob))
    raw = sum(len(t) + 1 for t in texts)
    rel = ", ".join(shown(p) for p in srcs)
    h = ["/* Generated by tools/gen_console_text.py from %s: do not edit. */" % rel,
         "/* %d strings, %d B as text (NUL included) -> %d B compressed + %d B tokens (%d pairs), depth %d */"
         % (len(texts), raw, len(blob), 2 * len(pairs), len(pairs), depth),
         "#ifndef CZ_TEXT_H", "#define CZ_TEXT_H", "",
         "#define CZ_DEPTH %du" % max(1, depth), "#define CZ_BIAS %d" % BIAS, ""]
    for name in sorted(names, key=lambda n: (order[names[n][0]], n)):
        h.append("#define CZ_%s (%d)" % (name, offs[order[names[name][0]]] - BIAS))
    h += ["", "#ifdef CZ_TEXT_DATA", "static const uint8_t cz_pair[%d][2] = {" % max(1, len(pairs))]
    for i, (a, b) in enumerate(pairs):
        h.append("    { 0x%02x, 0x%02x },   /* 0x%02x \"%s\" */" % (a, b, 0x80 + i, c_text(expand(0x80 + i, pairs))))
    if not pairs:
        h.append("    { 0, 0 }")
    h += ["};", "static const uint8_t cz_text[%d] = {" % len(blob)]
    by_text = {}
    for n, (t, _) in names.items():
        by_text.setdefault(t, []).append(n)
    for t, s in zip(texts, seqs):
        h.append("    /* %s: \"%s\" */" % (", ".join(sorted(by_text[t])), c_text(t)))
        for k in range(0, len(s), 16):
            h.append("    " + " ".join("0x%02x," % x for x in s[k:k + 16]))
    h += ["};", "#endif", "", "#endif", ""]
    text = "\n".join(h)
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "cz_text.h")
    old = open(path, encoding="utf-8").read() if os.path.exists(path) else None
    if old != text:
        with open(path, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
    print("console text: %d strings, %d -> %d B + %d B tokens" % (len(texts), raw, len(blob), 2 * len(pairs)))


if __name__ == "__main__":
    main()
