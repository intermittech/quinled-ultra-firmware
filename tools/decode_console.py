# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Strict jig-side reference decoder; no hardware, no writes outside supplied output."""
import json,pathlib,re,sys
ROOT=pathlib.Path(__file__).resolve().parents[1]
SCHEMA=json.loads((ROOT/'tools/console_schema.json').read_text())
TABLE={r['record']:r for r in SCHEMA['records']}
def crc8(data):
    c=0
    for b in data:
        c^=b
        for _ in range(8): c=((c<<1)^ (7 if c&128 else 0))&255
    return c
def decode(line):
    if len(line)>SCHEMA['max_line_bytes'] or not line.endswith(b'\r\n'):raise ValueError('line framing')
    m=re.fullmatch(rb'([A-Za-z])'+format(SCHEMA['schema'],'02X').encode()+rb',([0-9A-F]+)((?:,[0-9A-F]+)*)\*([0-9A-F]{2})\r\n',line)
    if not m or crc8(line[:line.index(b'*')])!=int(m[4],16):raise ValueError('format/schema/checksum')
    r=TABLE[m[1].decode()]; values=m[3].split(b',')[1:]
    if len(values)!=len(r['fields']):raise ValueError('field count')
    fields={}
    for f,x in zip(r['fields'],values):
        v=int(x,16); bits=f['bytes']*8
        if v>=(1<<bits):raise ValueError('field range')
        fields[f['name']]=v-(1<<bits) if f['signed'] and v&(1<<(bits-1)) else v
    return dict(record=r['record'],index=int(m[2],16),fields=fields)
if __name__=='__main__':
    for line in pathlib.Path(sys.argv[1]).read_bytes().splitlines(keepends=True):print(json.dumps(decode(line)))
