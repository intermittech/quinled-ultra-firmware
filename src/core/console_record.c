/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "console_record.h"
#include "config.h"
static char *hex(char *p, uint32_t v)
{
    /* Reserve the digits, then fill backwards: no temporary or reverse copy. */
    char *end=p, *q; uint32_t x=v;
    do { end++; x>>=4; } while(x);
    q=end;
    do { uint32_t d=v&15u; *--q=(char)(d<10u?'0'+d:'A'+d-10u); v>>=4; } while(v);
    return end;
}
/* Header: record ASCII in the high byte, number of native runs in the low.
 * Runs: width log2 in bits 14..15, count-1 in 12..13, byte offset in 0..11.
 * Naturally aligned volatile loads preserve ISR counter reads; no shared writes. */
void console_record(void (*write)(const char *), uint32_t index,
                    const void *base, const uint16_t *layout)
{
    char b[CONSOLE_LINE_MAX+1], *p=b; uint32_t i, n=*layout & 255u;
    *p++=(char)(*layout++>>8); *p++=CONSOLE_SCHEMA_HEX[0]; *p++=CONSOLE_SCHEMA_HEX[1]; *p++=','; p=hex(p,index);
    while(n--) {
        uint32_t d=*layout++, w=1u<<(d>>14), count=((d>>12)&3u)+1u;
        const volatile uint8_t *a=(const volatile uint8_t *)base;
        if(a) a+=d&4095u;
        do {
            uint32_t v=0;
            if(a) {
                if(w==4u) v=*(const volatile uint32_t *)a;
                else if(w==2u) v=*(const volatile uint16_t *)a;
                else v=*a;
                a+=w;
            }
            *p++=','; p=hex(p,v);
        } while(--count);
    }
    i=crc8_smbus((const uint8_t *)b,(uint32_t)(p-b));
    p=hex(p,0x100u|i); p[-3]='*';          /* "*XY": the CRC's two digits by hex() (a leading 1, overwritten) */
    *p++='\r'; *p++='\n'; *p=0; write(b);
}
