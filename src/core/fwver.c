/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "fwver.h"

static const char *num(const char *s, uint32_t max, uint32_t *out)
{
    uint32_t v = 0;
    if (*s < '0' || *s > '9') {
        return 0;
    }
    while (*s >= '0' && *s <= '9') {
        v = v * 10u + (uint32_t)(*s++ - '0');
        if (v > max) {
            return 0;
        }
    }
    *out = v;
    return s;
}

static int prefix(const char *s, const char *p)
{
    while (*p) {
        if (*s++ != *p++) {
            return 0;
        }
    }
    return 1;
}

uint32_t fwver_parse(const char *s)
{
    uint32_t part[3], pre = 0xFFu, n, i;
    if (*s == 'v') {
        s++;
    }
    for (i = 0; i < 3u; i++) {
        s = num(s, 255u, &part[i]);
        if (s == 0 || (i < 2u && *s++ != '.')) {
            return 0;
        }
    }
    if (*s == '-') {
        s++;
        if (prefix(s, "alpha.")) {
            pre = 0x00u;
            s += 6;
        } else if (prefix(s, "beta.")) {
            pre = 0x40u;
            s += 5;
        } else if (prefix(s, "rc.")) {
            pre = 0x80u;
            s += 3;
        } else {
            return 0;
        }
        s = num(s, 63u, &n);
        if (s == 0) {
            return 0;
        }
        pre += n;
    }
    if (*s != 0) {
        return 0;
    }
    return (part[0] << 24) | (part[1] << 16) | (part[2] << 8) | pre;
}

static char *put_u(char *p, uint32_t v)
{
    char t[4];
    uint32_t n = 0;
    do {
        t[n++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v != 0u);
    while (n) {
        *p++ = t[--n];
    }
    return p;
}

static char *put_s(char *p, const char *s)
{
    while (*s) {
        *p++ = *s++;
    }
    return p;
}

void fwver_format(uint32_t w, char *buf)
{
    char *p = buf;
    uint32_t pre = w & 0xFFu;
    if (w == 0u) {
        *p = 0;
        return;
    }
    p = put_u(p, w >> 24);
    *p++ = '.';
    p = put_u(p, (w >> 16) & 0xFFu);
    *p++ = '.';
    p = put_u(p, (w >> 8) & 0xFFu);
    if (pre != 0xFFu) {
        p = put_s(p, pre >= 0x80u ? "-rc." : pre >= 0x40u ? "-beta." : "-alpha.");
        p = put_u(p, pre & 0x3Fu);
    }
    *p = 0;
}
