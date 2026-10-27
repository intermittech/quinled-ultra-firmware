/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The Digital-M's test jig mode (core/ur_digital_m/udjig.c) on a physical fake: a strip of any length, chip order and
 * channel count whose current follows the bytes it latched, a board idle current, ADC noise and the board's average; the pad's debounce
 * (boot, noise, a floating pad, release, re-entry), the pattern's frames (bounded, the lit byte per slot and order), the
 * currents and the verdict (open data path, a short strip, an overload, the dark slots outside it), status LED; and the
 * `jig` line (core/jig_line.inc) against the earlier formatter, byte for byte (5000 random reports and the extremes). */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "ur_digital_m/udjig.h"
#include "jig_line.inc"

static unsigned checks, failures;
#define CHECK(x) do { checks++; if (!(x)) { failures++; printf("FAIL line %d: %s\n", __LINE__, #x); } } while (0)
static uint32_t rng = 12345u;
static uint32_t rnd(void) { rng = rng * 1664525u + 1013904223u; return rng; }

typedef struct {
    uint32_t px;            /* pixels on the strip */
    uint32_t bpp;           /* the strip's real bytes per pixel */
    uint32_t ma_full;       /* mA per channel at 255 */
    int32_t  idle_ma;       /* the board + strip idle current */
    uint32_t noise;         /* +- mA */
    uint32_t data_ok;       /* the data path works */
    uint32_t over;          /* each lit channel draws this much more (a short / the wrong load) */
} strip_t;

/* the bus current with frame fb (bpp_cfg bytes per pixel, nbytes) latched */
static int32_t current(const strip_t *s, const uint8_t *fb, uint32_t nbytes)
{
    uint32_t i, sum = 0;
    int32_t n = (int32_t)(rnd() % (2u * s->noise + 1u)) - (int32_t)s->noise;
    if (s->data_ok) {
        for (i = 0; i < nbytes && i / s->bpp < s->px; i++) {
            sum += fb[i] * s->ma_full / 255u + (fb[i] ? s->over : 0u);
        }
    }
    return s->idle_ma + (int32_t)sum + n;
}

/* runs test mode on: the pad held low from ms 0; returns the report count; the last report's state in *j. The current goes
 * through the board's ~64 ms average (main_ur_digital_m.c current_service: i_acc, i_avg) */
static uint32_t run(udjig_t *j, const strip_t *s, const uint8_t *ord, uint32_t bpp, uint32_t ms_end, uint32_t live_from)
{
    static uint8_t fb[UJ_PIXELS * 5u];
    uint32_t ms, reports = 0, n = 0, mode = 0;
    int32_t acc = s->idle_ma * 64;
    memset(j, 0, sizeof *j);
    memset(fb, 0, sizeof fb);
    for (ms = 0; ms < ms_end; ms++) {
        uint32_t live = ms >= live_from;
        acc += current(s, fb, n) - (acc >> 6);
        uj_ms(j, ms, 1u, acc >> 6, live, bpp);
        mode += j->say;
        reports += j->rep;
        j->say = j->rep = 0;
        if (live) {
            n = uj_frame(j, ord, bpp, fb);   /* (the glue hands a frame about every ms: it latches within the next) */
        }
    }
    CHECK(mode == 1u);
    return reports;
}

/* one ms of the pad; returns 1 when test mode turned on or off */
static uint32_t pad(udjig_t *j, uint32_t low)
{
    uint32_t say;
    uj_ms(j, 0u, low, 0, 0u, 3u);
    say = j->say;
    j->say = 0;
    return say;
}

static void test_debounce(void)
{
    udjig_t j;
    uint32_t ms, ev = 0;
    memset(&j, 0, sizeof j);
    for (ms = 0; ms < 49u; ms++) {
        ev |= pad(&j, 1u);
    }
    CHECK(!j.on && ev == 0u);                            /* 49 ms low: nothing */
    CHECK(pad(&j, 1u) == 1u && j.on);                    /* the 50th */
    for (ms = 0; ms < 1999u; ms++) {
        ev = pad(&j, (ms % 40u) != 39u);                 /* a 1 ms high every 40 ms: stays on */
        CHECK(ev == 0u && j.on);
    }
    for (ms = 0; ms < 49u; ms++) {
        CHECK(pad(&j, 0u) == 0u);
    }
    CHECK(pad(&j, 0u) == 1u && !j.on);                   /* 50 ms high: off, no reboot */
    for (ms = 0; ms < 100000u; ms++) {
        CHECK(pad(&j, (rnd() >> 7) & 1u) == 0u);         /* a floating pad's noise: never 50 in a row */
        if (failures) break;
    }
    CHECK(!j.on);
}

static void test_frame(void)
{
    static const uint8_t rgb[5] = { 0, 1, 2 }, grb[5] = { 1, 0, 2 }, wrgb[5] = { 3, 0, 1, 2 }, cct[5] = { 1, 0, 2, 3, 4 };
    uint8_t fb[UJ_PIXELS * 5u + 8u];
    udjig_t j;
    uint32_t s, i, n;
    memset(&j, 0, sizeof j);
    memset(fb, 0xEE, sizeof fb);
    CHECK(uj_frame(&j, rgb, 3u, fb) == UJ_PIXELS * 3u);   /* off: the black frame */
    for (i = 0; i < UJ_PIXELS * 3u; i++) CHECK(fb[i] == 0u);
    CHECK(fb[UJ_PIXELS * 3u] == 0xEEu);                  /* bounded: nothing past the pattern's pixels */
    j.on = 1;
    for (s = 0; s < 6u; s++) {
        const uint8_t *ords[4] = { rgb, grb, wrgb, cct };
        const uint32_t bpps[4] = { 3u, 3u, 4u, 5u };
        uint32_t k;
        j.ph = (uint16_t)(s * UJ_SLOT_MS + 7u);
        for (k = 0; k < 4u; k++) {
            uint32_t lit = 0;
            memset(fb, 0xEE, sizeof fb);
            n = uj_frame(&j, ords[k], bpps[k], fb);
            CHECK(n == UJ_PIXELS * bpps[k] && fb[n] == 0xEEu);
            for (i = 0; i < n; i++) {
                uint32_t want = s >= 1u && ords[k][i % bpps[k]] == s - 1u;   /* slot s: logical channel s - 1 */
                CHECK(fb[i] == (want ? UJ_LEVEL : 0u));
                lit += fb[i] != 0u;
            }
            CHECK(lit == (s >= 1u && s - 1u < bpps[k] ? UJ_PIXELS : 0u));   /* one channel of 16 pixels, or none */
        }
    }
}

/* status LED's level at ms now (a call that doesn't move the cycle) */
static uint32_t led(udjig_t *j, uint32_t now)
{
    uj_ms(j, now, j->on, 0, 0u, 3u);
    return j->led;
}

static void test_cycle(void)
{
    static const uint8_t grb[5] = { 1, 0, 2 }, rgbw[5] = { 0, 1, 2, 3 };
    strip_t s = { 300u, 3u, 20u, 35, 3u, 1u, 0u };
    udjig_t j;
    uint32_t r, i, ms;
    const int32_t want = (int32_t)(UJ_PIXELS * (UJ_LEVEL * 20u / 255u));   /* 16 x 5 mA */
    r = run(&j, &s, grb, 3u, 50u + 6u * 1500u, 0u);      /* (live from the start; entered at ms 49) */
    CHECK(r == 6u && j.cycles == 6u);
    for (i = 0; i < 3u; i++) CHECK(j.ma[i] >= want - 4 && j.ma[i] <= want + 4);
    CHECK(j.ma[3] >= -4 && j.ma[3] <= 4 && j.ma[4] >= -4 && j.ma[4] <= 4);   /* dark slots: ~0 */
    CHECK(j.fail == 0u);                                 /* (W1 / W2 outside the verdict on an RGB chip) */
    CHECK(led(&j, 1000u) == 1u && led(&j, 1100u) == 1u);   /* PASS: steady */
    /* RGBW: four slots in the verdict, a long strip draws only the pattern */
    s.bpp = 4u; s.px = 2000u;
    r = run(&j, &s, rgbw, 4u, 50u + 3u * 1500u, 0u);
    CHECK(r == 3u && j.fail == 0u);
    for (i = 0; i < 4u; i++) CHECK(j.ma[i] >= want - 4 && j.ma[i] <= want + 4);
    /* the data path open: every channel of the chip fails, the dark one doesn't count */
    s.bpp = 3u; s.data_ok = 0u;
    r = run(&j, &s, grb, 3u, 50u + 1500u, 0u);
    CHECK(r == 1u && j.fail == 7u);
    CHECK(led(&j, 1000u) != led(&j, 1100u));             /* FAIL: 5 Hz */
    /* a strip of 4 pixels: 4 x 5 mA is above 5 mA: PASS (the jig PC may apply its own limits) */
    s.data_ok = 1u; s.px = 4u;
    r = run(&j, &s, grb, 3u, 50u + 1500u, 0u);
    CHECK(r == 1u && j.fail == 0u && j.ma[0] >= 16 && j.ma[0] <= 24);
    /* an overload: each lit byte 30 mA more */
    s.px = 300u; s.over = 30u;
    r = run(&j, &s, grb, 3u, 50u + 1500u, 0u);
    CHECK(r == 1u && j.fail == 7u);
    /* the pattern not on the strip yet: the cycle waits for it */
    s.over = 0u;
    r = run(&j, &s, grb, 3u, 50u + 1500u, 1000u);
    CHECK(r == 0u && j.cycles == 0u);
    /* no verdict yet: the LED off in the zero slot, on in the others */
    memset(&j, 0, sizeof j);
    j.on = 1;
    for (ms = 0; ms < 1500u; ms++) {
        j.ph = (uint16_t)ms;
        CHECK(led(&j, ms) == (uint32_t)(ms >= UJ_SLOT_MS));
    }
    /* leaving and entering again: the verdict forgotten, a new cycle from the zero */
    j.cycles = 3; j.fail = 1; j.ph = 777;
    for (ms = 0; ms < 50u; ms++) (void)pad(&j, 0u);
    CHECK(!j.on);
    for (ms = 0; ms < 50u; ms++) (void)pad(&j, 1u);
    CHECK(j.on && j.cycles == 0u && j.ph == 0u);
}

/* the formatter before core/jig_line.inc (cli_compact.inc at 8c3ece8), the reference */
static char *old_dec(char *p,uint32_t v,uint32_t width) {
    char t[10]; uint32_t n=0;
    do { t[n++]=(char)('0'+v%10); v/=10; } while(v||n<width);
    while(n) *p++=t[--n];
    return p;
}
static char *old_fixed(char *p,uint32_t v) {
    uint32_t q=v/10; q+=v-q*10>=5;
    p=old_dec(p,q/100,1); *p++='.'; return old_dec(p,q%100,2);
}
static void old_line(char *b,const int32_t ma[5],uint32_t fail,uint32_t cycle,uint32_t supply_mv,const uint32_t *uid,const char *version) {
    char *p=b; uint32_t i; const char *s;
#define TEXT(str) do { s=(str); while(*s) *p++=*s++; } while(0)
    TEXT("jig cycle="); p=old_dec(p,cycle,1); TEXT(fail?" FAIL fail=":" PASS fail="); p=old_dec(p,fail,1);
    TEXT(" a="); for(i=0;i<5;i++) { if(i) *p++='/'; p=old_fixed(p,ma[i]>0?(uint32_t)ma[i]:0); }
    TEXT(" supply="); p=old_fixed(p,supply_mv); TEXT(" uid=");
    for(i=0;i<3;i++) { if(i) *p++=','; p=old_dec(p,uid[i],1); }
    TEXT(" fw="); TEXT(version); TEXT("\r\n"); *p=0;
#undef TEXT
}
static char got[240];
static void capture(const char *s) { memcpy(got, s, strlen(s) + 1u); }

static void test_line(void)
{
    char want[240];
    int32_t ma[5];
    uint32_t uid[3], k, i, same = 0;
    static const uint32_t edge[] = { 0u, 4u, 5u, 9u, 10u, 994u, 995u, 999u, 1000u, 0x7FFFFFFFu, 0x80000000u, 0xFFFFFFFAu,
                                     0xFFFFFFFBu, 0xFFFFFFFFu };
    for (k = 0; k < 5000u + 14u * 14u; k++) {
        uint32_t fail, cycle, supply;
        const char *ver = k & 1u ? "0.7.0" : "0.7.1-beta.3";
        for (i = 0; i < 3u; i++) uid[i] = rnd();
        for (i = 0; i < 5u; i++) ma[i] = (int32_t)rnd() >> (rnd() % 32u);
        fail = k % 32u; cycle = rnd(); supply = rnd() >> (rnd() % 32u);
        if (k >= 5000u) {                                /* the extremes, each against each */
            uint32_t a = edge[(k - 5000u) % 14u], b = edge[(k - 5000u) / 14u];
            for (i = 0; i < 5u; i++) ma[i] = (int32_t)(i & 1u ? a : b);
            supply = a; cycle = b; uid[0] = a; uid[2] = b;
        }
        old_line(want, ma, fail, cycle, supply, uid, ver);
        jig_line(capture, ma, fail, cycle, supply, uid, ver);
        same += strcmp(want, got) == 0;
    }
    CHECK(same == 5000u + 14u * 14u);
    ma[0] = 80; ma[1] = 81; ma[2] = 79; ma[3] = -1; ma[4] = 0; uid[0] = 1; uid[1] = 22; uid[2] = 333;
    jig_line(capture, ma, 0u, 7u, 24015u, uid, "0.7.0");
    CHECK(strcmp(got, "jig cycle=7 PASS fail=0 a=0.08/0.08/0.08/0.00/0.00 supply=24.02 uid=1,22,333 fw=0.7.0\r\n") == 0);
}

int main(void)
{
    test_debounce();
    test_frame();
    test_cycle();
    test_line();
    printf("udjig: %u checks, %u failures\n", checks, failures);
    return failures != 0u;
}
