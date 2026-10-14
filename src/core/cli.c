/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#define CZ_TEXT_DATA                       /* (the compressed text's blob and tokens live here: core/cz.h) */
#include <stdarg.h>
#include <stddef.h>
#include <string.h>
#include "cli.h"
#include "perf.h"
#include "../hal/board.h"
#include "climit.h"
#include "ultra_protocol.h"
#include "presets.h"
#include "testjig.h"
#include "../hal/board.h"               /* BOARD_TESTJIG: the jig's report */
#include "cz.h"

#if CONSOLE_FULL
/* The settings keys, one compressed string of records: the command byte and the key, NUL-terminated; a 0 command
 * byte ends it (0.6.7: without the values' short help; README / wiki: Console list them). key_name / key_next walk
 * the records on the compressed bytes (separators and command bytes stay plain: core/cz.h). */
#define KEYS ZC(keys,                                                                                                \
    "\x01" "protocol\0"  "\x02" "preset\0" "\x03" "dither\0" "\x04" "thresh\0" "\x05" "feedback\0" "\x06" "stagger\0"   \
    "\x07" "floor\0" "\x0d" "minpulse\0" "\x08" "loss\0" "\x0b" "smooth\0" "\x0c" "gamma\0" "\x0e" "scene\0"           \
    "\x0f" "scenefade\0" "\x12" "ilimit\0" "\x13" "ical\0" "\x14" "softstart\0")
typedef char keys_match_config_h[(CMD_PROTOCOL == 0x01 && CMD_PWM_PRESET == 0x02 && CMD_DITHER_MODE == 0x03 &&
                                  CMD_DITHER_THRESH == 0x04 && CMD_FEEDBACK_CH == 0x05 && CMD_STAGGER == 0x06 &&
                                  CMD_DIM_FLOOR == 0x07 && CMD_MIN_PULSE == 0x0d && CMD_SIGNAL_LOSS == 0x08 &&
                                  CMD_SMOOTHING == 0x0b && CMD_GAMMA == 0x0c && CMD_SCENE == 0x0e &&
                                  CMD_SCENE_FADE == 0x0f && CMD_ISENSE_LIMIT == 0x12 && CMD_ISENSE_CAL == 0x13 &&
                                  CMD_SOFT_START == 0x14) ? 1 : -1];
#endif

/* ------------------------------------------------------------------------------------ compressed text (cz.h) */
/* z: a Z(...) offset. The decoder's state: the next compressed byte and the right halves of the tokens being
 * expanded (at most CZ_DEPTH). One character per call; 0 at the string's end. */
typedef struct {
    const uint8_t *p;
    uint32_t n;
    uint8_t r[CZ_DEPTH];
} cz_dec_t;

#define cz_at(z) (cz_text + CZ_BIAS + (z))

static uint32_t cz_next(cz_dec_t *d)
{
    uint32_t b = d->n ? d->r[--d->n] : *d->p++;
    while (b & 0x80u) {                    /* a token: its left half now, its right half next */
        const uint8_t *e = cz_pair[b & 0x7Fu];
        d->r[d->n++] = e[1];
        b = e[0];
    }
    return b;
}

/* the string after the terminator of z (on the compressed bytes: a NUL is never part of a token) */
static int skip(int z)
{
    while (*cz_at(z++)) {
    }
    return z;
}

/* the i-th of the NUL-separated strings in z (the console's name lists) */
static int nth(int z, uint32_t i)
{
    while (i--) {
        z = skip(z);
    }
    return z;
}

#if CONSOLE_FULL
/* a key record k (at its command byte): its command, name, help, the record after it */
#define key_cmd(k)  (*cz_at(k))
#define key_name(k) ((k) + 1)
#define key_next(k) skip((k) + 1)

/* a == the text z */
static int cz_eq(const char *a, int z)
{
    cz_dec_t d;
    uint32_t ch;
    d.p = cz_at(z);
    d.n = 0;
    while ((ch = cz_next(&d)) != 0 && ch == (uint8_t)*a) {
        a++;
    }
    return ch == (uint8_t)*a;
}
#endif

/* ---------------------------------------------------------------------------------------------------- output */
/* The console's one formatter: f (a Z(...) text) with one argument per %-code. Readable units on the console (V, A,
 * W, %, s); telemetry and the data structures stay raw (mV, mA, 16-bit outputs).
 *   %z text (a Z(...))   %s string (RAM)       %u decimal (uint32)       %t s as [Nd ]h:mm:ss           %% a '%'
 *   n / d rounded with 1 or 2 decimals:
 *   %v uint32 / 10, 2 decimals (mV in V, 10 mWh in Wh)    %a int32 mA in A, 2 decimals (negative: 0)
 *   %V int32 / 10, 2 decimals, signed                      %p 16-bit level in %, 1 decimal
 *   %k uint32 / 10, 1 decimal (permille in %, 100 ms in s)  %K int32 / 10, 1 decimal, signed (0.1 C)
 *   %h uint32 / 100, 1 decimal (ms in s, 10 mW in W)       %H uint32 / 100, 2 decimals
 *   %q Q16 of the period in %, 1 decimal
 *   five values, "/" between: %P uint16[5] levels (%p), %A int32[5] mA (%a), %e int8[5] signed % (ULTRA_HE_NA: -) */
#define CF_BUF 32u                         /* (0.6.4: 48; 32 keeps cf's frame at 192 B with the two decoders: the
                                              deepest stack path, save_now > ... > cli_say > cf, stays 784 B) */

typedef struct {
    cli_t *c;
    uint32_t n;
    char b[CF_BUF];
} cf_out_t;

static void cf_flush(cf_out_t *o)
{
    o->b[o->n] = 0;
    if (o->n) {
        o->c->write(o->b);
    }
    o->n = 0;
}

static void cf_ch(cf_out_t *o, char ch)
{
    if (o->n + 1u >= CF_BUF) {
        cf_flush(o);
    }
    o->b[o->n++] = ch;
}

/* v in decimal, at least `width` digits (zero-padded) */
static void cf_dec(cf_out_t *o, uint32_t v, uint32_t width)
{
    char t[10];
    uint32_t k = 0;
    do {
        t[k++] = (char)('0' + v % 10u);
        v /= 10u;
    } while (v || k < width);
    while (k) {
        cf_ch(o, t[--k]);
    }
}

/* n / d rounded, with dec (1 or 2) decimals */
static void cf_fix(cf_out_t *o, uint32_t n, uint32_t d, uint32_t dec)
{
    uint32_t p = dec == 2u ? 100u : 10u, v = (n + d / 2u) / d;
    cf_dec(o, v / p, 1u);
    cf_ch(o, '.');
    cf_dec(o, v % p, dec);
}

/* the sign of v ("-" when negative), returns |v| */
static uint32_t cf_neg(cf_out_t *o, int32_t v)
{
    if (v < 0) {
        cf_ch(o, '-');
        return (uint32_t)-v;
    }
    return (uint32_t)v;
}

static uint32_t clamp0(int32_t v)
{
    return v > 0 ? (uint32_t)v : 0u;
}

/* the fixed-point codes, one table (0.6.6, review round 5: a case each was ~150 B more): the code, how the value's
 * sign is treated (CF_CLAMP: negative shows as 0; CF_NEG: a '-' and its magnitude), x 1000 for the Q16 levels, the
 * divisor and the decimals of cf_fix. %P / %A take the %p / %a entries per element. */
#define CF_CLAMP 1u
#define CF_NEG   2u
#define CF_K1000 4u
typedef struct {
    char    code;
    uint8_t kind;                          /* CF_* */
    uint8_t dec;                           /* decimals */
    uint8_t div;                           /* cf_divs[] index */
} cf_num_t;
static const uint32_t cf_divs[5] = { 1u, 10u, 100u, 65535u, 65536u };
static const cf_num_t cf_nums[9] = {
    { 'v', 0u, 2u, 1u }, { 'a', CF_CLAMP, 2u, 1u }, { 'V', CF_NEG, 2u, 1u }, { 'p', CF_K1000, 1u, 3u },
    { 'k', 0u, 1u, 0u }, { 'K', CF_NEG, 1u, 0u }, { 'h', 0u, 1u, 2u }, { 'H', 0u, 2u, 2u }, { 'q', CF_K1000, 1u, 4u },
};
#define CF_NUM_P (&cf_nums[3])
#define CF_NUM_A (&cf_nums[1])

static void cf_num(cf_out_t *o, const cf_num_t *n, uint32_t v)
{
    if (n->kind & CF_CLAMP) {
        v = clamp0((int32_t)v);
    } else if (n->kind & CF_NEG) {
        v = cf_neg(o, (int32_t)v);
    }
    if (n->kind & CF_K1000) {
        v *= 1000u;
    }
    cf_fix(o, v, cf_divs[n->div], n->dec);
}

static void cf(cli_t *c, int f, ...)
{
    cf_out_t o;
    cz_dec_t d, t;
    va_list ap;
    uint32_t i, ch;
    o.c = c;
    o.n = 0;
    d.p = cz_at(f);
    d.n = 0;
    va_start(ap, f);
    while ((ch = cz_next(&d)) != 0) {
        if (ch != '%') {
            cf_ch(&o, (char)ch);
            continue;
        }
        ch = cz_next(&d);
        for (i = 0; i < 9u && cf_nums[i].code != (char)ch; i++) {
        }
        if (i < 9u) {
            cf_num(&o, &cf_nums[i], va_arg(ap, uint32_t));
            continue;
        }
        switch (ch) {
        case 'z':
            t.p = cz_at(va_arg(ap, int));
            t.n = 0;
            while ((ch = cz_next(&t)) != 0) {
                cf_ch(&o, (char)ch);
            }
            break;
        case 's':
            cf_flush(&o);
            c->write(va_arg(ap, const char *));
            break;
        case 'u': cf_dec(&o, va_arg(ap, uint32_t), 1u); break;
        case 't': {
            uint32_t sec = va_arg(ap, uint32_t), m = sec / 60u, h = m / 60u;
            if (h >= 24u) {
                cf_dec(&o, h / 24u, 1u);
                cf_ch(&o, 'd');
                cf_ch(&o, ' ');
            }
            cf_dec(&o, h % 24u, 1u);
            cf_ch(&o, ':');
            cf_dec(&o, m % 60u, 2u);
            cf_ch(&o, ':');
            cf_dec(&o, sec % 60u, 2u);
            break;
        }
        case 'P': case 'A': case 'e': {
            const void *p = va_arg(ap, const void *);
            for (i = 0; i < 5u; i++) {
                if (i) {
                    cf_ch(&o, '/');
                }
                if (ch == 'P') {
                    cf_num(&o, CF_NUM_P, ((const uint16_t *)p)[i]);
                } else if (ch == 'A') {
                    cf_num(&o, CF_NUM_A, (uint32_t)((const int32_t *)p)[i]);
                } else if (((const int8_t *)p)[i] == ULTRA_HE_NA) {
                    cf_ch(&o, '-');
                } else {
                    cf_dec(&o, cf_neg(&o, ((const int8_t *)p)[i]), 1u);
                }
            }
            break;
        }
        default:
            cf_ch(&o, '%');                /* "%%" */
            break;
        }
    }
    va_end(ap);
    cf_flush(&o);
}

void cli_say(cli_t *c, int z)
{
    cf(c, z);
}

/* ---------------------------------------------------------------------------------------------------- reports */
#if BOARD_TESTJIG
void cli_test_report(cli_t *c, const int32_t ma[5], uint32_t fail, uint32_t cycle, uint32_t supply_mv)
{
    const uint32_t *u = c->st->uid;
    cf(c, ZJ(jig, "jig cycle=%u %z fail=%u a=%A supply=%v uid=%u,%u,%u fw=%s\r\n"), cycle,
       fail ? ZJ(jig_fail, "FAIL") : ZJ(jig_pass, "PASS"), fail, ma, supply_mv, u[0], u[1], u[2], c->st->version);
}
#endif

void evlog_boot(evlog_t *l, uint32_t keep, uint32_t prev_up_s)
{
    uint32_t i;
    if (!keep || l->magic != EVL_MAGIC) {
        uint8_t *p = (uint8_t *)l;
        for (i = 0; i < sizeof(*l); i++) {
            p[i] = 0;
        }
        l->magic = EVL_MAGIC;
        return;
    }
    for (i = 0; i < EVLOG_N; i++) {
        l->e[i].t_s -= prev_up_s + 1u;               /* (u32: the ages, now - t_s, go on; the reset's own second) */
    }
}

void cli_banner(cli_t *c)
{
    const cli_status_t *s = c->st;
#if CONSOLE_FULL
    cf(c, ZC(banner, "QuinLED dig2analog-ultra v%s (GPL-3.0)\r\n"), s->version);   /* (0.6.7: the copyright and the
                                                       source in the repository) */
#else
    cf(c, Z(banner_rel, "QuinLED %s v%s, (C) 2026 QuinLED, GPL-3.0, github.com/intermittech/quinled-ultra-firmware\r\n"),
       BOARD == BOARD_DP_RX ? "Diff-Power ultra receiver" : "dig2analog-ultra", s->version);   /* (0.6.7: the release)
                                                                                                   */
#endif
    if (s->bl_version != 0u) {
        cf(c, Z(bl_line, "bootloader rev %u (build %s), hardware %u revision %u, "), s->bl_version, s->bl_build,
           s->hw_id, s->hw_rev);
    }
    cf(c, Z(app_line, "app %z\r\n"),
       nth(Z(app_states, "standalone build (no bootloader)\0trial (not confirmed yet)\0confirmed"),
           s->app_state < 3u ? s->app_state : 0u));
}

#if CONSOLE_FULL
/* a key's value: the settings field of its command (a table by command, 0.6.6: the switch was ~130 B more), else the
 * scene's / the status's (the commands main.c keeps outside settings_t) */
static uint8_t get_key(const cli_t *c, uint8_t cmd)
{
#define SF(f) ((uint8_t)offsetof(settings_t, f))
#define SX 0xFFu                           /* not a settings_t field (below) */
    static const uint8_t ofs[CMD_SOFT_START] = {   /* [cmd - 1] */
        SF(protocol), SF(pwm_preset), SF(dither_mode), SF(dither_thresh), SF(feedback_ch), SF(stagger),
        SF(dim_floor), SF(signal_loss), SF(gamma), SF(gamma), SF(smoothing), SF(gamma), SF(min_pulse), SX, SX,
        SF(gamma), SF(gamma), SX, SX, SX,  /* (IDENTIFY / FACTORY_RESET / PERIOD_*: not keys; gamma, as the default) */
    };
#undef SF
    uint32_t o = (uint32_t)(cmd - 1u) < CMD_SOFT_START ? ofs[cmd - 1u] : SX;
    if (o != SX) {
        return ((const uint8_t *)c->set)[o];
    }
    switch (cmd) {
    case CMD_SCENE:         return (uint8_t)(c->scene && c->scene->mask != 0u);
    case CMD_SCENE_FADE:    return c->scene ? c->scene->fade : 0u;
    case CMD_ISENSE_LIMIT:  return c->st ? c->st->ilimit50 : 0u;
    case CMD_ISENSE_CAL:    return 0u;
    case CMD_SOFT_START:    return c->st ? c->st->softstart : 1u;
    default:                return c->set->gamma;
    }
#undef SX
}

/* parse an unsigned decimal; returns -1 if none */
static int32_t parse_u(const char **p)
{
    int32_t v = -1;
    while (**p == ' ') {
        (*p)++;
    }
    while (**p >= '0' && **p <= '9') {
        v = (v < 0 ? 0 : v * 10) + (**p - '0');
        if (v > 1000000) {
            return -1;
        }
        (*p)++;
    }
    return v;
}

static void result(cli_t *c, cfg_result_t r)
{
    cf(c, r.event == CFG_ACK ? ZC(ok_crlf, "ok\r\n") : r.event == CFG_SAME ? ZC(ok_same, "ok (unchanged)\r\n")
                                                   : ZC(err_value, "error: invalid value\r\n"));
}
#endif

/* the overrun counters (main.c ovr) in format f (a Z text), when any is not 0 */
static void ovr_out(cli_t *c, int f)
{
    const volatile uint32_t *o = c->st->ovr;
    if (o && (o[0] | o[1] | o[2])) {
        cf(c, f, o[0], o[1], o[2]);
    }
}

/* (0.6.5, D2) the event log (cli.h evlog_t), newest first: "event 1=0:00:12 ago, short, 9.80 A" */
static void events(cli_t *c)
{
    const cli_status_t *s = c->st;
    uint32_t k;
    for (k = 0; s->ev && k < EVLOG_N; k++) {
        const evl_ev_t *e = &s->ev->e[(s->ev->head - k) & (EVLOG_N - 1u)];
        if (e->type == EVL_NONE) {
            break;
        }
        cf(c, Z(ev_line, "event %u=%t ago, %z, %a A\r\n"), k + 1u, s->uptime_s - e->t_s,
           nth(Z(ev_names, "limit\0overload\0short\0supply dip\0calibration\0calibration incomplete\0off: retries used up\0"
                           "option bytes unreadable\0bootloader protected\0protection not as written\0too hot"),
               e->type - 1u),
           (int32_t)e->ma);
    }
}

/* `status` in parts (0.6.5, review round 4 #39): one part per call, each under 512 bytes, so the main loop writes a
 * part only when the console has room for it (main.c, uart_room) and never waits for the UART (a whole status of
 * ~1,400 bytes held the main loop ~44 ms before). The values: the snapshot taken at the command (cli_status_t), the
 * pointed-to logs as they are when their part is written. Returns 1 while parts remain. */
#define STATUS_PARTS (11u + MON_EXTRA)   /* (0.6.6: + the calibration history's, M2; 0.6.9: + the settings') */
#if MON_EXTRA
/* (0.6.6, M2) the calibration history (core/calhist.h): each stored test at 100 %, then the newest against the first */
static void calhist_lines(cli_t *c, const calhist_t *h)
{
    int32_t a[5];
    int8_t t[5];
    uint32_t k, i;
    for (k = 0; h && k < h->n; k++) {
        for (i = 0; i < 5u; i++) {
            a[i] = h->e[k].ma[i];
        }
        cf(c, ZU(cal_hist, "cal_history %u h lit=%A A\r\n"), (uint32_t)h->e[k].lit_h, a);
    }
    if (h && h->n) {
        k = calhist_trend(h, t);
        cf(c, ZU(cal_trend, "cal_trend R/G/B/W1/W2=%e %% over %u h lit, %u tests\r\n"), t, k, (uint32_t)h->tests);
    }
}
#endif

static uint32_t status_part(cli_t *c, uint32_t part)
{
    const cli_status_t *s = c->st;
    const int proto = Z(proto, "none\0WS2915 32-bit header\0WS2915 26-bit header\0WS2805\0Ultra");
    const int cause = Z(cause, "-\0short\0overload");
    const int empty = Z(empty, "");
    uint32_t i;
#if MON_EXTRA
    if (part == 6u) {                      /* (0.6.6, M2) its own part, after the calibration's (that one is full) */
        calhist_lines(c, s->ch);
        return 1;
    }
    part -= part > 6u;
#endif
    switch (part) {
    case 0:
    cli_banner(c);
    cf(c, Z(st_head, "uptime=%t\r\nreset=%z%z%z%z\r\nstack=%u B used\r\nprotocol=%z, active %z%z, stored %z\r\n"
                     "frames=%u\r\nfps=%u\r\ndecoded=%u\r\n"),
       s->uptime_s, (s->reset_flags & 0x20000000u) ? Z(rst_wd, "watchdog ") : empty,
       (s->reset_flags & 0x08000000u) ? Z(rst_por, "power-on ") : empty,
       (s->reset_flags & 0x04000000u) ? Z(rst_pin, "pin ") : empty,
       (s->reset_flags & 0x10000000u) ? Z(rst_sw, "software ") : empty, s->stack_max,
       nth(Z(mode, "auto\0WS2915\0WS2805\0Ultra"), s->proto_mode < 4u ? s->proto_mode : 0u),
       nth(proto, s->proto_locked < 5u ? s->proto_locked : 0u),
       s->proto_provisional ? Z(provisional, " (stored, not confirmed yet)") : empty,
       nth(proto, c->set->last_proto < 5u ? c->set->last_proto : 0u),
       s->frames, s->fps, s->decoded);
    break;
    case 1:                                /* (0.6.5 D8: part 0 reached 693 bytes with the longest values) */
    cf(c, Z(st_cpu, "cpu=%k %%, peak %k %%\r\n"), (uint32_t)s->cpu_pm, (uint32_t)s->cpu_peak);
    if (s->bc.magic == BC_MAGIC && !(s->reset_flags & 0x08000000u)) {   /* (0.6.5, D1) not after a power-on */
        cf(c, Z(st_bc, "before_reset=%t up, pc %u, fault %u\r\n"), s->bc.up_s, s->bc.pc, s->bc.cause);
    }
    for (i = 1; i < 6u; i++) {
        if (s->dec_err[i]) {
            cf(c, Z(st_decerr, "decode_error %z=%u\r\n"), nth(Z(err, "\0no edge\0no gap\0short\0truncated\0pulse"), i),
               s->dec_err[i]);
        }
    }
    if (s->ultra_headers || s->ultra_tail_err) {
        cf(c, Z(st_ultra, "ultra_headers=%u\r\nultra_tail_errors=%u\r\n"), s->ultra_headers, s->ultra_tail_err);
    }
    ovr_out(c, Z(st_ovr, "overruns=%u captures, %u frame ends, %u ms late\r\n"));   /* (main.c ovr) */
    break;
    case 2:                                /* (0.6.9) its own part: 218 bytes with the longest values */
    {                                      /* (0.6.9) the settings in effect by their console keys ('*': an Ultra
                                              CONFIG over the stored value), the retry policy, and the save state:
                                              pending, why (main.c save_why) */
        static const uint8_t so[11] = {
            offsetof(settings_t, protocol), offsetof(settings_t, pwm_preset), offsetof(settings_t, dither_mode),
            offsetof(settings_t, dither_thresh), offsetof(settings_t, dim_floor), offsetof(settings_t, min_pulse),
            offsetof(settings_t, stagger), offsetof(settings_t, signal_loss), offsetof(settings_t, feedback_ch),
            offsetof(settings_t, smoothing), offsetof(settings_t, gamma) };
        const int names = Z(set_names, "protocol\0preset\0dither\0thresh\0floor\0minpulse\0stagger\0loss\0"
                                       "feedback\0smooth\0gamma");
        cf(c, Z(set_head, "settings="));
        for (i = 0; i < 11u; i++) {
            uint32_t e = ((const uint8_t *)c->eff)[so[i]];
            cf(c, Z(set_kv, "%z %u%z, "), nth(names, i), e, e != ((const uint8_t *)c->set)[so[i]] ? Z(star, "*") : empty);
        }
        cf(c, Z(set_tail, "softstart %u, retry %u x %u s\r\nsave=%z%z\r\n"), (uint32_t)s->softstart != 0u,
           (uint32_t)s->retry_n, (uint32_t)s->retry_s, s->save_why ? Z(sv_pend, "pending: ") : Z(sv_ok, "ok"),
           nth(Z(sv_why, "\0debounce\0waiting for dark\0budget\0error hold-off\0supply"), s->save_why < 6u ?
               s->save_why : 0u));
    }
    break;
    case 3:                                /* (0.6.5 merge: every part under 470 bytes with the longest values) */
    cf(c, Z(st_cfg, "config_accepted=%u\r\npwm_hz=%u\r\npwm_steps=%u\r\nflash_saves=%u\r\n"), s->config_frames,
       (uint32_t)(F_TIM_HZ / s->steps), s->steps, s->saves);
#if MON_EXTRA
    if (s->fl_pend | s->fl_errs | s->fl_rej) {
        /* (0.6.5, D5) "flash: settings pending, calibration rejected 1, errors 2 (WRP)": stores with a write waiting,
         * the records their loaders rejected at boot, the failed operations (the last one's code) */
        const int store = ZU(fl_store, "settings\0calibration\0scene\0lifetime\0power-on effect");
        uint32_t r = s->fl_rej;
        cf(c, ZU(fl_head, "flash:"));
        for (i = 0; i < 5u; i++, r >>= 6) {
            if (s->fl_pend & (1u << i)) {
                cf(c, ZU(fl_pend, " %z pending,"), nth(store, i));
            }
            if (r & 63u) {
                cf(c, ZU(fl_rej, " %z rejected %u,"), nth(store, i), r & 63u);
            }
        }
        cf(c, ZU(fl_errs, " errors %u (%z)\r\n"), (uint32_t)s->fl_errs,   /* (flashrec.h FLASH_ERR_*, 0.6.5: VDD) */
           nth(ZU(fl_codes, "-\0-\0WRP\0EOP\0KEY\0VERIFY\0VDD"), s->fl_err < 7u ? s->fl_err : 0u));
    }
#endif
    if (s->stagger_moves || s->ripple_ma) {    /* the offsets in % of the period */
#if MON_EXTRA
        cf(c, ZU(st_stagger, "stagger_tim2=%q %%\r\nstagger_tim3=%q %%\r\nstagger_moves=%u\r\nripple_rms=%a A\r\n"
                             "c10_load=%u %%, 1 min %u %%, peak %u %%\r\n"),   /* (0.6.5, M5: % of C10's ripple rating;
                                                                                 0 = not known) */
           (uint32_t)s->stagger_o2, (uint32_t)s->stagger_o3, s->stagger_moves, (int32_t)s->ripple_ma,
           (uint32_t)s->cap.now, (uint32_t)s->cap.avg, (uint32_t)s->cap.peak);
#else
        cf(c, ZL(st_stagger, "stagger_tim2=%q %%\r\nstagger_tim3=%q %%\r\nstagger_moves=%u\r\nripple_rms=%a A\r\n"),
           (uint32_t)s->stagger_o2, (uint32_t)s->stagger_o3, s->stagger_moves, (int32_t)s->ripple_ma);
#endif
    }
    break;
    case 4:
    {
        cf(c, Z(st_current, "out R/G/B/W1/W2=%P %%\r\ncurrent=%a A\r\ncurrent_peak=%a A\r\ncurrent_zero=%a A\r\n"
                            "current_limit=%a A\r\ncurrent_scale=%k %%\r\ncurrent_trips=%u\r\n"
                            "current_state=%z, last trip %z\r\nchannel R/G/B/W1/W2=%A A (%z)\r\n"),
           s->out, s->i_ma, s->i_peak_ma, s->i_zero_ma, (int32_t)s->i_limit_ma, s->i_k_permille, s->i_trips,
           s->i_cal ? Z(calibrating, "calibrating") : (s->i_tripped ? Z(tripped_off, "tripped, outputs off")
                                                                    : Z(ok, "ok")),
           nth(cause, s->i_trip_cause < 3u ? s->i_trip_cause : 0u), s->i_ch_ma,
           nth(Z(model, "none\0learned\0calibrated"), s->i_model < 3u ? s->i_model : 0u));
    }
    if (s->cal_fail != 0u) {               /* (0.6.9, A2) the last test stopped: the stored one (next part) still
                                              counts (here: the calibration's part reached 511 bytes) */
        cf(c, Z(cal_fail, "calibration_last=incomplete: %z\r\n"),
           nth(Z(cal_why, "trip\0limit\0supply"), s->cal_fail - 1u < 3u ? s->cal_fail - 1u : 0u));
    }
    break;
    case 5:
    {
        if (s->cal.levels) {
            uint32_t l;
            cf(c, Z(cal_head, "calibration="));
            for (l = 0; l < s->cal.levels && l < 4u; l++) {
                cf(c, Z(cal_pct, "%u%z"), s->cal.pct[l], l + 1u < s->cal.levels ? Z(slash, "/") : Z(pct_comma, " %, "));
            }
            cf(c, Z(cal_count, "%u tests stored\r\n"), s->cal_count);
            for (i = 0; i < 5u; i++) {
                cf(c, Z(cal_ch, "calibration %z="), nth(Z(ch5, "R\0G\0B\0W1\0W2"), i));
                for (l = 0; l < s->cal.levels && l < 4u; l++) {
                    cf(c, Z(cal_ma, "%a%z"), (int32_t)s->cal.ma[i][l],
                       l + 1u < s->cal.levels ? Z(slash, "/") : Z(a_crlf, " A\r\n"));
                }
            }
        }
    }
    if (s->mcu_temp_c10 == -32768) {
        cf(c, Z(mcu_na, "mcu_temp=n/a\r\n"));
    } else {
        cf(c, Z(mcu_line, "mcu_temp=%K C (die), vdda=%v V, peak %K C %t ago\r\n"), (int32_t)s->mcu_temp_c10,
           (uint32_t)s->vdda_mv, (int32_t)s->mcu_peak_c10, s->uptime_s - s->mcu_peak_s);
    }
    cf(c, Z(supply, "supply=%v V (min %v V, max %v V)\r\n"), s->vs_supply_mv, s->vs_supply_min_mv, s->vs_supply_max_mv);
#if MON_EXTRA
    if (s->hwd) {                          /* (0.6.5, D6) "adc_stale=current 0, voltage 3; dma_errors
                                              ch1-7=0/0/0/0/0/0/1" (hal/hw.h hw_diag), only when one counted */
        const int lbl = ZU(hwd_lbl, "adc_stale=current \0, voltage \0; dma_errors ch1-7=\0/");
        uint32_t any = 0;
        for (i = 0; i < HW_DIAG_N; i++) {
            any |= s->hwd[i];
        }
        for (i = 0; any != 0u && i < HW_DIAG_N; i++) {
            cf(c, i + 1u < HW_DIAG_N ? ZU(hwd_v, "%z%u") : ZU(hwd_end, "%z%u\r\n"), nth(lbl, i < 3u ? i : 3u),
               (uint32_t)s->hwd[i]);
        }
    }
#endif
    break;
    case 6:
    if (s->dip) {                          /* supply dips (core/vdip.h): the deepest, then the log, newest first */
        const vdip_t *d = s->dip;
        const vdip_ev_t *e = d->deep.ref_x ? &d->deep : 0;
        uint32_t k;
        cf(c, Z(st_imp, "supply_impedance=%u mohm (%u steps)\r\n"), vdip_z_mohm(d), (uint32_t)d->zsteps);
        cf(c, Z(dips, "supply_dips=%u%z%v V"), (uint32_t)d->count,
           d->in_dip ? Z(dip_now, " (one now), reference ") : Z(dip_ref, ", reference "), s->dip_ref_mv);
        for (k = 0; e != 0; e = vdip_event(d, k++)) {
            cf(c, k ? Z(dip_n, "\r\nsupply_dip %u=") : Z(dip_deep, "\r\nsupply_dip_deepest="), k);
            cf(c, Z(dip_ev, "%t ago, %v V (%u %%), %u ms, %a A"), s->uptime_s - e->t_s, vsense_mv(s->dip_vs, e->min_x, 1u),
               vdip_pct(e), (uint32_t)e->dur_ms, (int32_t)e->i_ma);
        }
        cf(c, Z(crlf, "\r\n"));
    }
    break;
    case 7:
    /* V x I: 10 mV x mA / 100 keeps it in 32 bits (75 V x 65 A): mW, shown in W */
#if MON_EXTRA
    cf(c, ZU(st_input, "input=%h W\r\nenergy=%v Wh since boot\r\nenergy R/G/B/W1/W2=%A Wh\r\ntp8=%v V\r\n"
                       "test_mode=%z\r\nboard_limit="), s->vs_supply_mv / 10u * clamp0(s->i_ma) / 100u, s->en->boot_mwh,
       (const int32_t *)s->en->ch_mwh, s->vs_aux_mv, s->test_mode ? Z(on, "on") : Z(off, "off"));   /* (0.6.5, M3: per
                                                                                                    channel) */
#else
    cf(c, ZL(st_input, "input=%h W\r\nenergy=%v Wh since boot\r\ntp8=%v V\r\ntest_mode=%z\r\nboard_limit="),
       s->vs_supply_mv / 10u * clamp0(s->i_ma) / 100u, s->en->boot_mwh, s->vs_aux_mv,
       s->test_mode ? Z(on, "on") : Z(off, "off"));
#endif
    if (s->lim_ma == 0xFFFFu) {
        cf(c, Z(none, "none"));
    } else {
        cf(c, Z(lim, "%a A (%z)"), (int32_t)s->lim_ma,
           s->lim_src <= CLIMIT_SRC_NONE ? nth(Z(lim_src, "board\0user\0fuse\0none"), s->lim_src) : Z(qmark, "?"));
    }
    if (s->lim_conn_ma) {
        cf(c, Z(lim_conn, ", connected %a A: %z"), (int32_t)s->lim_conn_ma,
           s->lim_need == CLIMIT_NEED_NO ? Z(lim_no, "no limit needed\r\n") : Z(lim_may, "the limit may engage\r\n"));
    } else {
        cf(c, Z(lim_unknown, ", connected not known\r\n"));
    }
    /* the overcurrent signal on D9 (core/ocled.h) */
    if (s->oc_show == 0u) {
        cf(c, Z(oc_off, "overcurrent_led=off"));
    } else {
        cf(c, s->oc_show == 2u ? Z(oc_trip, "overcurrent_led=trip (%z), since %t (%u s ago), ")
                               : Z(oc_limit, "overcurrent_led=limit%z, since %t (%u s ago), "),
           s->oc_show == 2u ? nth(cause, s->oc_cause < 3u ? s->oc_cause : 0u) : empty, s->oc_since_s,
           s->uptime_s - s->oc_since_s);
        if (s->oc_show == 2u ? s->oc_tripped : s->oc_limiting) {
            cf(c, s->oc_show == 2u ? Z(oc_outputs_off, "outputs off") : Z(oc_limit_active, "board limit active"));
        } else {
            cf(c, Z(oc_hold, "hold %h s"), s->oc_hold_ms);
        }
    }
    cf(c, Z(crlf, "\r\n"));
    break;
    case 8:
    /* the power events (core/pwrev.h): what a Diff-Power ultra receiver reports on its STATUS / POWER pages */
    if (s->pw) {
        const pwrev_t *g = s->pw;
        uint8_t st = pwrev_state(g), lat = pwrev_latched(g);
        cf(c, Z(pw_state, "power_state=%z%z%z%z"),
           (st & (ULTRA_ST_LIMITING | ULTRA_ST_TRIPPED | ULTRA_ST_CH_LIMITING)) ? empty : Z(ok, "ok"),
           (st & ULTRA_ST_LIMITING) ? Z(pw_limiting, "limiting") : empty,
           (st & ULTRA_ST_TRIPPED) ? ((st & ULTRA_ST_LIMITING) ? Z(pw_c_tripped, ", tripped") : Z(pw_tripped, "tripped"))
                                   : empty,
           (st & ULTRA_ST_CH_LIMITING) ? ((st & (ULTRA_ST_LIMITING | ULTRA_ST_TRIPPED))
                                          ? Z(pw_c_chlim, ", channel-limiting") : Z(pw_chlim, "channel-limiting"))
                                       : empty);
        if (st & ULTRA_ST_TRIPPED) {
            cf(c, Z(pw_retry, " (retry in %u s)"), (g->in.retry_ms + 999u) / 1000u);   /* as the POWER page: whole
                                                                                            seconds, up */
        } else if (st & ULTRA_ST_LIMITING) {
            cf(c, Z(pw_factor, " (factor %k %%)"), (uint32_t)g->in.factor * 1000u / 65535u);
        }
        cf(c, Z(pw_events, "\r\npower_events=limit %u, overload trips %u, short trips %u%z%z%z%z%z\r\npower_last_limit="),
           (uint32_t)g->limit_events, (uint32_t)g->ovl_trips, (uint32_t)g->short_trips,
           lat ? Z(pw_latched, ", latched:") : Z(pw_nothing, ", nothing latched"),
           (lat & ULTRA_PW_EV_LIMITED) ? Z(pw_l_limit, " limit") : empty,
           (lat & ULTRA_PW_EV_OVERLOAD) ? Z(pw_l_overload, " overload") : empty,
           (lat & ULTRA_PW_EV_SHORT) ? Z(pw_l_short, " short") : empty,
           (lat & ULTRA_PW_EV_SUPPLY_DIP) ? Z(pw_l_dip, " dip") : empty);
        if (g->limit_age_s == PWREV_AGE_NONE) {
            cf(c, Z(none, "none"));
        } else {
            cf(c, Z(pw_ago, "%u s ago, %a A"), g->limit_age_s, (int32_t)g->event_ma);
        }
        if (g->trip_age_s == PWREV_AGE_NONE) {
            cf(c, Z(pw_trip_none, "\r\npower_last_trip=none\r\n"));
        } else {
            cf(c, Z(pw_trip, "\r\npower_last_trip=%u s ago, %z, %a A\r\n"), g->trip_age_s,
               nth(cause, g->last_cause < 3u ? g->last_cause : 0u), (int32_t)g->trip_ma);
        }
    }
    {                                      /* the lifetime record (core/energy.h); trips: short / overload */
        const life_rec_t *r = &s->en->lt;
        cf(c, Z(lifetime, "lifetime=%v kWh, powered %k h, lit %k h, %u power-ups, trips %u / %u, dips %u, "
                          "max %K C / %k A, wd %u, updates %u, %u records\r\n"),
           energy_total_wh(s->en), r->on_s / 360u, r->lit_s / 360u, r->ups, (uint32_t)r->trips[1],
           (uint32_t)r->trips[0], r->dips, (int32_t)r->peak_t * 5, (uint32_t)r->peak_a, (uint32_t)r->wd,
           (uint32_t)r->updates, r->writes);
    }
    break;
    default:
    {                                      /* (0.6.9, B2) the lifetime record's fault memory (here: the lifetime's part
                                              reached 516 bytes) */
        const life_rec_t *r = &s->en->lt;
        if (r->trip | r->dip_pct | r->rst) {
            cf(c, Z(faults, "faults=trip %z %h A at %u h lit, dip %u %% %u ms %k A, reset %z %u after %u h\r\n"),
               nth(Z(trip_k, "-\0short\0overload\0-"), r->trip & 3u), (uint32_t)(r->trip >> 2) * 50u,
               (uint32_t)r->trip_h, (uint32_t)r->dip_pct, (uint32_t)r->dip_cs * 10u, (uint32_t)r->dip_a,
               nth(Z(rst_k, "-\0watchdog\0fault\0-"), r->rst & 3u), (uint32_t)(r->rst >> 2), (uint32_t)r->rst_h);
        }
    }
    if (s->hl) {                           /* channel health (core/health.h) */
        const health_t *h = s->hl;
        uint32_t k;
        if (h->flags & ULTRA_HE_F_TESTED) {
            cf(c, Z(he_test, "health_test R/G/B/W1/W2="));
            for (i = 0; i < 5u; i++) {
                cf(c, nth(Z(he_cls, "-\0none\0ok\0FAULT"), h->cls[i] & 3u));
                for (k = 0; k < 3u; k++) {
                    if (h->cls[i] & (ULTRA_HE_R_NOT_RISING << k)) {
                        cf(c, nth(Z(he_why, ":rise\0:high\0:low"), k));
                    }
                }
                cf(c, i < 4u ? Z(slash, "/") : Z(he_change, ", change "));
            }
            cf(c, Z(he_dev, "%e %%\r\n"), h->test_dev);
        }
        cf(c, Z(he_model, "health_model R/G/B/W1/W2=%e %%, dark %V A%z%z\r\n"), h->model_dev, (int32_t)h->dark_ma,
           (h->flags & ULTRA_HE_F_DARK) ? Z(he_dark, " (!)") : empty,
           (h->flags & HEALTH_F_CAL) ? Z(he_cal, ", calibration suggested") : empty);
    }
    if (c->scene && c->scene->mask) {
        cf(c, Z(sc_head, "power-on scene R/G/B/W1/W2="));
        for (i = 0; i < 5u; i++) {
            if (c->scene->mask & (1u << i)) {
                cf(c, Z(sc_level, "%p"), (uint32_t)c->scene->v[i]);
            } else {
                cf(c, Z(dash, "-"));
            }
            cf(c, i < 4u ? Z(slash, "/") : Z(sc_fade, " %%, fade "));
        }
        cf(c, Z(sc_s, "%k s\r\n"), (uint32_t)c->scene->fade);
    }
        break;
    case 10:
    events(c);                             /* (0.6.5, D2) */
        break;
    }
    return part < 10u;                     /* (the last: 10) */
}

#if CONSOLE_FULL
/* `help` (0.6.7: one part, the commands and the keys' names; their values: README / wiki: Console). Returns 0: no part
 * remains (cli_poll) */
static uint32_t help_part(cli_t *c, uint32_t part)
{
    int k;
    (void)part;
#if MON_EXTRA
    cf(c, ZCU(help, "status | get | set <key> <value> | period <steps> | save | defaults | identify | version | "
                    "watch [ms] | backup | restore <hex>\r\nkeys:"));
#else
    cf(c, ZCL(help, "status | get | set <key> <value> | period <steps> | save | defaults | identify | version | "
                    "watch [ms]\r\nkeys:"));
#endif
    for (k = KEYS; key_cmd(k); k = key_next(k)) {
        cf(c, ZC(help_key, " %z"), key_name(k));
    }
    cf(c, Z(crlf, "\r\n"));
    return 0;
}
#endif

uint32_t cli_poll(cli_t *c)
{
    uint32_t p = c->st_part;
    if (p-- == 0u) {
        return 0;
    }
#if CONSOLE_FULL
    c->st_part = (p < STATUS_PARTS ? status_part(c, p) : help_part(c, p - STATUS_PARTS)) ? (uint8_t)(c->st_part + 1u)
                                                                                         : 0u;
#else
    c->st_part = status_part(c, p) ? (uint8_t)(c->st_part + 1u) : 0u;
#endif
    return c->st_part != 0u;
}

void cli_watch(cli_t *c)
{
    const cli_status_t *s = c->st;
    cf(c, s->fps == 0u ? Z(w_nosig, "%t no signal  ") : Z(w_link, "%t %z %ufps  "), s->uptime_s,   /* (0.6.9) the link */
       nth(Z(w_proto, "unlocked\0WS2915\0WS2915\0WS2805\0Ultra"), s->proto_locked < 5u ? s->proto_locked : 0u), s->fps);
    cf(c, Z(w_line, "%v V %a A %h W  ch %A A  out %P %%  cpu %k %%  "), s->vs_supply_mv, s->i_ma,
       s->vs_supply_mv / 10u * clamp0(s->i_ma) / 100u, s->i_now_ch_ma, s->out,   /* the power as `input` in status */
       (uint32_t)s->cpu_pm);
    if (s->mcu_temp_c10 != -32768) {
        cf(c, Z(w_temp, "%K C  "), (int32_t)s->mcu_temp_c10);
    }
    ovr_out(c, Z(w_ovr, "ovr %u/%u/%u  "));                  /* the overruns, as in status (when there are any) */
    cf(c, s->dip ? Z(w_k_dips, "k %k %%  dips %u\r\n") : Z(w_k, "k %k %%\r\n"), s->i_k_permille,
       s->dip ? (uint32_t)s->dip->count : 0u);
}

#if PERF_BUILD
/* `perf` (core/perf.h): the paths' counts since the last `perf` (then from 0); the clock's cycles per poll tick meanwhile
 * (POLL_CYC = it counts all the time; less: mcycle stops in WFI; the share only right within 29.8 s, a 32-bit clock) */
static void perf_print(cli_t *c)
{
    perf_t p[PERF_N];
    uint32_t i, clock, ticks;
    perf_take(p, &clock, &ticks);
    cf(c, ZP(perf_head, "perf (cycles, %s) over %h s: the clock %u per poll tick\r\n"),
       PERF_BUILD == 1 ? "mcycle" : "SysTick", ticks / 20u, ticks ? clock / ticks : 0u);
    for (i = 0; i < PERF_N; i++) {
        cf(c, ZP(perf_line, "  %z: %u, mean %u, max %u\r\n"),     /* (the names: perf.h PERF_NAMES) */
           nth(ZP(perf_names, "poll ISR\0slew ISR\0gate ISR\0render\0slot decode\0header decode\0tables\0isense_ms\0"
                              "stagger"), i), p[i].n, p[i].n ? (uint32_t)(p[i].sum / p[i].n) : 0u,
           p[i].max);
    }
}
#endif
#if MON_EXTRA && CONSOLE_FULL
/* (0.6.7, D9) `backup`: the settings, their extension and the calibration as one `restore <hex>` line to paste back:
 * BK_VERSION | BOARD | settings_t | store_ext_t | isense_cal_t (the structs as stored; levels 0: none) |
 * CRC-16/CCITT-FALSE over the bytes before it, big-endian. `restore` checks the length, the CRC, the version and the
 * board, main.c (cli_t.restore) the values. BK_VERSION changes with any of the three structs' layout. */
#define BK_VERSION 1u
#define BK_BYTES   (2u + 16u + 7u + 48u)                 /* without the CRC */
typedef char bk_layout[(sizeof(settings_t) == 16u && sizeof(store_ext_t) == 7u && sizeof(isense_cal_t) == 48u) ? 1
                                                                                                             : -1];

static int32_t hexv(char ch)                             /* a hex digit's value, -1: none */
{
    uint32_t l = (uint32_t)(ch | 0x20) - 'a';
    return (uint32_t)(ch - '0') < 10u ? ch - '0' : l < 6u ? (int32_t)l + 10 : -1;
}
#endif

#if CONSOLE_FULL
void cli_exec(cli_t *c, const char *line)
{
    const char *p = line;
    int k;
    char word[12];
    uint32_t n = 0;

    while (*p == ' ') {
        p++;
    }
    while (*p && *p != ' ' && n + 1u < sizeof word) {
        word[n++] = *p++;
    }
    word[n] = 0;
    if (n == 0u) {
        return;
    }
    if (cz_eq(word, ZC(w_help, "help")) || cz_eq(word, ZC(qmark, "?"))) {
        c->st_part = (uint8_t)(STATUS_PARTS + 1u);   /* printed by cli_poll, as status (0.6.5 D8) */
    } else if (cz_eq(word, ZC(w_status, "status"))) {
        if (c->status_begin) c->status_begin();
        c->st_part = 1;                    /* printed by cli_poll, part by part (0.6.5) */
    } else if (cz_eq(word, ZC(w_get, "get"))) {
        for (k = KEYS; key_cmd(k); k = key_next(k)) {
            cf(c, ZC(get_key, "%z=%u\r\n"), key_name(k), (uint32_t)get_key(c, key_cmd(k)));
        }
        cf(c, ZC(get_period, "period=%u\r\n"), settings_steps(c->set));
    } else if (cz_eq(word, ZC(w_set, "set"))) {
        char key[12];
        int32_t v;
        n = 0;
        while (*p == ' ') {
            p++;
        }
        while (*p && *p != ' ' && n + 1u < sizeof key) {
            key[n++] = *p++;
        }
        key[n] = 0;
        v = parse_u(&p);
        for (k = KEYS; key_cmd(k) && !cz_eq(key, key_name(k)); k = key_next(k)) {
        }
        if (key_cmd(k) == 0 || v < 0 || v > 255) {
            cf(c, ZC(err_set, "error: set <key> <value>, see help\r\n"));
            return;
        }
        result(c, c->apply(key_cmd(k), (uint8_t)v));
    } else if (cz_eq(word, ZC(w_period, "period"))) {
        int32_t v = parse_u(&p);
        if (v < (int32_t)PWM_STEPS_MIN || v > (int32_t)PWM_STEPS_MAX) {
            cf(c, ZC(err_period, "error: period 1440..65535\r\n"));
            return;
        }
        c->apply(CMD_PERIOD_HI, (uint8_t)(v >> 8));
        result(c, c->apply(CMD_PERIOD_LO, (uint8_t)(v & 0xFF)));
    } else if (cz_eq(word, ZC(w_save, "save"))) {
        c->save_now();                     /* (it reports: ok / pending / failed; 0.6.4) */
    } else if (cz_eq(word, ZC(w_defaults, "defaults"))) {
        result(c, c->apply(CMD_FACTORY_RESET, 0xA5));
    } else if (cz_eq(word, ZC(w_identify, "identify"))) {
        c->apply(CMD_IDENTIFY, 0);
        cf(c, ZC(ok_crlf, "ok\r\n"));
    } else if (cz_eq(word, ZC(w_version, "version"))) {
        cli_banner(c);
        c->apply(CMD_IDENTIFY, CFG_IDENTIFY_VERSION);   /* and blink it */
#if PERF_BUILD
    } else if (cz_eq(word, ZP(w_perf, "perf"))) {
        perf_print(c);                     /* (a bench build: core/perf.h) */
#endif
#if MON_EXTRA && CONSOLE_FULL
    } else if (cz_eq(word, ZCU(w_backup, "backup"))) {   /* (0.6.7, D9: the upper layout's console, not the receiver) */
        static const char hx[] = "0123456789abcdef";
        uint8_t r[BK_BYTES + 2u];
        char t[3];
        r[0] = BK_VERSION;
        r[1] = BOARD;
        memcpy(r + 2, c->set, 16u);
        memcpy(r + 18, c->ext, 7u);
        memcpy(r + 25, &c->st->cal, 48u);
        n = crc16_ccitt(r, BK_BYTES);
        r[BK_BYTES] = (uint8_t)(n >> 8);
        r[BK_BYTES + 1u] = (uint8_t)n;
        cf(c, ZCU(bk_head, "restore "));
        t[2] = 0;
        for (n = 0; n < sizeof r; n++) {
            t[0] = hx[r[n] >> 4];
            t[1] = hx[r[n] & 15u];
            c->write(t);
        }
        cf(c, Z(crlf, "\r\n"));
    } else if (cz_eq(word, ZCU(w_restore, "restore"))) {   /* (0.6.7, D9) */
        uint8_t r[BK_BYTES + 2u];
        int32_t h;
        while (*p == ' ') {
            p++;
        }
        for (n = 0; n < 2u * sizeof r && (h = hexv(*p)) >= 0; n++, p++) {   /* (n: nibbles) */
            r[n >> 1] = (uint8_t)(n & 1u ? r[n >> 1] | h : h << 4);
        }
        n = *p || n != 2u * sizeof r ? 0u
          : crc16_ccitt(r, BK_BYTES) != (uint32_t)(r[BK_BYTES] << 8 | r[BK_BYTES + 1u]) ? 1u
          : r[0] != BK_VERSION ? 2u : r[1] != BOARD ? 3u : !c->restore || !c->restore(r + 2) ? 4u : 5u;
        cf(c, n == 5u ? ZCU(rs_ok, "ok%z\r\n") : ZCU(rs_err, "error: restore: %z\r\n"),
           nth(ZCU(rs_why, "length\0CRC\0version\0board\0values\0"), n));
#endif
    } else if (cz_eq(word, ZC(w_watch, "watch"))) {
        int32_t v = parse_u(&p);
        c->watch_ms = (uint16_t)(v < 0 ? 1000 : v < 100 ? 100 : v > 60000 ? 60000 : v);
    } else {
        cf(c, ZC(err_unknown, "error: unknown command\r\n"));
    }
}

void cli_feed(cli_t *c, char ch)
{
    if (c->watch_ms) {                     /* any key stops `watch` (and is used up) */
        c->watch_ms = 0;
        c->len = 0;
        return;
    }
    if (ch == '\r' || ch == '\n') {
        c->line[c->len] = 0;
        if (c->len) {
            cli_exec(c, c->line);
        }
        c->len = 0;
    } else if (ch == 0x08 || ch == 0x7F) {
        c->len -= c->len != 0u;            /* backspace / delete: the last character goes (0.6.5) */
    } else if (ch != 0 && c->len + 1u < sizeof c->line) {   /* (0.6.7: a UART break's 0x00, the jig's TP6: none) */
        c->line[c->len++] = ch;
    }
}
#endif
