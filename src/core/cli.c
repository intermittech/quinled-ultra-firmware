/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include <stdarg.h>
#include "cli.h"
#include "climit.h"
#include "ultra_protocol.h"
#include "presets.h"

typedef struct {
    const char *key;
    uint8_t cmd;
    const char *help;
} cli_key_t;

static const cli_key_t keys[] = {        /* short help: the values (README / wiki: Console have the details) */
    { "protocol", CMD_PROTOCOL,      "0 auto 1 WS2915 2 WS2805 3 Ultra" },
    { "preset",   CMD_PWM_PRESET,    "0..14 = 2.2..100 kHz, 8 = 25 kHz" },
    { "dither",   CMD_DITHER_MODE,   "0 on 1 off-low 2 off" },
    { "thresh",   CMD_DITHER_THRESH, "1..255 x 0.1 %" },
    { "feedback", CMD_FEEDBACK_CH,   "0 red 1 all 2 off" },
    { "stagger",  CMD_STAGGER,       "0 off 1 fixed 2 +spread 3 adaptive 4 +spread" },
    { "floor",    CMD_DIM_FLOOR,     "0 skip 1 clamp 2 off" },
    { "minpulse", CMD_MIN_PULSE,     "5..100 x 10 ns" },
    { "loss",     CMD_SIGNAL_LOSS,   "0 hold 1 off 2 fade off" },
    { "smooth",   CMD_SMOOTHING,     "0 off 1 WS2805 2 all" },
    { "gamma",    CMD_GAMMA,         "WS2805: 0 linear 1 2.2 2 2.8" },
    { "scene",    CMD_SCENE,         "power-on: 0 none 1 store now" },
    { "scenefade", CMD_SCENE_FADE,   "0..255 x 100 ms" },
    { "ilimit",   CMD_ISENSE_LIMIT,  "x 50 mA, 0 = 8 A" },
    { "ical",     CMD_ISENSE_CAL,    "0 = 25/50/75/100 %, 5..100 = one level" },
    { "softstart", CMD_SOFT_START,   "0 off 1 on" },
};
#define NKEYS (sizeof(keys) / sizeof(keys[0]))

static int streq(const char *a, const char *b)
{
    while (*a && *a == *b) {
        a++;
        b++;
    }
    return *a == *b;
}

/* ---------------------------------------------------------------------------------------------------- output */
/* The console's one formatter: fmt with one argument per %-code. Readable units on the console (V, A, W, %, s);
 * telemetry and the data structures stay raw (mV, mA, 16-bit outputs).
 *   %s string            %u decimal (uint32)       %t s as [Nd ]h:mm:ss           %% a '%'
 *   n / d rounded with 1 or 2 decimals:
 *   %v uint32 / 10, 2 decimals (mV in V, 10 mWh in Wh)    %a int32 mA in A, 2 decimals (negative: 0)
 *   %V int32 / 10, 2 decimals, signed                      %p 16-bit level in %, 1 decimal
 *   %k uint32 / 10, 1 decimal (permille in %, 100 ms in s)  %K int32 / 10, 1 decimal, signed (0.1 C)
 *   %h uint32 / 100, 1 decimal (ms in s, 10 mW in W)       %H uint32 / 100, 2 decimals
 *   %q Q16 of the period in %, 1 decimal
 *   five values, "/" between: %P uint16[5] levels (%p), %A int32[5] mA (%a), %e int8[5] signed % (ULTRA_HE_NA: -) */
#define CF_BUF 48u

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

static void cf(cli_t *c, const char *f, ...)
{
    cf_out_t o;
    va_list ap;
    uint32_t i;
    o.c = c;
    o.n = 0;
    va_start(ap, f);
    for (; *f; f++) {
        if (*f != '%') {
            cf_ch(&o, *f);
            continue;
        }
        switch (*++f) {
        case 's':
            cf_flush(&o);
            c->write(va_arg(ap, const char *));
            break;
        case 'u': cf_dec(&o, va_arg(ap, uint32_t), 1u); break;
        case 'v': cf_fix(&o, va_arg(ap, uint32_t), 10u, 2u); break;
        case 'a': cf_fix(&o, clamp0(va_arg(ap, int32_t)), 10u, 2u); break;
        case 'V': cf_fix(&o, cf_neg(&o, va_arg(ap, int32_t)), 10u, 2u); break;
        case 'p': cf_fix(&o, va_arg(ap, uint32_t) * 1000u, 65535u, 1u); break;
        case 'k': cf_fix(&o, va_arg(ap, uint32_t), 1u, 1u); break;
        case 'K': cf_fix(&o, cf_neg(&o, va_arg(ap, int32_t)), 1u, 1u); break;
        case 'h': cf_fix(&o, va_arg(ap, uint32_t), 100u, 1u); break;
        case 'H': cf_fix(&o, va_arg(ap, uint32_t), 100u, 2u); break;
        case 'q': cf_fix(&o, va_arg(ap, uint32_t) * 1000u, 65536u, 1u); break;
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
                if (*f == 'P') {
                    cf_fix(&o, ((const uint16_t *)p)[i] * 1000u, 65535u, 1u);
                } else if (*f == 'A') {
                    cf_fix(&o, clamp0(((const int32_t *)p)[i]), 10u, 2u);
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

/* ---------------------------------------------------------------------------------------------------- reports */
void cli_test_report(cli_t *c, const int32_t ma[5], uint32_t pct, uint32_t cycle, uint32_t supply_mv)
{
    static const char *const name[5] = { "R ", "G ", "B ", "W1 ", "W2 " };
    uint32_t i, k;
    cf(c, "test @%u%%: ", pct);
    for (k = 0; k < 2u; k++) {             /* as measured at pct %, then scaled to 100 % */
        for (i = 0; i < 5u; i++) {
            uint32_t v = clamp0(ma[i]);
            cf(c, "%s%H%s", name[i], k == 0u ? v * 10u : (v * 1000u + pct / 2u) / (pct ? pct : 1u),
               i < 4u ? (k == 0u ? " A, " : ", ") : (k == 0u ? " A; est. @100%: " : " A; supply "));
        }
    }
    cf(c, supply_mv != 0u ? "%v V (cycle %u)\r\n" : "n/a (cycle %u)\r\n", supply_mv != 0u ? supply_mv : cycle, cycle);
}

void cli_banner(cli_t *c)
{
    static const char *const app[] = { "standalone build (no bootloader)",
                                       "trial (not confirmed yet)", "confirmed" };
    const cli_status_t *s = c->st;
    cf(c, "QuinLED dig2analog-ultra firmware v%s, Copyright (C) 2026 QuinLED, GPL-3.0, source: "
          "github.com/intermittech/quinled-ultra-firmware\r\n", s->version);
    if (s->bl_version != 0u) {
        cf(c, "bootloader rev %u (build %s), hardware %u revision %u, ", s->bl_version, s->bl_build, s->hw_id,
           s->hw_rev);
    }
    cf(c, "app %s\r\n", app[s->app_state < 3u ? s->app_state : 0]);
}

static uint8_t get_key(const cli_t *c, uint8_t cmd)
{
    const settings_t *s = c->set;
    switch (cmd) {
    case CMD_SCENE:         return (uint8_t)(c->scene && c->scene->mask != 0u);
    case CMD_SCENE_FADE:    return c->scene ? c->scene->fade : 0u;
    case CMD_ISENSE_LIMIT:  return c->st ? c->st->ilimit50 : 0u;
    case CMD_ISENSE_CAL:    return 0u;
    case CMD_SOFT_START:    return c->st ? c->st->softstart : 1u;
    case CMD_PROTOCOL:      return s->protocol;
    case CMD_PWM_PRESET:    return s->pwm_preset;
    case CMD_DITHER_MODE:   return s->dither_mode;
    case CMD_DITHER_THRESH: return s->dither_thresh;
    case CMD_FEEDBACK_CH:   return s->feedback_ch;
    case CMD_STAGGER:       return s->stagger;
    case CMD_DIM_FLOOR:     return s->dim_floor;
    case CMD_MIN_PULSE:     return s->min_pulse;
    case CMD_SIGNAL_LOSS:   return s->signal_loss;
    case CMD_SMOOTHING:     return s->smoothing;
    default:                return s->gamma;
    }
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
    c->write(r.event == CFG_ACK ? "ok\r\n" : r.event == CFG_SAME ? "ok (unchanged)\r\n" : "error: invalid value\r\n");
}

static void status(cli_t *c)
{
    const cli_status_t *s = c->st;
    static const char *const proto[] = { "none", "WS2915 32-bit header", "WS2915 26-bit header", "WS2805", "Ultra" };
    static const char *const mode[] = { "auto", "WS2915", "WS2805", "Ultra" };
    static const char *const err[] = { "", "no edge", "no gap", "short", "truncated", "pulse" };
    static const char *const cause[] = { "-", "short", "overload" };
    uint32_t i;
    cli_banner(c);
    cf(c, "uptime=%t\r\nreset=%s%s%s%s\r\nstack=%u B used\r\nprotocol=%s, active %s%s, stored %s\r\nframes=%u\r\n"
          "fps=%u\r\ndecoded=%u\r\n",
       s->uptime_s, (s->reset_flags & 0x20000000u) ? "watchdog " : "", (s->reset_flags & 0x08000000u) ? "power-on " : "",
       (s->reset_flags & 0x04000000u) ? "pin " : "", (s->reset_flags & 0x10000000u) ? "software " : "", s->stack_max,
       mode[s->proto_mode < 4u ? s->proto_mode : 0], proto[s->proto_locked < 5u ? s->proto_locked : 0],
       s->proto_provisional ? " (stored, not confirmed yet)" : "", proto[c->set->last_proto < 5u ? c->set->last_proto : 0],
       s->frames, s->fps, s->decoded);
    for (i = 1; i < 6u; i++) {
        if (s->dec_err[i]) {
            cf(c, "decode_error %s=%u\r\n", err[i], s->dec_err[i]);
        }
    }
    if (s->ultra_headers || s->ultra_tail_err) {
        cf(c, "ultra_headers=%u\r\nultra_tail_errors=%u\r\n", s->ultra_headers, s->ultra_tail_err);
    }
    cf(c, "config_accepted=%u\r\npwm_hz=%u\r\npwm_steps=%u\r\nflash_saves=%u\r\n", s->config_frames,
       (uint32_t)(F_TIM_HZ / s->steps), s->steps, s->saves);
    if (s->stagger_moves || s->ripple_ma) {    /* the offsets in % of the period */
        cf(c, "stagger_tim2=%q %%\r\nstagger_tim3=%q %%\r\nstagger_moves=%u\r\nripple_rms=%a A\r\n",
           (uint32_t)s->stagger_o2, (uint32_t)s->stagger_o3, s->stagger_moves, (int32_t)s->ripple_ma);
    }
    {
        static const char *const model[] = { "none", "learned", "calibrated" };
        cf(c, "out R/G/B/W1/W2=%P %%\r\ncurrent=%a A\r\ncurrent_peak=%a A\r\ncurrent_zero=%a A\r\ncurrent_limit=%a A\r\n"
              "current_scale=%k %%\r\ncurrent_trips=%u\r\ncurrent_state=%s, last trip %s\r\n"
              "channel R/G/B/W1/W2=%A A (%s)\r\n",
           s->out, s->i_ma, s->i_peak_ma, s->i_zero_ma, (int32_t)s->i_limit_ma, s->i_k_permille, s->i_trips,
           s->i_cal ? "calibrating" : (s->i_tripped ? "tripped, outputs off" : "ok"),
           cause[s->i_trip_cause < 3u ? s->i_trip_cause : 0], s->i_ch_ma, model[s->i_model < 3u ? s->i_model : 0]);
        if (s->cal.levels) {
            static const char ch[5][3] = { "R", "G", "B", "W1", "W2" };
            uint32_t l;
            c->write("calibration=");
            for (l = 0; l < s->cal.levels && l < 4u; l++) {
                cf(c, "%u%s", s->cal.pct[l], l + 1u < s->cal.levels ? "/" : " %, ");
            }
            cf(c, "%u tests stored\r\n", s->cal_count);
            for (i = 0; i < 5u; i++) {
                cf(c, "calibration %s=", ch[i]);
                for (l = 0; l < s->cal.levels && l < 4u; l++) {
                    cf(c, "%a%s", (int32_t)s->cal.ma[i][l], l + 1u < s->cal.levels ? "/" : " A\r\n");
                }
            }
        }
    }
    if (s->mcu_temp_c10 == -32768) {
        c->write("mcu_temp=n/a\r\n");
    } else {
        cf(c, "mcu_temp=%K C (die), vdda=%v V, peak %K C %t ago\r\n", (int32_t)s->mcu_temp_c10, (uint32_t)s->vdda_mv,
           (int32_t)s->mcu_peak_c10, s->uptime_s - s->mcu_peak_s);
    }
    cf(c, "supply=%v V (min %v V, max %v V)\r\n", s->vs_supply_mv, s->vs_supply_min_mv, s->vs_supply_max_mv);
    if (s->dip) {                          /* supply dips (core/vdip.h): the deepest, then the log, newest first */
        const vdip_t *d = s->dip;
        const vdip_ev_t *e = d->deep.ref_x ? &d->deep : 0;
        uint32_t k;
        cf(c, "supply_dips=%u%s%v V", (uint32_t)d->count, d->in_dip ? " (one now), reference " : ", reference ",
           s->dip_ref_mv);
        for (k = 0; e != 0; e = vdip_event(d, k++)) {
            cf(c, k ? "\r\nsupply_dip %u=" : "\r\nsupply_dip_deepest=", k);
            cf(c, "%t ago, %v V (%u %%), %u ms, %a A", s->uptime_s - e->t_s, vsense_mv(s->dip_vs, e->min_x, 1u),
               vdip_pct(e), (uint32_t)e->dur_ms, (int32_t)e->i_ma);
        }
        c->write("\r\n");
    }
    /* V x I: 10 mV x mA / 100 keeps it in 32 bits (75 V x 65 A): mW, shown in W */
    cf(c, "input=%h W\r\nenergy=%v Wh since boot, %u Wh lifetime\r\ntp8=%v V\r\ntest_mode=%s\r\nboard_limit=",
       s->vs_supply_mv / 10u * clamp0(s->i_ma) / 100u, s->en_boot_mwh, s->en_total_wh, s->vs_aux_mv,
       s->test_mode ? "on" : "off");
    if (s->lim_ma == 0xFFFFu) {
        c->write("none");
    } else {
        cf(c, "%a A (%s)", (int32_t)s->lim_ma, climit_src_name(s->lim_src));
    }
    if (s->lim_conn_ma) {
        cf(c, ", connected %a A: %s", (int32_t)s->lim_conn_ma,
           s->lim_need == CLIMIT_NEED_NO ? "no limit needed\r\n" : "the limit may engage\r\n");
    } else {
        c->write(", connected not known\r\n");
    }
    /* the overcurrent signal on D9 (core/ocled.h) */
    if (s->oc_show == 0u) {
        c->write("overcurrent_led=off");
    } else {
        cf(c, s->oc_show == 2u ? "overcurrent_led=trip (%s), since %t (%u s ago), " : "overcurrent_led=limit%s, since %t (%u s ago), ",
           s->oc_show == 2u ? cause[s->oc_cause < 3u ? s->oc_cause : 0] : "", s->oc_since_s, s->uptime_s - s->oc_since_s);
        if (s->oc_show == 2u ? s->oc_tripped : s->oc_limiting) {
            c->write(s->oc_show == 2u ? "outputs off" : "board limit active");
        } else {
            cf(c, "hold %h s", s->oc_hold_ms);
        }
    }
    c->write("\r\n");
    /* the power events (core/pwrev.h): what a Diff-Power ultra receiver reports on its STATUS / POWER pages */
    if (s->pw) {
        const pwrev_t *g = s->pw;
        uint8_t st = pwrev_state(g), lat = pwrev_latched(g);
        cf(c, "power_state=%s%s%s%s",
           (st & (ULTRA_ST_LIMITING | ULTRA_ST_TRIPPED | ULTRA_ST_CH_LIMITING)) ? "" : "ok",
           (st & ULTRA_ST_LIMITING) ? "limiting" : "",
           (st & ULTRA_ST_TRIPPED) ? ((st & ULTRA_ST_LIMITING) ? ", tripped" : "tripped") : "",
           (st & ULTRA_ST_CH_LIMITING) ? ((st & (ULTRA_ST_LIMITING | ULTRA_ST_TRIPPED)) ? ", channel-limiting"
                                                                                     : "channel-limiting") : "");
        if (st & ULTRA_ST_TRIPPED) {
            cf(c, " (retry in %u s)", (g->in.retry_ms + 999u) / 1000u);   /* as the POWER page: whole seconds, up */
        } else if (st & ULTRA_ST_LIMITING) {
            cf(c, " (factor %k %%)", (uint32_t)g->in.factor * 1000u / 65535u);
        }
        cf(c, "\r\npower_events=limit %u, overload trips %u, short trips %u%s%s%s%s%s\r\npower_last_limit=",
           (uint32_t)g->limit_events, (uint32_t)g->ovl_trips, (uint32_t)g->short_trips,
           lat ? ", latched:" : ", nothing latched", (lat & ULTRA_PW_EV_LIMITED) ? " limit" : "",
           (lat & ULTRA_PW_EV_OVERLOAD) ? " overload" : "", (lat & ULTRA_PW_EV_SHORT) ? " short" : "",
           (lat & ULTRA_PW_EV_SUPPLY_DIP) ? " dip" : "");
        if (g->limit_age_s == PWREV_AGE_NONE) {
            c->write("none");
        } else {
            cf(c, "%u s ago, %a A", g->limit_age_s, (int32_t)g->event_ma);
        }
        if (g->trip_age_s == PWREV_AGE_NONE) {
            c->write("\r\npower_last_trip=none\r\n");
        } else {
            cf(c, "\r\npower_last_trip=%u s ago, %s, %a A\r\n", g->trip_age_s,
               cause[g->last_cause < 3u ? g->last_cause : 0], (int32_t)g->trip_ma);
        }
    }
    if (s->hl) {                           /* channel health (core/health.h) */
        static const char *const cls[4] = { "-", "none", "ok", "FAULT" };
        static const char *const why[3] = { ":rise", ":high", ":low" };
        const health_t *h = s->hl;
        uint32_t k;
        if (h->flags & ULTRA_HE_F_TESTED) {
            c->write("health_test R/G/B/W1/W2=");
            for (i = 0; i < 5u; i++) {
                c->write(cls[h->cls[i] & 3u]);
                for (k = 0; k < 3u; k++) {
                    c->write((h->cls[i] & (ULTRA_HE_R_NOT_RISING << k)) ? why[k] : "");
                }
                c->write(i < 4u ? "/" : ", change ");
            }
            cf(c, "%e %%\r\n", h->test_dev);
        }
        cf(c, "health_model R/G/B/W1/W2=%e %%, dark %V A%s\r\n", h->model_dev, (int32_t)h->dark_ma,
           (h->flags & ULTRA_HE_F_DARK) ? " (!)" : "");
    }
    if (c->scene && c->scene->mask) {
        c->write("power-on scene R/G/B/W1/W2=");
        for (i = 0; i < 5u; i++) {
            if (c->scene->mask & (1u << i)) {
                cf(c, "%p", (uint32_t)c->scene->v[i]);
            } else {
                c->write("-");
            }
            c->write(i < 4u ? "/" : " %, fade ");
        }
        cf(c, "%k s\r\n", (uint32_t)c->scene->fade);
    }
}

void cli_watch(cli_t *c)
{
    const cli_status_t *s = c->st;
    cf(c, "%t %v V %a A %h W  ch %A A  out %P %%  ", s->uptime_s, s->vs_supply_mv, s->i_ma,
       s->vs_supply_mv / 10u * clamp0(s->i_ma) / 100u, s->i_now_ch_ma, s->out);   /* the power as `input` in status */
    if (s->mcu_temp_c10 != -32768) {
        cf(c, "%K C  ", (int32_t)s->mcu_temp_c10);
    }
    cf(c, s->dip ? "k %k %%  dips %u\r\n" : "k %k %%\r\n", s->i_k_permille, s->dip ? (uint32_t)s->dip->count : 0u);
}

void cli_exec(cli_t *c, const char *line)
{
    const char *p = line;
    char word[12];
    uint32_t n = 0, i;

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
    if (streq(word, "help") || streq(word, "?")) {
        c->write("status | get | set <key> <value> | period <steps> | save | defaults | identify | version | "
                 "watch [ms]\r\n");
        for (i = 0; i < NKEYS; i++) {
            cf(c, "  %s: %s\r\n", keys[i].key, keys[i].help);
        }
    } else if (streq(word, "status")) {
        status(c);
    } else if (streq(word, "get")) {
        for (i = 0; i < NKEYS; i++) {
            cf(c, "%s=%u\r\n", keys[i].key, (uint32_t)get_key(c, keys[i].cmd));
        }
        cf(c, "period=%u\r\n", settings_steps(c->set));
    } else if (streq(word, "set")) {
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
        for (i = 0; i < NKEYS && !streq(key, keys[i].key); i++) {
        }
        if (i == NKEYS || v < 0 || v > 255) {
            c->write("error: set <key> <value>, see help\r\n");
            return;
        }
        result(c, c->apply(keys[i].cmd, (uint8_t)v));
    } else if (streq(word, "period")) {
        int32_t v = parse_u(&p);
        if (v < (int32_t)PWM_STEPS_MIN || v > (int32_t)PWM_STEPS_MAX) {
            c->write("error: period 1440..65535\r\n");
            return;
        }
        c->apply(CMD_PERIOD_HI, (uint8_t)(v >> 8));
        result(c, c->apply(CMD_PERIOD_LO, (uint8_t)(v & 0xFF)));
    } else if (streq(word, "save")) {
        c->save_now();
        c->write("ok\r\n");
    } else if (streq(word, "defaults")) {
        result(c, c->apply(CMD_FACTORY_RESET, 0xA5));
    } else if (streq(word, "identify")) {
        c->apply(CMD_IDENTIFY, 0);
        c->write("ok\r\n");
    } else if (streq(word, "version")) {
        cli_banner(c);
        c->apply(CMD_IDENTIFY, CFG_IDENTIFY_VERSION);   /* and blink it */
    } else if (streq(word, "watch")) {
        int32_t v = parse_u(&p);
        c->watch_ms = (uint16_t)(v < 0 ? 1000 : v < 100 ? 100 : v > 60000 ? 60000 : v);
    } else {
        c->write("error: unknown command\r\n");
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
    } else if (c->len + 1u < sizeof c->line) {
        c->line[c->len++] = ch;
    }
}
