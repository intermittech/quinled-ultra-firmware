/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
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
    { "ilimit",   CMD_ISENSE_LIMIT,  "x 50 mA, 0 = 8 A (isense)" },
    { "ical",     CMD_ISENSE_CAL,    "0 = 25/50/75/100 %, 5..100 = one level (isense)" },
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

static char *utoa10(uint32_t v, char *end)
{
    *--end = 0;
    do {
        *--end = (char)('0' + v % 10u);
        v /= 10u;
    } while (v);
    return end;
}

static void wnum(cli_t *c, uint32_t v)
{
    char b[12];
    c->write(utoa10(v, b + sizeof b));
}

static void kv(cli_t *c, const char *k, uint32_t v)
{
    c->write(k);
    c->write("=");
    wnum(c, v);
    c->write("\r\n");
}

void cli_test_report(cli_t *c, const int32_t ma[5], uint32_t pct, uint32_t cycle, uint32_t supply_mv)
{
    static const char *const name[5] = { "R ", "G ", "B ", "W1 ", "W2 " };
    uint8_t i, k;
    for (k = 0; k < 2u; k++) {             /* as measured at pct %, then scaled to 100 % */
        c->write(k == 0u ? "test @" : " mA; est. @100%: ");
        if (k == 0u) {
            wnum(c, pct);
            c->write("%: ");
        }
        for (i = 0; i < 5u; i++) {
            uint32_t v = ma[i] > 0 ? (uint32_t)ma[i] : 0u;
            if (i != 0u) {
                c->write(k == 0u ? " mA, " : ", ");
            }
            c->write(name[i]);
            wnum(c, k == 0u ? v : (v * 100u + pct / 2u) / (pct ? pct : 1u));
        }
    }
    c->write(" mA; supply ");
    if (supply_mv != 0u) {
        wnum(c, supply_mv);
        c->write(" mV");
    } else {
        c->write("n/a");
    }
    c->write(" (cycle ");
    wnum(c, cycle);
    c->write(")\r\n");
}

void cli_banner(cli_t *c)
{
    static const char *const app[] = { "standalone build (no bootloader, no updates over the data line)",
                                       "trial (not confirmed yet)", "confirmed" };
    const cli_status_t *s = c->st;
    c->write("QuinLED dig2analog-ultra firmware v");
    c->write(s->version);
    c->write(", Copyright (C) 2026 QuinLED, GPL-3.0, source: github.com/intermittech/quinled-ultra-firmware\r\n");
    if (s->bl_version != 0u) {
        c->write("bootloader rev ");
        wnum(c, s->bl_version);
        c->write(" (build ");
        c->write(s->bl_build);
        c->write("), hardware ");
        wnum(c, s->hw_id);
        c->write(" revision ");
        wnum(c, s->hw_rev);
        c->write(", app ");
    } else {
        c->write("app ");
    }
    c->write(app[s->app_state < 3u ? s->app_state : 0]);
    c->write("\r\n");
}

static uint8_t get_key(const cli_t *c, uint8_t cmd)
{
    const settings_t *s = c->set;
    switch (cmd) {
    case CMD_SCENE:         return (uint8_t)(c->scene && c->scene->mask != 0u);
    case CMD_SCENE_FADE:    return c->scene ? c->scene->fade : 0u;
    case CMD_ISENSE_LIMIT:  return c->st ? c->st->ilimit50 : 0u;
    case CMD_ISENSE_CAL:    return 0u;
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
    uint32_t i;
    cli_banner(c);
    kv(c, "uptime_ms", s->uptime_ms);
    c->write("reset=");
    c->write((s->reset_flags & 0x20000000u) ? "watchdog " : "");
    c->write((s->reset_flags & 0x08000000u) ? "power-on " : "");
    c->write((s->reset_flags & 0x04000000u) ? "pin " : "");
    c->write((s->reset_flags & 0x10000000u) ? "software " : "");
    c->write("\r\nprotocol=");
    c->write(mode[s->proto_mode < 4u ? s->proto_mode : 0]);
    c->write(", active ");
    c->write(proto[s->proto_locked < 5u ? s->proto_locked : 0]);
    c->write(s->proto_provisional ? " (stored, not confirmed yet)" : "");
    c->write(", stored ");
    c->write(proto[c->set->last_proto < 5u ? c->set->last_proto : 0]);
    c->write("\r\n");
    kv(c, "frames", s->frames);
    kv(c, "fps", s->fps);
    kv(c, "decoded", s->decoded);
    for (i = 1; i < 6u; i++) {
        if (s->dec_err[i]) {
            c->write("decode_error ");
            c->write(err[i]);
            c->write("=");
            wnum(c, s->dec_err[i]);
            c->write("\r\n");
        }
    }
    if (s->ultra_headers || s->ultra_tail_err) {
        kv(c, "ultra_headers", s->ultra_headers);
        kv(c, "ultra_tail_errors", s->ultra_tail_err);
    }
    kv(c, "config_accepted", s->config_frames);
    kv(c, "pwm_hz", F_TIM_HZ / s->steps);
    kv(c, "pwm_steps", s->steps);
    kv(c, "flash_saves", s->saves);
    if (s->stagger_moves || s->ripple_ma) {
        kv(c, "stagger_tim2_permille", (uint32_t)s->stagger_o2 * 1000u >> 16);
        kv(c, "stagger_tim3_permille", (uint32_t)s->stagger_o3 * 1000u >> 16);
        kv(c, "stagger_moves", s->stagger_moves);
        kv(c, "ripple_ma_rms", s->ripple_ma);
    }
    c->write("out R/G/B/W1/W2=");
    for (i = 0; i < 5u; i++) {
        wnum(c, s->out[i]);
        c->write(i < 4u ? "/" : "\r\n");
    }
    if (s->isense) {
        static const char *const model[] = { "none", "learned", "calibrated" };
        static const char *const cause[] = { "-", "short", "overload" };
        kv(c, "current_ma", (uint32_t)(s->i_ma > 0 ? s->i_ma : 0));
        kv(c, "current_peak_ma", (uint32_t)(s->i_peak_ma > 0 ? s->i_peak_ma : 0));
        kv(c, "current_zero_ma", (uint32_t)(s->i_zero_ma > 0 ? s->i_zero_ma : 0));
        kv(c, "current_limit_ma", s->i_limit_ma);
        kv(c, "current_scale_permille", s->i_k_permille);
        kv(c, "current_trips", s->i_trips);
        c->write("current_state=");
        c->write(s->i_cal ? "calibrating" : (s->i_tripped ? "tripped, outputs off" : "ok"));
        c->write(", last trip ");
        c->write(cause[s->i_trip_cause < 3u ? s->i_trip_cause : 0]);
        c->write("\r\nchannel_ma R/G/B/W1/W2=");
        for (i = 0; i < 5u; i++) {
            wnum(c, (uint32_t)(s->i_ch_ma[i] > 0 ? s->i_ch_ma[i] : 0));
            c->write(i < 4u ? "/" : "");
        }
        c->write(" (");
        c->write(model[s->i_model < 3u ? s->i_model : 0]);
        c->write(")\r\n");
        if (s->cal_levels) {
            static const char ch[5][3] = { "R", "G", "B", "W1", "W2" };
            uint32_t l;
            c->write("calibration=");
            for (l = 0; l < s->cal_levels && l < 4u; l++) {
                wnum(c, s->cal_pct[l]);
                c->write(l + 1u < s->cal_levels ? "/" : " %, ");
            }
            wnum(c, s->cal_count);
            c->write(" tests stored\r\n");
            for (i = 0; i < 5u; i++) {
                c->write("calibration_ma ");
                c->write(ch[i]);
                c->write("=");
                for (l = 0; l < s->cal_levels && l < 4u; l++) {
                    wnum(c, s->cal_ma[i][l]);
                    c->write(l + 1u < s->cal_levels ? "/" : "\r\n");
                }
            }
        }
    }
    c->write("mcu_temp=");
    if (s->mcu_temp_c10 == -32768) {
        c->write("n/a\r\n");
    } else {
        int32_t t = s->mcu_temp_c10;
        if (t < 0) {
            c->write("-");
            t = -t;
        }
        wnum(c, (uint32_t)t / 10u);
        c->write(".");
        wnum(c, (uint32_t)t % 10u);
        c->write(" C (die), vdda_mv=");
        wnum(c, s->vdda_mv);
        c->write("\r\n");
    }
    c->write("supply_mv=");
    if (!s->vs_supply_fitted) {
        c->write("n/a (PA5 not read: BOARD_VSENSE 0)");
    } else {
        wnum(c, s->vs_supply_mv);
        c->write(" (min ");
        wnum(c, s->vs_supply_min_mv);
        c->write(", max ");
        wnum(c, s->vs_supply_max_mv);
        c->write(" since the last status)");
        if (s->isense) {
            /* V x I: 10 mV x mA / 100 keeps it in 32 bits (75 V x 65 A) */
            uint32_t ma = s->i_ma > 0 ? (uint32_t)s->i_ma : 0u;
            c->write("\r\ninput_mw=");
            wnum(c, s->vs_supply_mv / 10u * ma / 100u);
            c->write(" (supply_mv x current_ma)");
        }
    }
    c->write("\r\ntp8_mv=");
    wnum(c, s->vs_aux_mv);
    c->write(" (PA0, spare input)\r\n");
    c->write(s->test_mode ? "test_mode=on (test pad grounded)\r\n" : "test_mode=off\r\n");
    c->write("board_limit=");
    if (s->lim_ma == 0xFFFFu) {
        c->write("none");
    } else {
        wnum(c, s->lim_ma);
        c->write(" mA (");
        c->write(climit_src_name(s->lim_src));
        c->write(")");
    }
    c->write(", connected ");
    if (s->lim_conn_ma) {
        wnum(c, s->lim_conn_ma);
        c->write(" mA: ");
        c->write(s->lim_need == CLIMIT_NEED_NO ? "no limit needed\r\n" : "the limit may engage\r\n");
    } else {
        c->write("not known\r\n");
    }
    /* the overcurrent signal on D9 (core/ocled.h) */
    c->write("overcurrent_led=");
    if (s->oc_show == 0u) {
        c->write("off");
    } else {
        static const char *const cause[] = { "", " (short)", " (overload)" };
        c->write(s->oc_show == 2u ? "trip" : "limit");
        c->write(s->oc_show == 2u ? cause[s->oc_cause < 3u ? s->oc_cause : 0] : "");
        c->write(", since ");
        wnum(c, s->oc_since_ms);
        c->write(" ms (");
        wnum(c, s->uptime_ms - s->oc_since_ms);
        c->write(" ms ago), ");
        if (s->oc_show == 2u ? s->oc_tripped : s->oc_limiting) {
            c->write(s->oc_show == 2u ? "outputs off" : "board limit active");
        } else {
            c->write("hold ");
            wnum(c, s->oc_hold_ms);
            c->write(" ms");
        }
    }
    c->write("\r\n");
    /* the power events (core/pwrev.h), decoded from the POWER page bytes a Diff-Power ultra receiver would send */
    {
        static const char *const cause[] = { "", "short", "overload" };
        const uint8_t *g = s->pw_page;
        uint8_t st = s->pw_state, lat = g[ULTRA_PW_LATCHED], cs = (uint8_t)(g[ULTRA_PW_SRC_CAUSE] & 0x0Fu);
        uint32_t age;
        c->write("power_state=");
        c->write((st & (ULTRA_ST_LIMITING | ULTRA_ST_TRIPPED | ULTRA_ST_CH_LIMITING)) ? "" : "ok");
        c->write((st & ULTRA_ST_LIMITING) ? "limiting" : "");
        c->write((st & ULTRA_ST_TRIPPED) ? ((st & ULTRA_ST_LIMITING) ? ", tripped" : "tripped") : "");
        c->write((st & ULTRA_ST_CH_LIMITING) ? ((st & (ULTRA_ST_LIMITING | ULTRA_ST_TRIPPED)) ? ", channel-limiting"
                                                                                           : "channel-limiting") : "");
        if (st & ULTRA_ST_TRIPPED) {
            c->write(" (retry in ");
            wnum(c, g[ULTRA_PW_RETRY_S]);
            c->write(" s)");
        } else if (st & ULTRA_ST_LIMITING) {
            c->write(" (factor ");
            wnum(c, ultra_get16(g + ULTRA_PW_FACTOR) * 1000u / 65535u);
            c->write(" permille)");
        }
        c->write("\r\npower_events=limit ");
        wnum(c, ultra_get16(g + ULTRA_PW_LIMIT_EVENTS));
        c->write(", overload trips ");
        wnum(c, ultra_get16(g + ULTRA_PW_OVL_TRIPS));
        c->write(", short trips ");
        wnum(c, ultra_get16(g + ULTRA_PW_SHORT_TRIPS));
        c->write(lat ? ", latched:" : ", nothing latched");
        c->write((lat & ULTRA_PW_EV_LIMITED) ? " limit" : "");
        c->write((lat & ULTRA_PW_EV_OVERLOAD) ? " overload" : "");
        c->write((lat & ULTRA_PW_EV_SHORT) ? " short" : "");
        c->write("\r\npower_last_limit=");
        age = ultra_get24(g + ULTRA_PW_LIMIT_AGE);
        if (age == PWREV_AGE_NONE) {
            c->write("none");
        } else {
            wnum(c, age);
            c->write(" s ago, ");
            wnum(c, ultra_get16(g + ULTRA_PW_EVENT_MA));
            c->write(" mA");
        }
        c->write("\r\npower_last_trip=");
        age = ultra_get24(g + ULTRA_PW_TRIP_AGE);
        if (age == PWREV_AGE_NONE) {
            c->write("none");
        } else {
            wnum(c, age);
            c->write(" s ago, ");
            c->write(cause[cs < 3u ? cs : 0]);
            c->write(", ");
            wnum(c, ultra_get16(g + ULTRA_PW_TRIP_MA));
            c->write(" mA");
        }
        c->write("\r\n");
    }
    if (c->scene && c->scene->mask) {
        c->write("power-on scene R/G/B/W1/W2=");
        for (i = 0; i < 5u; i++) {
            if (c->scene->mask & (1u << i)) {
                wnum(c, c->scene->v[i]);
            } else {
                c->write("-");
            }
            c->write(i < 4u ? "/" : "");
        }
        c->write(", fade ");
        wnum(c, c->scene->fade * 100u);
        c->write(" ms\r\n");
    }
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
        c->write("status | get | set <key> <value> | period <steps> | save | defaults | identify | version\r\n");
        for (i = 0; i < NKEYS; i++) {
            c->write("  ");
            c->write(keys[i].key);
            c->write(": ");
            c->write(keys[i].help);
            c->write("\r\n");
        }
    } else if (streq(word, "status")) {
        status(c);
    } else if (streq(word, "get")) {
        for (i = 0; i < NKEYS; i++) {
            kv(c, keys[i].key, get_key(c, keys[i].cmd));
        }
        kv(c, "period", settings_steps(c->set));
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
    } else {
        c->write("error: unknown command, try help\r\n");
    }
}

void cli_feed(cli_t *c, char ch)
{
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
