/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* The console's whole output against the plain-text build's (review round 5: the compressed console text, core/cz.h),
 * included by test_main.c after test_console060.c (its capture helpers). A fixed sweep drives every command (each
 * settings key with in- and out-of-range values, the argument and line-length edges, `watch` / any-key) and `status`,
 * `watch`, the banner and the test jig's report through every branch (each reset flag, protocol, decode error,
 * current state, calibration level count, temperature, dip, board limit, D9, power event, health and scene case; 0.6.5:
 * the CPU load, the reset breadcrumbs, the overruns, the event log, the supply impedance, "calibration suggested",
 * the jig line), then 4,000 random states (seeded: every field, the pointers NULL or set). The transcript (each step's
 * name and its output) must match the plain build's byte for byte: its CRC-32 and length, CONSOLE_GOLDEN_* below,
 * come from this same file built against 0.6.5's src/core before the compression (cli.c with plain strings;
 * the 0.6.5 release notes). CONSOLE_DUMP=<file> writes the transcript (diff it against one from that
 * build). */

#define CONSOLE_GOLDEN_CRC 0x9018c899u      /* 0.6.5 with plain strings (this file): 10,549 steps */
#define CONSOLE_GOLDEN_LEN 12200684u

static uint32_t ccz_crc = 0xFFFFFFFFu, ccz_len, ccz_steps;
static uint32_t ccz_uid[3] = { 19088743u, 2309737967u, 4275878552u };
static volatile uint32_t ccz_ovr[3];
static volatile uint16_t ccz_hwd[HW_DIAG_N];
static evlog_t ccz_ev;
static FILE *ccz_dump;
static unsigned long long ccz_rs = 0x2545F4914F6CDD1Dull;

static uint32_t ccz_rnd(void)
{
    ccz_rs ^= ccz_rs << 13;
    ccz_rs ^= ccz_rs >> 7;
    ccz_rs ^= ccz_rs << 17;
    return (uint32_t)(ccz_rs >> 32);
}

static void ccz_add(const char *s)
{
    size_t n = strlen(s), i;
    int k;
    for (i = 0; i < n; i++) {
        ccz_crc ^= (uint8_t)s[i];
        for (k = 0; k < 8; k++) {
            ccz_crc = (ccz_crc >> 1) ^ (0xEDB88320u & (0u - (ccz_crc & 1u)));
        }
    }
    ccz_len += (uint32_t)n;
    if (ccz_dump) {
        fwrite(s, 1, n, ccz_dump);
    }
}

/* one step: its name, then what the console wrote */
static void ccz_take(const char *name)
{
    char h[96];
    snprintf(h, sizeof h, "\n### %u %s\n", (unsigned)ccz_steps++, name);
    ccz_add(h);
    ccz_add(cli_out);
}

static void ccz_cmd(cli_t *c, const char *line)
{
    char name[80], w[16];
    run_cli(c, line);
    snprintf(w, sizeof w, " [watch %u]", (unsigned)c->watch_ms);
    snprintf(name, sizeof name, "%.60s%s", line, c->watch_ms ? w : "");
    ccz_take(name);
    if (c->watch_ms) {                     /* (any key stops it: back to commands) */
        run_cli(c, "\n");
        ccz_take("(key)");
    }
}

static void ccz_status_begin(void) { cap_write("<begin>"); }

static void ccz_status(cli_t *c, const char *name)
{
    run_cli(c, "status\r");
    ccz_take(name);
    cli_out[0] = 0;
    cli_watch(c);
    ccz_take("(watch line)");
}

static void ccz_random(cli_status_t *st, cli_t *c, scene_t *sc, vdip_t *d, pwrev_t *pw, health_t *h)
{
    uint32_t i, l;
    uint8_t *b;
    static const char *const vers[] = { "test", "0.6.4", "0.6.5-beta.1 (development build)", "" };
    static const char *const builds[] = { "20261014", "", "abcdefgh" };
    const energy_t *en = st->en;
    const vsense_t *vs = st->dip_vs;
    b = (uint8_t *)st;                     /* every byte random, then the pointers and the counts made valid */
    for (i = 0; i < sizeof *st; i++) {
        b[i] = (uint8_t)ccz_rnd();
    }
    st->version = vers[ccz_rnd() % 4u];
    snprintf(st->bl_build, sizeof st->bl_build, "%s", builds[ccz_rnd() % 3u]);
    st->bl_version = (uint8_t)(ccz_rnd() % 3u == 0u ? 0u : st->bl_version);
    st->app_state = (uint8_t)(ccz_rnd() % 5u);
    st->proto_mode = (uint8_t)(ccz_rnd() % 6u);
    st->proto_locked = (uint8_t)(ccz_rnd() % 7u);
    st->proto_provisional = (uint8_t)(ccz_rnd() & 1u);
    for (i = 0; i < 6u; i++) {
        st->dec_err[i] = (ccz_rnd() & 1u) ? st->dec_err[i] : 0u;
    }
    if (ccz_rnd() & 1u) st->ultra_headers = st->ultra_tail_err = 0;
    st->steps = 1u + ccz_rnd() % 65535u;
    if (ccz_rnd() & 1u) st->stagger_moves = st->ripple_ma = 0;
    st->i_cal = (uint8_t)(ccz_rnd() % 3u == 0u);
    st->i_tripped = (uint8_t)(ccz_rnd() & 1u);
    st->i_trip_cause = (uint8_t)(ccz_rnd() % 5u);
    st->i_model = (uint8_t)(ccz_rnd() % 5u);
    st->cal.levels = (uint8_t)(ccz_rnd() % 6u);
    st->mcu_temp_c10 = (int16_t)(ccz_rnd() % 3u == 0u ? -32768 : st->mcu_temp_c10);
    st->en = en;
    st->hl = (ccz_rnd() & 1u) ? h : NULL;
    st->dip = (ccz_rnd() & 1u) ? d : NULL;
    st->dip_vs = vs;
    st->test_mode = (uint8_t)(ccz_rnd() & 1u);
    st->lim_ma = (uint16_t)(ccz_rnd() % 3u == 0u ? 0xFFFFu : st->lim_ma);
    st->lim_src = (uint8_t)(ccz_rnd() % 6u);
    st->lim_need = (uint8_t)(ccz_rnd() % 4u);
    if (ccz_rnd() & 1u) st->lim_conn_ma = 0;
    st->oc_show = (uint8_t)(ccz_rnd() % 4u);
    st->oc_cause = (uint8_t)(ccz_rnd() % 5u);
    st->oc_limiting = (uint8_t)(ccz_rnd() & 1u);
    st->oc_tripped = (uint8_t)(ccz_rnd() & 1u);
    st->pw = (ccz_rnd() % 4u) ? pw : NULL;
    st->uid = ccz_uid;                     /* (0.6.5: always set, as main.c) */
    st->ovr = (ccz_rnd() & 1u) ? ccz_ovr : NULL;
    for (i = 0; i < 3u; i++) {
        ccz_ovr[i] = (ccz_rnd() % 3u == 0u) ? 0u : ccz_rnd();
    }
    st->ev = (ccz_rnd() % 4u) ? &ccz_ev : NULL;
    for (i = 0; i < EVLOG_N; i++) {
        ccz_ev.e[i].t_s = ccz_rnd();
        ccz_ev.e[i].ma = (uint16_t)ccz_rnd();
        ccz_ev.e[i].type = (uint8_t)(ccz_rnd() % (EVL_CAL + 1u));
    }
    ccz_ev.head = (uint8_t)(ccz_rnd() % EVLOG_N);
    if (ccz_rnd() & 1u) st->bc.magic = BC_MAGIC;
    st->hwd = (ccz_rnd() & 1u) ? ccz_hwd : NULL;          /* (0.6.5, D6) */
    for (i = 0; i < HW_DIAG_N; i++) {
        ccz_hwd[i] = (uint16_t)((ccz_rnd() % 3u == 0u) ? ccz_rnd() : 0u);
    }
    if (ccz_rnd() & 1u) st->fl_pend = st->fl_errs = 0, st->fl_rej = 0;   /* (0.6.5, D5: often nothing) */
    for (i = 0; i < 5u; i++) {             /* (0.6.5, M3) */
        test_en.ch_mwh[i] = ccz_rnd() >> (ccz_rnd() % 32u);
    }
    cli_set.last_proto = (uint8_t)(ccz_rnd() % 7u);
    c->scene = (ccz_rnd() & 1u) ? sc : NULL;
    /* the scene, the dips, the power events, the health, the lifetime record */
    for (i = 0; i < 5u; i++) {
        sc->v[i] = (uint16_t)ccz_rnd();
    }
    sc->mask = (uint8_t)(ccz_rnd() % 33u);
    sc->fade = (uint8_t)ccz_rnd();
    b = (uint8_t *)d;
    for (i = 0; i < sizeof *d; i++) {
        b[i] = (uint8_t)ccz_rnd();
    }
    d->n_log = (uint8_t)(ccz_rnd() % (VDIP_LOG + 1u));
    d->head = (uint8_t)(ccz_rnd() % VDIP_LOG);
    d->in_dip = (uint8_t)(ccz_rnd() & 1u);
    if (ccz_rnd() & 1u) d->deep.ref_x = 0;
    d->znum = (int32_t)(ccz_rnd() % 0x400000u) - 0x200000;   /* (0.6.5: the impedance's sums, in their range) */
    d->zden = (ccz_rnd() & 0x3FFFFFFu) | 8u;
    if (ccz_rnd() & 1u) d->zsteps = (uint16_t)(ccz_rnd() % 6u);
    for (l = 0; l < VDIP_LOG; l++) {
        d->log[l].ref_x = (uint16_t)(d->log[l].ref_x | 1u);
    }
    b = (uint8_t *)pw;
    for (i = 0; i < sizeof *pw; i++) {
        b[i] = (uint8_t)ccz_rnd();
    }
    pw->limit_on = (uint8_t)(ccz_rnd() & 1u);
    pw->ch_on = (uint8_t)(ccz_rnd() & 1u);
    pw->in.trip_cause = (uint8_t)(ccz_rnd() % 4u);
    pw->last_cause = (uint8_t)(ccz_rnd() % 5u);
    if (ccz_rnd() & 1u) pw->ack_limit = pw->limit_events;
    if (ccz_rnd() & 1u) pw->ack_ovl = pw->ovl_trips;
    if (ccz_rnd() & 1u) pw->ack_short = pw->short_trips;
    if (ccz_rnd() & 1u) pw->ack_dips = pw->in.dips;
    if (ccz_rnd() & 1u) pw->limit_age_s = PWREV_AGE_NONE;
    if (ccz_rnd() & 1u) pw->trip_age_s = PWREV_AGE_NONE;
    b = (uint8_t *)h;
    for (i = 0; i < sizeof *h; i++) {
        b[i] = (uint8_t)ccz_rnd();
    }
    for (i = 0; i < 5u; i++) {
        if (ccz_rnd() % 3u == 0u) h->test_dev[i] = ULTRA_HE_NA;
        if (ccz_rnd() % 3u == 0u) h->model_dev[i] = ULTRA_HE_NA;
    }
    b = (uint8_t *)&test_en.lt;
    for (i = 0; i < sizeof test_en.lt; i++) {
        b[i] = (uint8_t)ccz_rnd();
    }
    test_en.boot_mwh = ccz_rnd();
    test_en.mwh_seen = ccz_rnd();
    test_en.base_wh = ccz_rnd();
}

static void test_console_cz(void)
{
    cli_t c;
    cli_status_t st;
    vsense_t vs;
    vdip_t d;
    health_t h;
    pwrev_t pw;
    scene_t sc;
    char line[96], name[96];
    const char *dump = getenv("CONSOLE_DUMP");
    static const char *const keys[] = { "protocol", "preset", "dither", "thresh", "feedback", "stagger", "floor",
                                        "minpulse", "loss", "smooth", "gamma", "scene", "scenefade", "ilimit", "ical",
                                        "softstart", "nokey", "PRESET", "" };
    static const char *const vals[] = { "0", "1", "2", "3", "4", "5", "8", "13", "14", "15", "20", "100", "101", "254",
                                        "255", "256", "1000", "1000001", "", "x", "-1", " 7", "3x" };
    static const char *const cmds[] = {
        "help", "?", "  help", "help  extra", "HELP", "hel", "helpx", "help?", "get", "get x", "set", "set ",
        "set preset", "set  preset   3", "set preset 2 9", "period", "period 1439", "period 1440", "period 5000",
        "period 65535", "period 65536", "period 99999999", "period abc", "period 05760", "save", "save now",
        "defaults", "identify", "version", "watch", "watch 0", "watch 5", "watch 99", "watch 100", "watch 500",
        "watch 60000", "watch 60001", "watch 99999", "watch x", "statusx", "stat", "unknown", "x", "%", "%s%u%z",
        "abcdefghijklmnopqrstuvwxyz", "set abcdefghijklmnop 1", "set preset 3abcdefghijklmnopqrstuvwxyz",
        "                                                            help",
        "0123456789012345678901234567890123456789012345678901234567890123456789" };
    uint32_t i, k, r;

    printf("console: the whole output against the plain-text build's (compressed text)\n");
    ccz_dump = dump ? fopen(dump, "wb") : NULL;
    memset(&c, 0, sizeof c);
    memset(&st, 0, sizeof st);
    memset(&sc, 0, sizeof sc);
    settings_default(&cli_set);
    config_rx_init(&cli_rx);
    memset(&test_en, 0, sizeof test_en);
    st.version = "test";
    st.steps = 5760;
    st.en = &test_en;
    st.uid = ccz_uid;
    c.write = cap_write;
    c.apply = cap_apply;
    c.save_now = cap_save;
    c.status_begin = ccz_status_begin;
    c.set = &cli_set;
    c.st = &st;
    vsense_init(&vs, 220000u, 10000u, 3280u);
    st.dip_vs = &vs;

    /* commands: every key with every kind of value, then the rest */
    for (i = 0; i < sizeof cmds / sizeof cmds[0]; i++) {
        snprintf(line, sizeof line, "%s\r", cmds[i]);
        ccz_cmd(&c, line);
    }
    for (k = 0; k < sizeof keys / sizeof keys[0]; k++) {
        for (i = 0; i < sizeof vals / sizeof vals[0]; i++) {
            snprintf(line, sizeof line, "set %s %s\n", keys[k], vals[i]);
            ccz_cmd(&c, line);
        }
        ccz_cmd(&c, "get\r");
    }
    c.scene = &sc;                         /* get: the scene keys from the scene, the current keys from st */
    sc.mask = 5;
    sc.fade = 30;
    st.ilimit50 = 77;
    st.softstart = 1;
    ccz_cmd(&c, "get\r");
    c.st = NULL;
    ccz_cmd(&c, "get\r");
    c.st = &st;
    ccz_cmd(&c, "help\r\nget\r\n");        /* two lines in one feed, CR LF */
    run_cli(&c, "statu");                  /* a line in pieces */
    run_cli(&c, "s\n");
    ccz_take("status in two pieces");

    /* status, one branch at a time from a plain state */
    c.scene = NULL;
    ccz_status(&c, "status: plain");
    st.bl_version = 3;
    memcpy(st.bl_build, "20261014", 9);
    st.hw_id = 1;
    st.hw_rev = 3;
    for (i = 0; i < 5u; i++) {
        st.app_state = (uint8_t)i;
        snprintf(name, sizeof name, "status: app_state %u", (unsigned)i);
        ccz_status(&c, name);
        cli_out[0] = 0;
        cli_banner(&c);
        ccz_take("banner");
    }
    for (i = 0; i < 16u; i++) {
        st.reset_flags = (i & 1u ? 0x20000000u : 0u) | (i & 2u ? 0x08000000u : 0u) | (i & 4u ? 0x04000000u : 0u) |
                         (i & 8u ? 0x10000000u : 0u) | 0x00FFFFFFu;
        snprintf(name, sizeof name, "status: reset flags %u", (unsigned)i);
        ccz_status(&c, name);
    }
    for (i = 0; i < 7u; i++) {
        st.proto_mode = (uint8_t)(i % 6u);
        st.proto_locked = (uint8_t)i;
        st.proto_provisional = (uint8_t)(i & 1u);
        cli_set.last_proto = (uint8_t)(6u - i);
        st.dec_err[i % 6u] = 1000u * i + 1u;
        snprintf(name, sizeof name, "status: protocols %u", (unsigned)i);
        ccz_status(&c, name);
    }
    st.ultra_headers = 12345;
    st.stagger_moves = 3;
    st.stagger_o2 = 21845;
    st.stagger_o3 = 43690;
    st.ripple_ma = 1234;
    st.out[0] = 65535;
    st.out[2] = 1;
    st.out[4] = 32768;
    st.i_ma = -5;
    st.i_peak_ma = 12345;
    st.i_zero_ma = 77;
    st.i_limit_ma = 8000;
    st.i_k_permille = 987;
    st.i_trips = 2;
    st.i_ch_ma[0] = -100;
    st.i_ch_ma[1] = 2500;
    st.i_ch_ma[4] = 65000;
    for (i = 0; i < 24u; i++) {            /* current state x trip cause x model */
        st.i_cal = (uint8_t)(i % 3u == 2u);
        st.i_tripped = (uint8_t)(i % 3u == 1u);
        st.i_trip_cause = (uint8_t)(i % 4u);
        st.i_model = (uint8_t)(i % 5u);
        snprintf(name, sizeof name, "status: current %u", (unsigned)i);
        ccz_status(&c, name);
    }
    for (i = 0; i < 6u; i++) {             /* calibration: 0..5 levels */
        st.cal.levels = (uint8_t)i;
        for (k = 0; k < ISENSE_LEVELS; k++) {
            st.cal.pct[k] = (uint8_t)(25u * (k + 1u));
            for (r = 0; r < ISENSE_CH; r++) {
                st.cal.ma[r][k] = (uint16_t)(100u * r + 10u * k + i);
            }
        }
        st.cal_count = (uint16_t)(i * 3u);
        snprintf(name, sizeof name, "status: calibration %u levels", (unsigned)i);
        ccz_status(&c, name);
    }
    {
        static const int16_t temps[] = { -32768, -405, -1, 0, 452, 1250 };
        for (i = 0; i < 6u; i++) {
            st.mcu_temp_c10 = temps[i];
            st.mcu_peak_c10 = (int16_t)(temps[i] + 7);
            st.vdda_mv = (uint16_t)(3300u - i);
            st.uptime_s = 90061u * i;
            st.mcu_peak_s = 1000u * i;
            snprintf(name, sizeof name, "status: temperature %d", temps[i]);
            ccz_status(&c, name);
        }
    }
    vdip_init(&d);                         /* dips: none yet, the deepest only, the log filling up, in a dip */
    st.dip = &d;
    st.dip_ref_mv = 24020;
    for (i = 0; i <= VDIP_LOG + 1u; i++) {
        d.count = (uint16_t)i;
        d.in_dip = (uint8_t)(i & 1u);
        if (i) {
            d.deep.t_s = 1u;
            d.deep.min_x = 1800;
            d.deep.ref_x = 2604;
            d.deep.dur_ms = 12;
            d.deep.i_ma = 6200;
            d.log[d.head] = d.deep;
            d.log[d.head].t_s = 100u * i;
            d.log[d.head].min_x = (uint16_t)(2000u + i);
            d.log[d.head].dur_ms = (uint16_t)(i == 3u ? 65535u : 40u);
            d.head = (uint8_t)((d.head + 1u) % VDIP_LOG);
            d.n_log = (uint8_t)(d.n_log < VDIP_LOG ? d.n_log + 1u : VDIP_LOG);
        }
        snprintf(name, sizeof name, "status: dips %u", (unsigned)i);
        ccz_status(&c, name);
    }
    for (i = 0; i < 2u * 6u * 4u; i++) {   /* board limit: none / a source x connected x need, test mode */
        st.lim_ma = (uint16_t)(i < 4u ? 0xFFFFu : 4000u + i);
        st.lim_src = (uint8_t)(i % 6u);
        st.lim_conn_ma = (i / 6u) & 1u ? 0u : 9000u;
        st.lim_need = (uint8_t)((i / 12u) % 4u);
        st.test_mode = (uint8_t)(i & 1u);
        st.vs_supply_mv = 24000u + i;
        st.vs_aux_mv = 5000u * i;
        snprintf(name, sizeof name, "status: board limit %u", (unsigned)i);
        ccz_status(&c, name);
    }
    for (i = 0; i < 4u * 5u * 4u; i++) {   /* D9: off / limit / trip / 3 x cause x the condition now */
        st.oc_show = (uint8_t)(i % 4u);
        st.oc_cause = (uint8_t)((i / 4u) % 5u);
        st.oc_limiting = (uint8_t)((i / 20u) & 1u);
        st.oc_tripped = (uint8_t)((i / 40u) & 1u);
        st.oc_since_s = 3u * i;
        st.oc_hold_ms = 1234u * i;
        snprintf(name, sizeof name, "status: D9 %u", (unsigned)i);
        ccz_status(&c, name);
    }
    pwrev_init(&pw, 0);                    /* power events: every state, latch, age and cause */
    st.pw = &pw;
    for (i = 0; i < 256u; i++) {
        pw.limit_on = (uint8_t)(i & 1u);
        pw.in.trip_cause = (uint8_t)((i >> 1) & 1u ? 1u + (i % 2u) : 0u);
        pw.ch_on = (uint8_t)((i >> 2) & 1u);
        pw.limit_events = (uint16_t)((i >> 3) & 1u);
        pw.ovl_trips = (uint16_t)((i >> 4) & 1u);
        pw.short_trips = (uint16_t)((i >> 5) & 1u);
        pw.in.dips = (uint16_t)((i >> 6) & 1u);
        pw.limit_age_s = (i >> 7) & 1u ? 17u : PWREV_AGE_NONE;
        pw.trip_age_s = (i % 3u) ? 1000u * i : PWREV_AGE_NONE;
        pw.last_cause = (uint8_t)(i % 5u);
        pw.in.retry_ms = 1001u * i;
        pw.in.factor = (uint16_t)(257u * i);
        pw.event_ma = (uint16_t)(30u * i);
        pw.trip_ma = (uint16_t)(60u * i);
        snprintf(name, sizeof name, "status: power events %u", (unsigned)i);
        ccz_status(&c, name);
    }
    health_init(&h);                       /* health: every class and reason, dark flag, not tested */
    st.hl = &h;
    for (i = 0; i < 64u; i++) {
        h.flags = (uint8_t)((i & 1u ? ULTRA_HE_F_TESTED : 0u) | (i & 2u ? ULTRA_HE_F_DARK : 0u));
        for (k = 0; k < 5u; k++) {
            h.cls[k] = (uint8_t)(((i + k) & 3u) | (((i >> 2) + k) & 7u) << 2);
            h.test_dev[k] = (int8_t)(k == (i % 5u) ? ULTRA_HE_NA : (int)(i * 7u + k) - 128);
            h.model_dev[k] = (int8_t)(k == ((i + 2u) % 5u) ? ULTRA_HE_NA : (int)(k * 9u) - (int)i);
        }
        h.dark_ma = (int16_t)(i * 50 - 1000);
        snprintf(name, sizeof name, "status: health %u", (unsigned)i);
        ccz_status(&c, name);
    }
    c.scene = &sc;                         /* scene: every mask */
    for (i = 0; i < 33u; i++) {
        sc.mask = (uint8_t)i;
        for (k = 0; k < 5u; k++) {
            sc.v[k] = (uint16_t)(2000u * i + 13u * k);
        }
        sc.fade = (uint8_t)(i * 7u);
        snprintf(name, sizeof name, "status: scene %u", (unsigned)i);
        ccz_status(&c, name);
    }
    for (i = 0; i < 16u; i++) {            /* (0.6.5) CPU load, breadcrumbs (magic x power-on), overruns */
        st.cpu_pm = (uint16_t)(i * 67u);
        st.cpu_peak = (uint16_t)(1000u - i);
        st.bc.magic = (i & 1u) ? BC_MAGIC : 0x12345678u;
        st.bc.up_s = 90123u * i;
        st.bc.pc = 0x4A2Eu + i;
        st.bc.cause = i & 2u ? 5u : 0u;
        st.reset_flags = (i & 4u) ? 0x08000000u : 0x20000000u;
        st.ovr = (i & 8u) ? ccz_ovr : NULL;
        ccz_ovr[0] = i & 1u;
        ccz_ovr[1] = 7u * (i & 2u);
        ccz_ovr[2] = 11u * (i & 4u);
        snprintf(name, sizeof name, "status: cpu / before_reset / overruns %u", (unsigned)i);
        ccz_status(&c, name);
    }
    st.ev = &ccz_ev;                       /* (0.6.5) the event log: empty, filling up, wrapped */
    memset(&ccz_ev, 0, sizeof ccz_ev);
    for (i = 0; i < EVLOG_N + 3u; i++) {
        ccz_ev.head = (uint8_t)((ccz_ev.head + (i != 0u)) & (EVLOG_N - 1u));
        if (i) {
            ccz_ev.e[ccz_ev.head].t_s = 10u * i;
            ccz_ev.e[ccz_ev.head].ma = (uint16_t)(1234u * i);
            ccz_ev.e[ccz_ev.head].type = (uint8_t)(1u + i % EVL_CAL);
        }
        st.uptime_s = 100u * i;
        snprintf(name, sizeof name, "status: event log %u", (unsigned)i);
        ccz_status(&c, name);
    }
    st.dip = &d;                           /* (0.6.5) the supply impedance: below / at the steps it needs, values */
    for (i = 0; i < 8u; i++) {
        d.zsteps = (uint16_t)(i < 6u ? i : 0xFFFFu);
        d.znum = (int32_t)(i * 9000u) - 4000;
        d.zden = 8u + 60000u * i;
        snprintf(name, sizeof name, "status: impedance %u", (unsigned)i);
        ccz_status(&c, name);
    }
    for (i = 0; i < 24u; i++) {            /* (0.6.5, D5) the flash line: pending, rejected, errors and codes */
        st.fl_pend = (uint8_t)(i & 31u);
        st.fl_rej = (i & 1u) ? 0u : (uint32_t)i * 0x01041041u;
        st.fl_errs = (uint16_t)(i % 3u);
        st.fl_err = (uint8_t)(i % 9u);
        snprintf(name, sizeof name, "status: flash state %u", (unsigned)i);
        ccz_status(&c, name);
    }
    st.fl_pend = st.fl_errs = st.fl_err = 0;
    st.fl_rej = 0;
    st.hwd = ccz_hwd;                      /* (0.6.5, D6) ADC / DMA counters: none, one, all */
    for (i = 0; i <= HW_DIAG_N; i++) {
        for (k = 0; k < HW_DIAG_N; k++) {
            ccz_hwd[k] = (uint16_t)(i == HW_DIAG_N ? 65535u - k : k + 1u == i ? 7u * i : 0u);
        }
        snprintf(name, sizeof name, "status: adc / dma %u", (unsigned)i);
        ccz_status(&c, name);
    }
    st.hwd = NULL;
    st.stagger_moves = 1;                  /* (0.6.5, M5) C10's load with the stagger lines; (M3) energy by channel */
    for (i = 0; i < 6u; i++) {
        st.cap.now = (uint16_t)(37u * i);
        st.cap.avg = (uint16_t)(29u * i);
        st.cap.peak = (uint16_t)(i == 5u ? 65535u : 41u * i);
        for (k = 0; k < 5u; k++) {
            test_en.ch_mwh[k] = (uint32_t)(i * 1234567u + k * 89u);
        }
        snprintf(name, sizeof name, "status: c10 / energy by channel %u", (unsigned)i);
        ccz_status(&c, name);
    }
    memset(test_en.ch_mwh, 0, sizeof test_en.ch_mwh);
    st.hl = &h;                            /* (0.6.5, M1) calibration suggested */
    for (i = 0; i < 4u; i++) {
        h.flags = (uint8_t)((i & 1u ? HEALTH_F_CAL : 0u) | (i & 2u ? ULTRA_HE_F_DARK : 0u));
        snprintf(name, sizeof name, "status: calibration suggested %u", (unsigned)i);
        ccz_status(&c, name);
    }
    for (i = 0; i < 12u; i++) {            /* the test jig's report: pct 0 / any, supply known or not */
        int32_t ma[5];
        for (k = 0; k < 5u; k++) {
            ma[k] = (int32_t)(ccz_rnd() % 9000u) - 500;
        }
        cli_out[0] = 0;
        cli_test_report(&c, ma, i == 0u ? 0u : 9u * i, 100u + i, (i & 1u) ? 23950u + i : 0u);
        snprintf(name, sizeof name, "test report %u", (unsigned)i);
        ccz_take(name);
    }

    /* random states */
    for (i = 0; i < 4000u; i++) {
        ccz_random(&st, &c, &sc, &d, &pw, &h);
        snprintf(name, sizeof name, "status: random %u", (unsigned)i);
        ccz_status(&c, name);
        if (i % 16u == 0u) {
            int32_t ma[5];
            for (k = 0; k < 5u; k++) {
                ma[k] = (int32_t)ccz_rnd();
            }
            cli_out[0] = 0;
            cli_test_report(&c, ma, ccz_rnd() % 120u, ccz_rnd(), (ccz_rnd() & 1u) ? ccz_rnd() : 0u);
            ccz_take("test report: random");
            ccz_cmd(&c, "get\r");
            ccz_cmd(&c, "version\r");
        }
    }
    if (ccz_dump) {
        fclose(ccz_dump);
    }
    ccz_crc = ~ccz_crc;
    printf("  transcript: %u steps, %u bytes, CRC-32 %08x\n", (unsigned)ccz_steps, (unsigned)ccz_len,
           (unsigned)ccz_crc);
    CHECK(ccz_crc == CONSOLE_GOLDEN_CRC && ccz_len == CONSOLE_GOLDEN_LEN,
          "console transcript differs from the plain-text build's (%08x / %u bytes, want %08x / %u): CONSOLE_DUMP=<file> "
          "and diff",
          (unsigned)ccz_crc, (unsigned)ccz_len, (unsigned)CONSOLE_GOLDEN_CRC, (unsigned)CONSOLE_GOLDEN_LEN);
}
