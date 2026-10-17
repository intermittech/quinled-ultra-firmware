/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* commands.c: what a config frame or console command does: the power-on scene commands, the retry policy, soft start,
 * handle_result, cli_apply, the D9 restore.
 * A unit of src/app/main.c (round 12, S10): main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

/* config frame / console: power-on scene commands (and factory reset). Written after the normal debounce. */
static cfg_event_t scene_command(uint8_t cmd, uint8_t arg)
{
    scene_t s = scene_want;
    if (cmd == CMD_SCENE_FADE) {
        s.fade = arg;
    } else if (cmd == CMD_SCENE && arg == 1u) {
        shown_outputs(s.v);
        s.mask = 0x1Fu;
    } else {
        s.mask = 0;                        /* none: dark at power-up */
        if (cmd == CMD_FACTORY_RESET) {
            s.fade = SCENE_FADE_DEFAULT;
        }
    }
    if (scene_equal(&s, &scene_want)) {
        return CFG_SAME;
    }
    scene_want = s;
    scene_pend = (uint8_t)!scene_equal(&s, &scene_stored);
    scene_major_pend = 1;
    scene_changed_ms = ms_now - (SCENE_SAVE_MS - SAVE_DEBOUNCE_MS);
    return CFG_ACK;
}

/* (0.6.9, B7) the trips' retry policy (config frames 0x15 / 0x16, Ultra CONFIG 24): stored in the settings extension's
 * spare bytes, coded so that an erased byte (0xFF) is the default: 10 retries, 1 s apart */
static uint8_t retry_set(uint32_t count, uint32_t secs)
{
    uint8_t n = (uint8_t)(count ^ RETRY_N_X), t = (uint8_t)(secs ^ RETRY_S_X);
    if (n == sext.reserved[2] && t == sext.reserved[3]) {
        return 0;
    }
    sext.reserved[2] = n;
    sext.reserved[3] = t;
    is.retry_max = (uint8_t)count;
    is.retry_s = (uint8_t)secs;
    settings_changed();
    return 1;
}

/* soft start on / off (console, config frame, Ultra CONFIG 23): stored with the settings; 1 = changed */
static uint8_t soft_start_set(uint32_t on)
{
    uint8_t o = (uint8_t)(on ? sext.opts | STORE_OPT_SOFT_START : sext.opts & ~STORE_OPT_SOFT_START);
    if (o == sext.opts) {
        return 0;
    }
    sext.opts = o;
    settings_changed();
    return 1;
}

static cfg_result_t handle_result(cfg_result_t r)
{
    if (r.event == CFG_ACK && r.cmd == CMD_ISENSE_CAL) {
        if (!isense_calibrate(r.arg)) {
            r.event = CFG_NACK;            /* tripped: not now */
            ack_start(&ack, ACK_P_NACK, 0);
        } else {
            stat.config_frames++;
        }
        return r;
    }
    if (r.event == CFG_ACK && (r.cmd == CMD_ISENSE_LIMIT || r.cmd == CMD_RETRY_COUNT || r.cmd == CMD_RETRY_S ||
                               r.cmd == CMD_SOFT_START || r.cmd == CMD_SCENE || r.cmd == CMD_SCENE_FADE)) {
        /* the values main.c keeps (the settings extension; the scene, not a setting: no settings write): ACK when
         * it changed (config.c: the value code of SOFT_START / SCENE, else 0), SAME when not (one tail: 0.6.10) */
        if (r.cmd == CMD_ISENSE_LIMIT) {
            if (sext.ilimit50 == r.arg || (r.arg == 0u && sext.ilimit50 == 0xFFu)) {
                r.event = CFG_SAME;
            } else {
                sext.ilimit50 = r.arg;     /* stored with the settings */
                limit_update();
                settings_changed();
            }
        } else if (r.cmd == CMD_SOFT_START) {
            r.event = soft_start_set(r.arg) ? CFG_ACK : CFG_SAME;
        } else if (r.cmd == CMD_SCENE || r.cmd == CMD_SCENE_FADE) {
            r.event = scene_command(r.cmd, r.arg);
        } else {
            r.event = retry_set(r.cmd == CMD_RETRY_COUNT ? r.arg : is.retry_max,
                                r.cmd == CMD_RETRY_S ? r.arg : is.retry_s) ? CFG_ACK : CFG_SAME;
        }
        ack_start(&ack, r.event == CFG_ACK ? ACK_P_ACK : ACK_P_SAME, r.event == CFG_ACK ? r.value_code : 0u);
        stat.config_frames += r.event == CFG_ACK;
        return r;
    }
    switch (r.event) {
    case CFG_ACK:
        ack_start(&ack, r.value_code == 0xFFu ? ACK_P_FACTORY : ACK_P_ACK, r.value_code == 0xFFu ? 0u : r.value_code);
        if (r.cmd == CMD_PROTOCOL || r.cmd == CMD_FACTORY_RESET) {
            proto_set_mode(&frx.ps, (proto_mode_t)set.protocol);   /* a config frame / the console: at once */
        }
        if (r.cmd == CMD_FACTORY_RESET) {
            (void)scene_command(CMD_FACTORY_RESET, 0);
            store_ext_clear(&sext);        /* current limit and calibration too (the model relearns) */
            /* (review round 6, F10: like a reboot) the stores it clears are void until their clears are stored: the
             * settings record (the defaults) carries it, written at once (reset_save; a program, as a user's
             * command); the boot doesn't load a void store (records_load) */
            STORE_VOIDS(&sext) = (uint8_t)~((scene_pend ? STORE_VOID_SCENE : 0u) | (fx_mask ? STORE_VOID_FX : 0u) |
                                            (cal_present ? STORE_VOID_CAL : 0u)
#if MON_EXTRA
                                            | (chist.n ? STORE_VOID_HIST : 0u)
#endif
                                            );
            reset_save = 1;
            /* and the power-on effects and the stored calibration (never the lifetime record): in the flash's next
             * low moment (0.6.4: an erase; 0.6.3 at once, the outputs as they were). The acknowledge pattern's final
             * dark second is usually that moment. */
            fx_clear_pend = 1;
            fx_mask = 0;
            /* (review round 6, fable-sys 04) the power-on effect and the stored scene stop showing: a reset board shows
             * nothing until data, as its flash now says (0.6.6 kept both running from RAM until a power cycle) */
            if (fx_ch) {
                ultra_scene_fx_stop(&ul);
                fx_ch = 0;
            }
            srun.hold = srun.fading = 0;
            outputs_dirty = 1;
            if (cal_present) {
                cal_save_pend = 2;         /* a cleared record (levels 0) */
                cal_present = 0;
            }
#if MON_EXTRA
            chist.n = 0;                   /* (M2) the calibration history begins anew (written in a low moment) */
            chist.tests = 0;
            chist_pend = 1;
#endif
            isense_setup();
        }
        apply_settings();
        settings_changed();
        stat.config_frames++;
        break;
    case CFG_SAME:
        ack_start(&ack, ACK_P_SAME, 0);
        break;
    case CFG_NACK:
        ack_start(&ack, ACK_P_NACK, 0);
        break;
    case CFG_IDENTIFY:
        if (r.arg == CFG_IDENTIFY_VERSION) {
            ack_start_version(&ack, fw_version);   /* config frame / console `version` */
        } else {
            ack_start(&ack, ACK_P_IDENTIFY, 0);
        }
        break;
    default:
        break;
    }
    return r;
}

static cfg_result_t cli_apply(uint8_t cmd, uint8_t arg)
{
    return handle_result(config_apply(&rx, &set, cmd, arg));
}

#if MON_EXTRA && CONSOLE_FULL
/* (0.6.7, D9) the console's `restore` (core/cli.c checked the length, CRC, version, board): the values as a console
 * change sets them. The settings: none out of range (settings_sanitize), in effect at once, saved after the debounce
 * through the store's budget and the flash policy (pending meanwhile); the calibration (when the backup holds one,
 * not while a test runs): as a finished test leaves it, stored through its budget. 1 = taken. */
static int restore_rec(const uint8_t *d)
{
    settings_t s;
    isense_cal_t c;
    memcpy(&s, d, sizeof s);
    memcpy(&c, d + sizeof s + sizeof sext, sizeof c);
    if (settings_sanitize(&s) ||
        (c.levels != 0u && (is.cal_step != 0u || (c.levels != 1u && c.levels != ISENSE_LEVELS) ||
                            (c.levels == 1u && c.pct[0] - 5u > 95u)))) {
        return 0;                          /* (the calibration: isense_cal_set's own conditions, it would ignore it) */
    }
    set = s;
    {
        uint8_t v = STORE_VOIDS(&sext);    /* (r6: this board's pending clears, not the backup's) */
        memcpy(&sext, d + sizeof s, sizeof sext);
        STORE_VOIDS(&sext) = v;
    }
    proto_set_mode(&frx.ps, (proto_mode_t)set.protocol);
    apply_settings();
    settings_changed();
    if (c.levels != 0u) {
        isense_cal_set(&is, &c);
        cal_present = 1;
        cal_save_pend = 1;
        limit_update();
    }
    return 1;
}
#endif
