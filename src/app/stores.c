/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* stores.c: the writes: a flash operation's stall, the settings, the power-on scene and effects, their records
 * (scene_service).
 * A unit of src/app/main.c (round 12, S10): main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

/* A flash operation (flash_may's answer `may`): the frame tracking stops meanwhile (the tick runs from SRAM: no frames,
 * the gate closed; downstream boards miss a frame or two, the next idle gap or frame end re-arms it), so a capture in
 * progress is dropped. The time bases run on (0.6.4; 0.6.3 added an estimate). */
static NOINLINE void track_stop(void)       /* (before and after it; out of line: -18 B) */
{
    __disable_irq();
    ftrack_disarm(&ft);
    cap_ready = 0;
    __enable_irq();
}

static void flash_stall_begin(uint32_t may)
{
    ovr_on = 0;                                          /* (the render ms it delays: not an overrun) */
#if BOARD_RETURN
    while (rc_state != RC_IDLE) {
        watchdog_kick();                                 /* (a dip's or the console's write) after the window */
    }
    rc_pend = 0;                                         /* this frame's end is lost: no window */
#endif
    if (may == FM_BRIGHT) {
        (void)budget_take(&flash_bright, ms_now, 1u);    /* the hour's write while brighter */
    }
    track_stop();
}

/* ... done with result r (core/flashrec.h): 1 = written */
static uint32_t flash_stall_end(uint32_t r)
{
    track_stop();
    ultra_clock_gap(&ul);
    flash_recheck = 1;                                   /* (whatever the result: a store's position may have moved) */
    if (r == FLASH_OK) {
        stat.saves++;
        if (!flash_held()) {
            flash_err = 0;                 /* (r6) a success ends it, but not a program (FM_PROG: a dip's, a factory
                                              reset's) inside the hold-off: the failing operation would come again */
        }
    } else if (r != FLASH_NONE) {
        flash_err = (uint8_t)(flash_err + (flash_err <= FLASH_RETRY_MAX));   /* (r6: the hold-off, doubling) */
        flash_err_ms = ms_now;
        say(Z(m_flash_err, "flash: error\r\n"));      /* (the record stays pending; its budget asks again) */
#if MON_EXTRA
        stat.fl_err = (uint8_t)r;                        /* (0.6.5, D5) */
        stat.fl_errs++;
#endif
    }
    return r == FLASH_OK;
}

/* the settings record (may: flash_may's answer for it; the store erases the next page first when that is due):
 * 1 = written */
static uint32_t settings_write(uint32_t may)
{
    flash_stall_begin(may);
    if (flash_stall_end(settings_store(&set, &sext))) {
        settings_dirty = 0;
        return 1;
    }
    return 0;
}

#if CONSOLE_FULL
/* the console's `save`: at once (a program: a user's command), unless the next page needs its erase while the outputs
 * are bright: then it is written in the next low moment */
static void save_now(void)
{
    uint32_t t0 = ms_now, may;
    settings_dirty = 1;
    changed_ms = ms_now - SAVE_DEBOUNCE_MS;
    while (uart_busy() && ms_now - t0 < 100u) {          /* (the console's line first: hw.c flash_op) */
        uart_kick();
        watchdog_kick();
    }
    may = flash_may(settings_pending() == 2u, FM_PROG);
    cli_say(&cli, may == 0u ? ZC(m_pending, "pending\r\n") : settings_write(may) ? ZC(ok_crlf, "ok\r\n")
                                                          : ZC(m_failed, "failed\r\n"));
}
#endif

/* (r6, F10) a store voided by a factory reset (core/store.h STORE_VOIDS) is written: valid again, stored with the next
 * settings write */
static void void_done(uint8_t b)
{
    if (!(STORE_VOIDS(&sext) & b)) {
        STORE_VOIDS(&sext) |= b;
        settings_changed();
    }
}

static void scene_save_now(uint32_t may)
{
    flash_stall_begin(may);
    if (flash_stall_end(scene_store(&scene_want))) {
        scene_stored = scene_want;
        scene_pend = 0;
        void_done(STORE_VOID_SCENE);
    }
}

/* the power-on effects to store (of the lights a SCENE_EFFECT reached since the engine's init): their mask, their
 * CRC-8s into crc; *any = a host sent SCENE_EFFECT at all */
static uint8_t fx_want(uint8_t *crc, uint8_t *any)
{
    uint8_t mask = 0;
    uint32_t i;
    *any = 0;
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        const ultra_light_t *L = &ul.light[i];
        if (L->scene_fx_have) {
            *any = 1;
        }
        if (L->scene_fx_have && ultra_scene_fx_ok(L)) {
            crc[i] = crc8_smbus((const uint8_t *)L, ULTRA_HEAD_BYTES);
            mask |= (uint8_t)(1u << i);
        } else {
            crc[i] = 0;
        }
    }
    return mask;
}

/* what the engine holds is what the store holds (fx_mask, fx_crc)? Compared light by light: a sum of the CRCs
 * (0.6.2) let two lights' changes cancel each other out (CRC deltas d0 = -16 d1 ...), so that the new effects
 * were never written */
static int fx_stored(uint8_t mask, const uint8_t *crc)
{
    uint32_t i;
    if (mask != fx_mask) {
        return 0;
    }
    for (i = 0; i < ULTRA_LIGHTS; i++) {
        if ((mask & (1u << i)) && crc[i] != fx_crc[i]) {
            return 0;
        }
    }
    return 1;
}

/* store the power-on effects (budget permitting): only once a host sent SCENE_EFFECT (a host without it never
 * changes them), and only when they differ from what is stored. fx_pend 1: compare (after a quiet change), 2: differs,
 * waiting for a token (only the budget is checked meanwhile) */
static void fx_service(void)
{
    uint8_t crc[ULTRA_LIGHTS], mask, any;
    uint32_t i;
    if (fx_pend == 0u ||
        (fx_pend == 2u && (flash_may(1u, 0u) != FM_LOW || !budget_take(&fx_budget, ms_now, 1u)))) {
        return;                            /* (two erases: only in a low moment, 0.6.4) */
    }
    mask = fx_want(crc, &any);
    if (!any || fx_stored(mask, crc)) {
        fx_pend = 0;                       /* nothing to write */
        return;
    }
    if (fx_pend == 1u) {
        fx_pend = 2;                       /* differs: written with the next token (a token per 6 h) */
        return;
    }
    fx_pend = 0;
    flash_stall_begin(FM_LOW);
    if (flash_stall_end(scene_fx_store(&ul, mask, crc))) {
        fx_mask = mask;
        for (i = 0; i < ULTRA_LIGHTS; i++) {
            fx_crc[i] = crc[i];
        }
        fx_clear_pend = 0;                 /* (it erased the store anyway) */
        void_done(STORE_VOID_FX);
    } else {
        fx_mask = 0;                       /* failed: nothing valid stored; again with the next token */
        fx_pend = 2;
    }
}

/* every ms while the power-on effect runs: its channels follow the engine until the take-over (then the scene's
 * cross-fade starts from where the effect was); when none is left, the engine's power-on effect stops */
static void fx_follow(void)
{
    uint8_t i;
    fx_ch &= srun.hold;
    for (i = 0; i < 5u; i++) {
        if (fx_ch & (1u << i)) {
            srun.sc.v[i] = uout[i];
        }
    }
    if (fx_ch == 0u) {
        ultra_scene_fx_stop(&ul);
    }
}

/* every ms: Ultra take-over deadline, recompute the scene from the lights' SCENE values once things are quiet,
 * write it when due (SCENE_SAVE_MS after the last change, within the write budget: core/scene.h), not while a
 * pattern blinks (except its final dark pause) */
/* (round 9) inline, as LTO had it: after the hw.c split it left the standalone image's copy out of line (a call and
 * its frame every ms: tools/mainloop.py's ms pass 1,564 -> 1,602 cycles; the app images had it inlined either way,
 * and are byte-identical with the attribute) */
static ALWAYS_INLINE void scene_service(void)
{
    uint32_t may;
    if (ultra_on && srun.hold && ultra_seen && ms_now - ultra_first_ms >= SCENE_TAKEOVER_MS) {
        scene_takeover(&srun, 0x1Fu, ms_now);     /* the host is there but sends nothing for these channels */
        outputs_dirty = 1;
    }
    if (scene_check && ultra_on && ms_now - scene_check_ms >= SCENE_SAVE_MS) {
        scene_t sc;
        scene_check = 0;
        fx_pend = 1;                       /* and the power-on effects (fx_service) */
        ultra_scene(&ul, &scene_stored, &sc);
        scene_want = sc;
        scene_pend = (uint8_t)!scene_equal(&sc, &scene_stored);
        scene_major_pend = (uint8_t)scene_major(&scene_stored, &sc);
        scene_changed_ms = scene_check_ms;
    }
    if (scene_pend && (!ack.active || ack.seg + 1u == ack.nseg) &&
        (may = flash_may(scene_pending() == 2u, 0u)) != 0u && scene_save_due(&sbudget, ms_now, scene_changed_ms, scene_major_pend)) {
        scene_save_now(may);
    }
}
