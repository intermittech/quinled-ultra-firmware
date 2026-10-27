/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* stores.c: the writes: a flash operation's stall, the settings, the power-on scene and effects, their records
 * (scene_service).
 * A unit of src/app/main.c: main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

/* A flash operation (flash_may's answer `may`): the frame tracking stops meanwhile (the tick runs from SRAM: no frames,
 * the gate closed; downstream boards miss a frame or two, the next idle gap or frame end re-arms it), so a capture in
 * progress is dropped. The time bases run on. */
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
    (void)may;
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
            flash_err = 0;                 /* a success ends it, but not a program (FM_PROG: a dip's, a factory
                                              reset's) inside the hold-off: the failing operation would come again */
        }
    } else if (r != FLASH_NONE) {
        flash_err = (uint8_t)(flash_err + (flash_err <= FLASH_RETRY_MAX));   /* (the hold-off, doubling) */
        flash_err_ms = ms_now;
        say(Z(m_flash_err, "flash: error\r\n"));      /* (the record stays pending; its budget asks again) */
#if MON_EXTRA
        stat.fl_err = (uint8_t)r;
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

/* (F10) a store voided by a factory reset (core/store.h STORE_VOIDS) is written: valid again, stored with the next
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

/* every ms: Ultra take-over deadline, recompute the scene from the lights' SCENE values once things are quiet,
 * write it when due (SCENE_SAVE_MS after the last change, within the write budget: core/scene.h), not while a
 * pattern blinks (except its final dark pause) */
/* inline, as LTO had it: after the hw.c split it left the standalone image's copy out of line (a call and
 * its frame every ms: tools/mainloop.py's ms pass 1,564 -> 1,602 cycles; the app images had it inlined either way,
 * and are byte-identical with the attribute) */
static ALWAYS_INLINE void scene_service(void)
{
    uint32_t may;
    if (ultra_on && srun.hold && ultra_seen && ms_now - ultra_first_ms >= SCENE_TAKEOVER_MS) {
        scene_takeover(&srun, 0x1Fu, ms_now, 0, 0);     /* the host is there but sends nothing for these channels */
        outputs_dirty = 1;
    }
    if (scene_check && ultra_on && ms_now - scene_check_ms >= SCENE_SAVE_MS) {
        scene_t sc;
        scene_check = 0;
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
