/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* flash_policy.c: the flash: when (0.6.4): flash_may and its policy, why a write waits (save_why), the erase ahead, the
 * lifetime record's admission.
 * A unit of src/app/main.c (round 12, S10): main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

/* ------------------------------------------------------------------------------- the flash: when (0.6.4)
 * Quindor's policy (2026-10-14). An erase / program halves HCLK (RM 32.1): the PWM runs at half frequency, the duty
 * unchanged, and the main loop waits (renders, fades) for as long as it takes; the current trip keeps running (hw.c:
 * from SRAM). So:
 *  - low: every output below 15 % of full scale (dark, or really dimmed down: the halved frequency is least visible
 *    there) for FLASH_LOW_MS, no calibration / test jig pattern / soft start, nothing fading up (a WS ramp's target, an
 *    Ultra ramp that ends brighter than it shows); and at power-up before the main loop: anything, erases too;
 *  - brighter: a program into erased flash only (no erase), at most one per FLASH_BRIGHT_MS (pending changes wait:
 *    the newest values are written); erases wait for a low moment, which also erases each store's next page ahead;
 *  - at a supply dip: programs into erased flash (the lifetime record's dip write, any store's pending write);
 *  - the console's `save`: a program at once (a user's command, not housekeeping).
 * "Below 15 %" is the outputs as the tables hold them (out_now: transitions and effects as they show), which is what
 * the PWM runs while the main loop waits: nothing can cross the level during an operation. Every regular write also
 * waits until the console sends nothing (hw.c flash_op: the baud halves with HCLK). */
#define FLASH_LOW_LEVEL  9830u             /* 15 % of 65535 */
#define FLASH_LOW_MS     500u              /* every output below it this long: an effect's peaks reset it */
#define FLASH_BRIGHT_MS  3600000u          /* brighter: one write per hour */
/* dip writes (Quindor 2026-10-14): a program into an erased entry of an A/B store, the check written last, never an
 * erase, only with VDD above the PVD's 2.9 V (hw.c flash_op); a cut leaves the previous record (test r5dip). 0: no
 * write at all during a dip (when the bench finds the hold-up shorter than a record's write: README, bench). */
#ifndef FLASH_DIP_WRITES
#define FLASH_DIP_WRITES 1
#endif
#define FM_LOW    1u                       /* flash_may: low (or boot): erases too */
#define FM_PROG   2u                       /* a program into erased flash (a dip, the console's save) */
#define FM_BRIGHT 3u                       /* ... brighter: with the hour's token */
/* (review round 6) after a failed operation no regular write or erase for a while: FLASH_RETRY_MS after the first
 * failure, doubling with each failure in a row up to FLASH_RETRY_MS << FLASH_RETRY_MAX (~11 min); a success after it
 * ends it.
 * A flash that keeps failing (write protection over the record sectors, a worn page, an operation without EOP) must
 * not take the main loop nor wear a page out: 0.6.6 retried an erase ahead / the effects' clear every ms while low
 * and the calibration history's write every pass, so every frame was lost (the tracker disarmed each time), `flash:
 * error` filled the console, and an erase whose status alone failed ran ~60 times a second. A supply dip's write and
 * the console's `save` (FM_PROG) still try at once: the last chance, a user's command. */
#define FLASH_RETRY_MS   10000u
#define FLASH_RETRY_MAX  6u
static uint32_t flash_hi_ms;               /* the last ms the outputs were not low */
static uint32_t flash_err_ms;              /* (r6) the last failed operation; flash_err: failures in a row (0: none) */
static uint8_t flash_err;
static scene_budget_t flash_bright;        /* the writes while brighter */
static uint8_t flash_boot = 1;             /* before the main loop: the outputs are dark */
static uint8_t fx_clear_pend;              /* a factory reset: the power-on effects' erase is due */
static uint8_t reset_save;                 /* (r6, F10) a factory reset: its settings record (the defaults, the void
                                              bits: core/store.h STORE_VOIDS) written at once, a program */
static void void_done(uint8_t b);
static uint8_t flash_recheck = 1;          /* a flash operation ran since flash_erase_ahead last found nothing due: the
                                              stores' positions (core/pagepair.h) change only in one, so until then its
                                              four checks would answer the same */

/* (r6) in the hold-off after a failed operation (above) */
static uint32_t flash_held(void)
{
    return flash_err && ms_now - flash_err_ms < (FLASH_RETRY_MS << (flash_err - 1u));
}

/* may a store write now? need_erase: its next record needs an erase first (only while low); prog: FM_PROG when a
 * program may run whatever the outputs show (a dip, the console's save). Returns FM_LOW / FM_PROG / FM_BRIGHT
 * (flash_stall_begin takes the brighter state's token), 0 = not now. */
static uint32_t flash_may(uint32_t need_erase, uint32_t prog)
{
#if BOARD_RETURN
    if (RC_BUSY()) {
        return 0;                                        /* the return channel's window first */
    }
#endif
    if (prog != FM_PROG && uart_busy()) {
        return 0;                                        /* (the console first: hw.c flash_op) */
    }
    if (prog != FM_PROG && flash_held()) {
        return 0;                                        /* the hold-off after a failed operation (above) */
    }
    if (hw_vdd_low()) {
        /* (review round 6) below the PVD flash_op refuses at once (FLASH_ERR_VDD): nothing is admitted, so no store
         * burns a write token (0.6.6: a pending settings change tried every ms, 16 tokens in 16 ms, then waited 3 h
         * for the next) and no A/B entry is used up by a write that never started; the change stays pending */
        return 0;
    }
    if (flash_boot || (ms_now - flash_hi_ms >= FLASH_LOW_MS && !vdip.in_dip)) {
        return FM_LOW;
    }
    if (need_erase) {
        return 0;
    }
    if (vdip.in_dip) {
        return FLASH_DIP_WRITES ? FM_PROG : 0u;          /* a dip: programs only (or nothing) */
    }
    if (prog == FM_PROG) {
        return FM_PROG;
    }
    return budget_avail(&flash_bright, ms_now) ? FM_BRIGHT : 0u;
}

/* (0.6.9) the status: why a pending write still waits (cli.h cli_status_t save_why; the main reasons, in order) */
static uint8_t save_why(void)
{
    if (!(settings_dirty | cal_save_pend | scene_pend | fx_clear_pend | (fx_pend == 2u))) {
        return 0;
    }
    if (hw_vdd_low()) {
        return 5;
    }
    if (flash_held()) {
        return 4;
    }
    if ((settings_dirty && ms_now - changed_ms < SAVE_DEBOUNCE_MS) ||
        (scene_pend && ms_now - scene_changed_ms < SCENE_SAVE_MS)) {
        return 1;
    }
    return (ms_now - flash_hi_ms < FLASH_LOW_MS && !budget_avail(&flash_bright, ms_now)) ? 2u : 3u;
}

/* every ms while low (and once at power-up): one erase that is due, so a later write needs none. Not before
 * LIFE_PREP_S after power-up (a steady supply) except at power-up itself (before the outputs start: 0.6.3's
 * lifetime erase there keeps wall-switched boards going). */
static void flash_erase_ahead(void)
{
    uint32_t i;
    if ((!flash_recheck && !fx_clear_pend) ||            /* nothing due since the last operation (see above) */
        (!flash_boot && up_s < LIFE_PREP_S) || flash_may(1u, 0u) != FM_LOW) {
        return;
    }
    if (fx_clear_pend) {
        flash_stall_begin(FM_LOW);                       /* (a factory reset: the power-on effects go) */
        fx_clear_pend = (uint8_t)!flash_stall_end(scene_fx_store(&ul, 0, fx_crc));
        if (!fx_clear_pend) {
            void_done(STORE_VOID_FX);
        }
        return;
    }
    for (i = 0; i < REC_COUNT; i++) {                    /* (flashrec.h REC_*: the lifetime record, the settings, the
                                                            calibration, the scene log) */
        if (flash_rec_pending(i)) {
            flash_stall_begin(FM_LOW);
            (void)flash_stall_end(flash_rec_prepare(i));
            return;
        }
    }
    flash_recheck = 0;                                   /* none due: not asked again until a flash operation */
}

/* the lifetime record's writes (core/energy.h ENERGY_DUE_*): a regular one as any store's, a dip's always (into an
 * erased entry; none while its next page waits for an erase) */
static uint32_t life_admit(void)
{
    uint32_t m = flash_may(0u, 0u);
    if (m == FM_BRIGHT && life_room() < 2u) {
        m = 0;                                           /* brighter: the last erased entry stays for a dip write */
    }
    return life_pending() == 2u ? 0u : (FLASH_DIP_WRITES && !hw_vdd_low() ? ENERGY_DUE_DIP : 0u) |
                                       (m != 0u ? ENERGY_DUE_SAVE : 0u);   /* (r6: no dip write below the PVD either) */
}
