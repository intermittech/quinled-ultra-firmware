/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* flash_policy.c: the flash: when: flash_may and its policy, why a write waits (save_why), the erase ahead, the
 * lifetime record's admission.
 * A unit of src/app/main.c: main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

/* Flash program and erase admission: outputs exactly zero for 500 ms, no
 * calibration/test/soft-start/rising fade, no active return window, adequate VDD.
 * Boot admission precedes enabling the outputs. Dips and explicit saves never
 * bypass darkness. FM_PROG can bypass console/retry delay only while dark. */
#define FLASH_LOW_LEVEL 1u
#define FLASH_LOW_MS 500u
#define FM_LOW 1u
#define FM_PROG 2u
#ifndef FLASH_DIP_WRITES
#define FLASH_DIP_WRITES 1
#endif
#define FLASH_RETRY_MS   10000u
#define FLASH_RETRY_MAX  6u
static uint32_t flash_hi_ms;               /* the last ms the outputs were not low */
static uint32_t flash_err_ms;              /* the last failed operation; flash_err: failures in a row (0: none) */
static uint8_t flash_err;
static uint8_t flash_boot = 1;             /* before the main loop: the outputs are dark */
static uint8_t reset_save;                 /* (F10) a factory reset: its settings record (the defaults, the void
                                              bits: core/store.h STORE_VOIDS) written at once, a program */
static void void_done(uint8_t b);
static uint8_t flash_recheck = 1;          /* a flash operation ran since flash_erase_ahead last found nothing due: the
                                              stores' positions (core/pagepair.h) change only in one, so until then its
                                              four checks would answer the same */

/* in the hold-off after a failed operation (above) */
static uint32_t flash_held(void)
{
    return flash_err && ms_now - flash_err_ms < (FLASH_RETRY_MS << (flash_err - 1u));
}

/* 0 means deferred. FM_LOW allows erase/program; FM_PROG allows only program.
 * All non-boot operations require confirmed dark; a dip can never erase. */
static uint32_t flash_may(uint32_t need_erase, uint32_t prog)
{
    uint32_t i;
    /* The once-per-ms dark-age sample can precede an output change. Check the
     * published levels again at admission so that interval cannot admit a write. */
    if (!flash_boot) {
        BARRIER();
        if (srun.in || srun.fading) return 0;
        for (i=0;i<BOARD_CHANNELS;++i) if (out_now[i]) return 0;
    }
#if BOARD_RETURN
    if (RC_BUSY()) {
        return 0;                                        /* the return channel's window first */
    }
#endif
    if (vdip.in_dip && !need_erase && FLASH_DIP_WRITES) prog = FM_PROG;
    if (prog != FM_PROG && uart_busy()) {
        return 0;                                        /* (the console first: hw.c flash_op) */
    }
    if (prog != FM_PROG && flash_held()) {
        return 0;                                        /* the hold-off after a failed operation (above) */
    }
    if (hw_vdd_low()) {
        /* below the PVD flash_op refuses at once (FLASH_ERR_VDD): nothing is admitted, so no store
         * burns a write token (a pending settings change retried every ms would burn 16 tokens in 16 ms, then wait
         * 3 h for the next) and no A/B entry is used up by a write that never started; the change stays pending */
        return 0;
    }
    if (flash_boot || ms_now - flash_hi_ms >= FLASH_LOW_MS) {
        if (vdip.in_dip) return !need_erase && FLASH_DIP_WRITES ? FM_PROG : 0u;
        return FM_LOW;
    }
    return 0; /* Every program and erase waits for confirmed dark, including dips/manual saves. */
}

/* the status: why a pending write still waits (cli.h cli_status_t save_why; the main reasons, in order) */
static uint8_t save_why(void)
{
    if (!(settings_dirty | cal_save_pend | scene_pend)) {
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
    return ms_now - flash_hi_ms < FLASH_LOW_MS ? ULTRA_DG_SW_DARK : ULTRA_DG_SW_BUDGET;
}

/* every ms while low (and once at power-up): one erase that is due, so a later write needs none. Not before
 * LIFE_PREP_S after power-up (a steady supply) except at power-up itself (before the outputs start: the
 * lifetime erase there keeps wall-switched boards going). */
static void flash_erase_ahead(void)
{
    uint32_t i;
    if (!flash_recheck ||            /* nothing due since the last operation (see above) */
        (!flash_boot && up_s < LIFE_PREP_S) || flash_may(1u, 0u) != FM_LOW) {
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
    return life_pending() == 2u || !m ? 0u : ENERGY_DUE_SAVE | ENERGY_DUE_DIP;
}
