/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* update.c: firmware update, the app side (core/upd_app.h): the session's actions, the hand-over, the bootloader's
 * records at boot, the trial's confirm.
 * A unit of src/app/main.c (round 12, S10): main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

/* ------------------------------------------------------------------------------- firmware update (app side) */
/* (0.6.9, Quindor) an update session's SAME / NACK on D9 only, whatever the feedback setting: a host repeats a session
 * (ESPHome's auto_retry) and a board that has it already, or can't take it, keeps its light. A real update's ACK (the
 * new version, upd_boot) keeps the strip pattern; the config / console SAME / NACK are unchanged. */
static void upd_ack_d9(ack_kind_t kind)
{
    ack_start(&ack, kind, 0);
    ack.led_only = 1;
}

static void upd_action(uint32_t a)
{
    switch (a) {
    case UPD_APP_TAKE:
#if UPD_APP
    {
        uint32_t t0 = ms_now;
        HO->request = UPD_REQ_ENTER;       /* the bootloader checks the ANNOUNCE again and installs it */
        upd_ho_seal(HO);
        cli_say(&cli, Z(m_update, "firmware update: into the bootloader\r\n"));
        while (ms_now - t0 < 10u) {        /* the host's lead-in is >= 500 ms: time to send the line */
            uart_kick();
            watchdog_kick();
        }
        gate_close_now();
        NVIC_SystemReset();
    }
#endif
        break;
    case UPD_APP_CONFIRM:
#if UPD_APP
    {
        /* one half-word, as any regular write (0.6.4: the program's status decides, not a read back, which may come
         * from the copy made at reset; not admitted now: at the next PING) */
        uint32_t a = UPD_APP_DESC + 2u * (UPD_ST_FIRST + upd.info.st_next), ok = 0, may = flash_may(0u, 0u);
        if (may != 0u) {
            flash_stall_begin(may);
            ok = flash_stall_end(hw_flash_half(a, UPD_ST_CONFIRMED));
        }
        upd_app_confirmed(&upd, (int)ok);
    }
#endif
        break;
    case UPD_APP_SHOW_SAME:
        upd_ack_d9(ACK_P_SAME);            /* the session's image is already installed */
        break;
    case UPD_APP_SHOW_REJECTED:
        upd_ack_d9(ACK_P_NACK);            /* not for this board, or older without FORCE */
        break;
    default:
        break;
    }
}

/* Decoded bytes live only through frx_capture / upd_app_frame. Both consume or copy
 * them before handle_slot / upd_action can build tables or program a flash record.
 * No ISR uses core_scratch. Keep the decoder's 16-byte memset alignment. */
typedef char decode_fits_scratch[(PROTO_BITS_BUF <= sizeof(core_scratch) &&
                                 UPD_FRAME_BYTES + 4u <= sizeof(core_scratch)) ? 1 : -1];

static void upd_capture(void)
{
    uint8_t *f = (uint8_t *)core_scratch;
    if (cap_bits < UPD_FRAME_BITS ||
        ws_decode_frame(din_copy, (uint32_t)cap_words * 16u, (uint16_t)UPD_FRAME_BITS, &ws_tm, f) != (int)UPD_FRAME_BITS) {
        return;                            /* the session repeats every block (passes) */
    }
    upd_action(upd_app_frame(&upd, f, ms_now));
}

#include "../core/bl_protection.h"
typedef char protection_events_match[(BL_PROT_OK - BL_PROT_BAD == EVL_PROT_OK - EVL_OB_BAD &&
                                       BL_PROT_FAIL - BL_PROT_BAD == EVL_PROT_FAIL - EVL_OB_BAD) ? 1 : -1];

/* boot: what the bootloader left in the hand-over block (UPD_APP), the app's descriptor and status records */
static void upd_boot(void)
{
    upd_app_info_t info;
#if UPD_APP
    upd_app_read_info(&info, (const uint8_t *)UPD_APP_DESC, (const uint8_t *)UPD_BL_INFO);
    if (upd_ho_valid(HO)) {
        uint32_t protection = HO->reserved - BL_PROT_BAD;
        stat.reset_flags = HO->reset_flags;   /* the bootloader cleared the RCC flags (its own resets pile up) */
        HO->reset_flags = 0;
        switch (HO->result) {
        case UPD_RES_UPDATED: {
            uint32_t minor = (HO->result_version >> 16) & 0xFFu;
            ack_start(&ack, ACK_P_ACK, (uint8_t)((minor > 29u ? 29u : minor) + 1u));   /* ACK + the minor version */
            break;
        }
        case UPD_RES_SAME:                 /* (the bootloader's own check, after a TAKE: D9 only as above) */
            upd_ack_d9(ACK_P_SAME);
            break;
        case UPD_RES_REJECTED:
            upd_ack_d9(ACK_P_NACK);
            break;
        default:
            break;
        }
        HO->result = UPD_RES_NONE;
        HO->request = UPD_REQ_NONE;
        /* The bootloader owns option writes/read-back. Keep the app's event/jig reports, with a protection
         * fault taking priority over an update ACK as before. Older bootloaders leave reserved zero. */
        if (protection <= BL_PROT_FAIL - BL_PROT_BAD) {
            ev_add((uint8_t)(EVL_OB_BAD + protection), 0);
            if (protection != BL_PROT_OK - BL_PROT_BAD) {
                ack_start(&ack, ACK_P_FAULT, HEALTH_CODE_PROTECT);
            }
        }
        HO->reserved = 0;
        upd_ho_seal(HO);
    }
#else
    info.hw_id = info.hw_rev = info.bl_version = 0;   /* standalone: forward update sessions only */
    info.desc_ok = info.confirmed = 0;
    info.st_next = (uint8_t)UPD_ST_COUNT;
    info.version = 0;
#endif
    upd_app_init(&upd, &info);
}

/* every ms: transparency timeout / pending results / a delayed confirm; the watchdog-loop count */
static void upd_service(void)
{
    uint8_t was_transparent = (uint8_t)upd_app_transparent(&upd);
    uint32_t a;
    if (up_s >= UPD_APP_AUTO_S && stat.decoded >= UPD_APP_AUTO_FRAMES && !(stat.reset_flags & RCC_IWDGRSTF)) {
        upd.auto_ok = 1;                   /* (round 11, B4) no PING (WLED, the sender library): a trial image that
                                              runs a minute with frames and no watchdog reset confirms itself */
    }
    a = upd_app_poll(&upd, ms_now);
    if (a != 0u) {
        upd_action(a);                     /* (0 = nothing: upd_action's default; the call every ms saved) */
    }
    if (was_transparent && !upd_app_transparent(&upd)) {
        gate_plan_t g = proto_gate_plan(proto_active(&frx.ps));
        uint8_t i = (uint8_t)(plan_idx ^ 1u);
        plan_buf[i] = g;
        BARRIER();
        plan_idx = i;
        /* With no more frames there is no frame-end ISR to install the normal plan.
         * Re-arm only on an armed idle input; an active/partial frame finishes normally. */
        __disable_irq();
        if (ft.armed && ft.cap_state == 0u && DIN_EDGES() == 0u) {
            gate_rearm(&g);
            plan = g;
        }
        __enable_irq();
    }
#if UPD_APP
    if (!upd_wd_cleared && up_s >= UPD_WD_CLEAR_S && upd_ho_valid(HO)) {
        HO->wd_resets = 0;                 /* running fine: the bootloader starts counting again */
        upd_ho_seal(HO);
        upd_wd_cleared = 1;
    }
#endif
}
