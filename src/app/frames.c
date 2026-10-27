/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* frames.c: a frame's own slot: handle_slot (WS config / WS frames), handle_ultra (Ultra), a protocol change, the
 * gate's close.
 * A unit of src/app/main.c: main.c includes the units in order as one translation
 * unit, so their statics stay static and the image is the single file's; compiled only through main.c. */

static void handle_slot(const slot_t *slot, proto_t p)
{
    cfg_result_t r = config_process(&rx, &set, slot);
    uint16_t tgt[5];
    uint8_t i, ramp;
    uint32_t d;

    last_data_ms = ms_now;
    signal_lost = 0;
    if (r.event != CFG_NONE) {
        handle_result(r);                  /* config frame: outputs hold */
        return;
    }
    if (p == PROTO_WS2805) {
        for (i = 0; i < 5u; i++) {
            tgt[i] = gamma_apply(set.gamma, slot->raw[i]);
        }
        ramp = (uint8_t)(set.smoothing != 0u);
    } else {
        for (i = 0; i < 5u; i++) {
            tgt[i] = slot->ch[i];          /* WS2915 gain header ignored (wiki: WS modes) */
        }
        ramp = (uint8_t)(set.smoothing == 2u);
    }
    for (i = 0, d = 0; i < 5u; i++) {
        d |= (uint32_t)(tgt[i] ^ sm.to[i]);
    }
    if (d != 0u) {
        isense_rearm(&is);                 /* a new frame value: retry if the retries were used up */
    }
    smooth_frame(&sm, tgt, tick_now, ramp);
    scene_takeover(&srun, 0x1Fu, ms_now, SCENE_IN_MS, 1); /* WS has no explicit SET duration. */
    if (ack.repeat) {
        ack_stop(&ack);                    /* identify ends with the next normal frame */
    }
    outputs_dirty = 1;
}

/* Ultra: the header at the end of a frame (host time at the frame end, local tick of its last edge) and this
 * board's slot from the start of the same frame (NULL: none, e.g. a board past the last slot) */
static void handle_ultra(uint32_t host_us, uint32_t hdr_tick, const uint8_t *slot)
{
    uint32_t ev;
    ultra_on_header(&ul, hdr_tick, host_us);
    last_data_ms = ms_now;
    signal_lost = 0;
    ul_used = 1;                           /* a host drives the engine now */
    if (slot == 0) {
        return;
    }
    if (!ultra_seen) {
        /* the power-on scene holds at most SCENE_TAKEOVER_MS from this board's first slot (not from the first
         * header: that would run out inside the hub's DARK preamble on long pins: 202 frames, ~3.5 s at 50 boards) */
        ultra_seen = 1;
        ultra_first_ms = ms_now;
    }
    ev = ultra_apply_slot(&ul, slot, hdr_tick);       /* frx_capture checked its CRC */
    if (ev & ULTRA_EV_STATE) {
        isense_rearm(&is);                 /* a new command: retry if the retries were used up */
    }
    if (ev & (ULTRA_EV_STATE | ULTRA_EV_DEF)) {
        /* Each light takes over once both its SET and definition are present.
         * The first SET owns the transition duration, curve and start time. */
        uint32_t l = (uint32_t)slot[1] % ULTRA_LIGHTS;
        if ((slot[0] & ULTRA_TYPE_MASK) == ULTRA_SET) {
            ul_stated |= (uint8_t)(1u << l);
        }
        if ((ul_stated & (1u << l)) && ul.light[l].defined && ul.light[l].mask != 0u) {
            ultra_light_t *L = &ul.light[l];
            uint8_t mask = (uint8_t)(srun.hold & ultra_channel_mask(L));
            if (mask) {
                uint32_t dur = L->cur.dur;
                int32_t start = (int32_t)(L->cur.t0 - ul.frame_ms);
                if (start < 0) {
                    uint32_t elapsed = 0u - (uint32_t)start;
                    dur = elapsed < dur ? dur - elapsed : 0u;
                    start = 0;
                }
                /* Physical scene -> first target, with the SET duration/curve. */
                scene_takeover(&srun, mask, ms_now + (uint32_t)start, dur,
                               (uint8_t)(L->cur.smooth | (L->cur.via_off << 1)));
                L->cur.from = L->cur.to; L->cur.dur = 0; L->prev = L->cur;
                L->settled = 0;
            }
        }
        outputs_dirty = 1;
    }
    if (ev & (ULTRA_EV_SCENE | ULTRA_EV_DEF | ULTRA_EV_CONFIG)) {
        scene_check = 1;                   /* the scene's outputs may change: recompute when quiet */
        scene_check_ms = ms_now;
    }
    if (ev & ULTRA_EV_CONFIG) {
        if (ul.cfg_changed & (1u << ULTRA_CFG_BOOT_PROTOCOL)) {
            uint16_t v = ul.cfg[ULTRA_CFG_BOOT_PROTOCOL];
            if (v <= PMODE_ULTRA && v != set.protocol) {
                set.protocol = (uint8_t)v;     /* persisted: saved after the debounce, only when it differs */
                settings_changed();
                if (v == PMODE_AUTO || v == PMODE_ULTRA) {
                    proto_set_mode(&frx.ps, (proto_mode_t)v);   /* Ultra keeps running; WS modes from the next power-up */
                }
            }
        }
        if ((ul.cfg_changed & (1u << ULTRA_CFG_SOFT_START)) && ul.cfg[ULTRA_CFG_SOFT_START] <= 1u) {
            (void)soft_start_set(ul.cfg[ULTRA_CFG_SOFT_START]);   /* persisted, like 22 */
        }
        if (ul.cfg_changed & (1u << ULTRA_CFG_RETRY)) {
            (void)retry_set(ul.cfg[ULTRA_CFG_RETRY] & 0xFFu, ul.cfg[ULTRA_CFG_RETRY] >> 8);   /* persisted */
        }
        ul.cfg_changed = 0;
        apply_settings();                  /* host-managed: live at once, no blink, not saved (except 22, 23) */
        stat.config_frames++;
    }
    if (ev & ULTRA_EV_IDENTIFY) {
        if (ul.identify == ULTRA_IDENT_CAL) {
            (void)isense_calibrate(ul.identify_arg);   /* refused while tripped or in test mode */
        } else if (ul.identify == ULTRA_IDENT_DARK) {
            dark_hold(&dk, ms_now, (ul.identify_arg ? ul.identify_arg : 60u) * 1000u);
        } else if (ul.identify == ULTRA_IDENT_RELEASE) {
#if BOARD == BOARD_UR_ANALOG_M
            if (!vdip.planned)                 /* not during the power-off notice: dark to the cut */
#endif
            {
                dark_release(&dk, ms_now);
            }
        } else if (ul.identify == ULTRA_IDENT_VERSION) {
            ack_start_version(&ack, fw_version);
#if BOARD == BOARD_UR_ANALOG_M
        } else if (ul.identify == ULTRA_IDENT_POWER_OFF) {   /* (ultra.c drops it on the Dig2Analog-Plus-5ch) */
            vdip.planned = 1;
            plan_ms = ms_now;
            if (!dk.active || (int32_t)(dk.until - ms_now) < (int32_t)PLAN_HOLD_MS) {
                dark_hold(&dk, ms_now, PLAN_HOLD_MS);   /* dark (0.3 s) before the cut; a longer
                                                           hold (IDENTIFY 3) stays as it is */
            }
#endif
        } else if (ul.identify == ULTRA_IDENT_ON) {      /* (ultra_apply_slot drops unknown values) */
            ack_start(&ack, ACK_P_IDENTIFY, 0);
        } else {
            ack_stop(&ack);
        }
        outputs_dirty = 1;
    }
}

static void protocol_changed(proto_t from, proto_t to)
{
    if (to == PROTO_ULTRA) {
        if (ul_used) {
            ultra_init(&ul, tick_now ^ 0xA5C3u);   /* (not before a host used it: it may run the power-on effect) */
        }
        ultra_on = 1;
    } else if (from == PROTO_ULTRA) {
        ultra_on = 0;                      /* the host's CONFIG ends with Ultra: stored settings again */
    } else {
        return;
    }
    apply_settings();
}

/* A stored (provisional) protocol turned out wrong: its gate plan may cut the frame at the wrong bits, so
 * forward nothing more until the next re-arm (the new plan). The gate ISR must not re-open it meanwhile. */
static void gate_close_now(void)
{
    __disable_irq();
    gate_quiet = 0;
    gate_force_closed();
    __enable_irq();
}
