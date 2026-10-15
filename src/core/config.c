/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include <stddef.h>
#include "config.h"
#include "presets.h"
#include "ultra_protocol.h"

uint8_t crc8_smbus(const uint8_t *d, uint32_t n)
{
    return ultra_crc8(d, n);                          /* the same CRC-8/SMBus */
}

uint16_t crc16_ccitt(const uint8_t *d, uint32_t n)
{
    return ultra_crc16(d, n);
}

#if UPPER_RECORDS
int crc16_erased(uint32_t c)
{
    return c == 0xFFFFu || c == 0xE339u;
}
#endif

void settings_default(settings_t *s)
{
    s->protocol = PMODE_AUTO;
    s->pwm_preset = PWM_PRESET_DEFAULT;
    s->raw_steps = 5760;
    s->dither_mode = 0;
    s->dither_thresh = 10;      /* 1.0 % */
    s->feedback_ch = 0;
    s->stagger = STAGGER_ADAPTIVE_SPREAD;   /* least LED_+ ripple + spread spectrum (fixed thirds without PB11) */
    s->dim_floor = 0;           /* skip pulses below the minimum pulse */
    s->min_pulse = 20;          /* 200 ns: safe start until measured on the scope */
    s->signal_loss = 0;         /* hold the last colour, like a WS2805 / WS2915 */
    s->smoothing = 1;           /* WS2805 input only: 8-bit fades become smooth 16-bit ramps */
    s->gamma = 0;               /* linear: WLED and most controllers already apply gamma */
    s->last_proto = PROTO_NONE; /* nothing locked yet: the first frames decide */
}

/* the byte settings (config.h setdesc_t), in the console's order: the field, its range, the config-frame command (the
 * ACK blinks the value, except DITHER_THRESH / MIN_PULSE), the Ultra CONFIG id (entries 2..8). The preset / period pair
 * (CMD_PERIOD_*, ULTRA_CFG_PWM_PRESET / PWM_PERIOD) is code: config_apply, main.c ultra_overlay. (round 9: one table
 * instead of settings_sanitize's, config_apply's, ultra_overlay's and the console's two) */
const setdesc_t settings_desc[SETD_N] = {
    { offsetof(settings_t, protocol),      0, PMODE_ULTRA,           CMD_PROTOCOL,                     0 },
    { offsetof(settings_t, pwm_preset),    0, PWM_PRESET_COUNT - 1u, CMD_PWM_PRESET,                   0 },
    { offsetof(settings_t, dither_mode),   0, 2,                     CMD_DITHER_MODE,                  ULTRA_CFG_DITHER },
    { offsetof(settings_t, dither_thresh), 1, 255,                   CMD_DITHER_THRESH | SETD_NO_CODE, ULTRA_CFG_DITHER_THRESH },
    { offsetof(settings_t, dim_floor),     0, 2,                     CMD_DIM_FLOOR,                    ULTRA_CFG_DIM_FLOOR },
    { offsetof(settings_t, min_pulse),     5, 100,                   CMD_MIN_PULSE | SETD_NO_CODE,     ULTRA_CFG_MIN_PULSE },
    { offsetof(settings_t, stagger),       0, STAGGER_MODES - 1u,    CMD_STAGGER,                      ULTRA_CFG_STAGGER },
    { offsetof(settings_t, signal_loss),   0, 3,                     CMD_SIGNAL_LOSS,                  ULTRA_CFG_LOSS },   /* (0.6.9, B10: 3 = scene) */
    { offsetof(settings_t, feedback_ch),   0, 2,                     CMD_FEEDBACK_CH,                  ULTRA_CFG_FEEDBACK },
    { offsetof(settings_t, smoothing),     0, 2,                     CMD_SMOOTHING,                    0 },
    { offsetof(settings_t, gamma),         0, 2,                     CMD_GAMMA,                        0 },
    { offsetof(settings_t, last_proto),    0, PROTO_ULTRA,           0,                                0 },
};

int settings_sanitize(settings_t *s)
{
    settings_t d;
    uint32_t i;
    int fixed = 0;
    settings_default(&d);
    for (i = 0; i < SETD_N; i++) {                    /* (entry 1, pwm_preset, also takes 0xFF: a raw period) */
        const setdesc_t *t = &settings_desc[i];
        uint8_t *f = (uint8_t *)s + t->ofs;
        if ((*f < t->lo || *f > t->hi) && !(i == 1u && *f == 0xFFu)) {
            *f = ((const uint8_t *)&d)[t->ofs];
            fixed = 1;
        }
    }
    if (s->raw_steps < PWM_STEPS_MIN || s->raw_steps > PWM_STEPS_MAX) {
        s->raw_steps = d.raw_steps;
        fixed = 1;
    }
    return fixed;
}

uint32_t settings_steps(const settings_t *s)
{
    return s->pwm_preset < PWM_PRESET_COUNT ? pwm_preset_steps[s->pwm_preset] : s->raw_steps;
}

void config_rx_init(config_rx_t *r)
{
    uint8_t i;
    for (i = 0; i < 5u; i++) {
        r->last[i] = 0;
    }
    r->count = 0;
    r->done = 0;
    r->period_hi = 0;
    r->period_hi_valid = 0;
}

void config_build(uint8_t cmd, uint8_t arg, uint8_t ws2915, uint8_t *out)
{
    uint8_t i;
    out[0] = (uint8_t)((CONFIG_MAGIC >> 8) & 0xFFu);
    out[1] = (uint8_t)(CONFIG_MAGIC & 0xFFu);
    out[2] = cmd;
    out[3] = arg;
    out[4] = crc8_smbus(out, 4);
    if (ws2915) {
        for (i = 0; i < 5u; i++) {
            out[5u + i] = (uint8_t)~out[i];
        }
    }
}

static int is_config(const slot_t *slot)
{
    uint8_t i;
    if (slot->raw[0] != (uint8_t)((CONFIG_MAGIC >> 8) & 0xFFu) || slot->raw[1] != (uint8_t)(CONFIG_MAGIC & 0xFFu)) {
        return 0;
    }
    if (crc8_smbus(slot->raw, 4) != slot->raw[4]) {
        return 0;
    }
    if (slot->raw_len == 10u) {
        for (i = 0; i < 5u; i++) {
            uint8_t inv = (uint8_t)~slot->raw[i];
            if (inv != slot->raw[5u + i]) {
                return 0;
            }
        }
    }
    return 1;
}

/* apply one command; returns ACK / SAME / NACK and the blink value code */
cfg_result_t config_apply(config_rx_t *r, settings_t *s, uint8_t cmd, uint8_t arg)
{
    cfg_result_t res = { CFG_NACK, cmd, 0, arg };
    uint32_t i;

    /* a byte setting (settings_desc: the field, its range, whether the ACK blinks the value; 0.6.6, review round 5: a
     * table instead of a case each) */
    for (i = 0; i < SETD_SHOWN; i++) {
        const setdesc_t *t = &settings_desc[i];
        if ((t->cmd & ~SETD_NO_CODE) == cmd) {
            uint8_t *field = (uint8_t *)s + t->ofs;
            if (arg < t->lo || arg > t->hi) {
                return res;
            }
            res.event = (*field == arg) ? CFG_SAME : CFG_ACK;
            *field = arg;
            res.value_code = (t->cmd & SETD_NO_CODE) ? 0u : (uint8_t)(arg + 1u);
            return res;
        }
    }
    switch (cmd) {
    case CMD_IDENTIFY:
        res.event = CFG_IDENTIFY;
        return res;
    case CMD_SCENE:                                   /* the scene is not a setting: main.c stores it */
        if (arg > 1u) {
            return res;
        }
        res.event = CFG_ACK;
        res.value_code = (uint8_t)(arg + 1u);
        return res;
    case CMD_SCENE_FADE:
        res.event = CFG_ACK;                          /* ACK only, no value code */
        return res;
    case CMD_ISENSE_LIMIT:                            /* main.c stores it (settings extension) */
        res.event = CFG_ACK;                          /* ACK only, no value code */
        return res;
    case CMD_RETRY_COUNT:                             /* (0.6.9, B7) main.c stores them (settings extension) */
    case CMD_RETRY_S:
        res.event = CFG_ACK;                          /* ACK only, no value code */
        return res;
    case CMD_SOFT_START:                              /* main.c stores it (settings extension) */
        if (arg > 1u) {
            return res;
        }
        res.event = CFG_ACK;
        res.value_code = (uint8_t)(arg + 1u);
        return res;
    case CMD_ISENSE_CAL:                              /* main.c runs it: the calibration pattern is the feedback */
        if (arg != 0u && (arg < 5u || arg > 100u)) {
            return res;
        }
        res.event = CFG_ACK;
        return res;
    case CMD_FACTORY_RESET:
        if (arg != 0xA5u) {
            return res;
        }
        settings_default(s);
        res.event = CFG_ACK;
        res.value_code = 0xFF;                        /* special: ACK twice */
        return res;
    case CMD_PERIOD_HI:
        r->period_hi = arg;
        r->period_hi_valid = 1;
        res.event = CFG_PENDING;                      /* applied with the low byte */
        return res;
    case CMD_PERIOD_LO: {
        uint32_t steps;
        if (!r->period_hi_valid) {
            return res;
        }
        steps = ((uint32_t)r->period_hi << 8) | arg;
        r->period_hi_valid = 0;
        if (steps < PWM_STEPS_MIN || steps > PWM_STEPS_MAX) {
            return res;
        }
        res.event = (s->pwm_preset == 0xFFu && s->raw_steps == steps) ? CFG_SAME : CFG_ACK;
        s->pwm_preset = 0xFF;
        s->raw_steps = steps;
        return res;
    }
    default:
        return res;
    }
}

cfg_result_t config_process(config_rx_t *r, settings_t *s, const slot_t *slot)
{
    cfg_result_t none = { CFG_NONE, 0, 0, 0 };
    cfg_result_t wait = { CFG_PENDING, 0, 0, 0 };
    uint8_t i, same = 1;

    if (!is_config(slot)) {
        r->count = 0;
        r->done = 0;                                  /* a normal frame re-enables the same command */
        return none;
    }
    for (i = 0; i < 5u; i++) {
        if (r->last[i] != slot->raw[i]) {
            same = 0;
        }
        r->last[i] = slot->raw[i];
    }
    if (!same) {
        r->count = 1;
        r->done = 0;
    } else if (r->count < 255u) {
        r->count++;
    }
    if (r->done || r->count < CONFIG_REPEAT) {
        return wait;                                  /* outputs hold while config frames arrive */
    }
    r->done = 1;
    return config_apply(r, s, slot->raw[2], slot->raw[3]);
}
