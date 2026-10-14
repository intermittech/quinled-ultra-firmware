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

int settings_sanitize(settings_t *s)
{
    /* the byte fields: offset, lowest and highest value (pwm_preset also takes 0xFF: a raw period) */
    static const uint8_t lim[12][3] = {
        { offsetof(settings_t, protocol), 0, PMODE_ULTRA },
        { offsetof(settings_t, pwm_preset), 0, PWM_PRESET_COUNT - 1u },
        { offsetof(settings_t, dither_mode), 0, 2 },
        { offsetof(settings_t, dither_thresh), 1, 255 },
        { offsetof(settings_t, feedback_ch), 0, 2 },
        { offsetof(settings_t, stagger), 0, STAGGER_MODES - 1u },
        { offsetof(settings_t, dim_floor), 0, 2 },
        { offsetof(settings_t, min_pulse), 5, 100 },
        { offsetof(settings_t, signal_loss), 0, 3 },   /* (0.6.9, B10: 3 = scene) */
        { offsetof(settings_t, smoothing), 0, 2 },
        { offsetof(settings_t, gamma), 0, 2 },
        { offsetof(settings_t, last_proto), 0, PROTO_ULTRA },
    };
    settings_t d;
    uint32_t i;
    int fixed = 0;
    settings_default(&d);
    for (i = 0; i < 12u; i++) {
        uint8_t *f = (uint8_t *)s + lim[i][0];
        if ((*f < lim[i][1] || *f > lim[i][2]) && !(i == 1u && *f == 0xFFu)) {
            *f = ((const uint8_t *)&d)[lim[i][0]];
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

/* the byte settings a command sets: its field, its range and whether the ACK blinks the value (0.6.6, review round 5: a
 * table instead of a case each; MIN_PULSE / DITHER_THRESH: an ACK without the value code) */
static const uint8_t cmd_field[CMD_MIN_PULSE][4] = {   /* [cmd - 1]: offset (0xFF: not a byte setting), min, max, code */
    { offsetof(settings_t, protocol), 0, 3, 1 },
    { offsetof(settings_t, pwm_preset), 0, PWM_PRESET_COUNT - 1u, 1 },
    { offsetof(settings_t, dither_mode), 0, 2, 1 },
    { offsetof(settings_t, dither_thresh), 1, 255, 0 },
    { offsetof(settings_t, feedback_ch), 0, 2, 1 },
    { offsetof(settings_t, stagger), 0, STAGGER_MODES - 1u, 1 },
    { offsetof(settings_t, dim_floor), 0, 2, 1 },
    { offsetof(settings_t, signal_loss), 0, 3, 1 },
    { 0xFF, 0, 0, 0 },                                /* CMD_IDENTIFY */
    { 0xFF, 0, 0, 0 },                                /* CMD_FACTORY_RESET */
    { offsetof(settings_t, smoothing), 0, 2, 1 },
    { offsetof(settings_t, gamma), 0, 2, 1 },
    { offsetof(settings_t, min_pulse), 5, 100, 0 },
};

/* apply one command; returns ACK / SAME / NACK and the blink value code */
cfg_result_t config_apply(config_rx_t *r, settings_t *s, uint8_t cmd, uint8_t arg)
{
    cfg_result_t res = { CFG_NACK, cmd, 0, arg };

    if ((uint32_t)(cmd - 1u) < CMD_MIN_PULSE && cmd_field[cmd - 1u][0] != 0xFFu) {
        const uint8_t *f = cmd_field[cmd - 1u];
        uint8_t *field = (uint8_t *)s + f[0];
        if (arg < f[1] || arg > f[2]) {
            return res;
        }
        res.event = (*field == arg) ? CFG_SAME : CFG_ACK;
        *field = arg;
        res.value_code = f[3] ? (uint8_t)(arg + 1u) : 0u;
        return res;
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
