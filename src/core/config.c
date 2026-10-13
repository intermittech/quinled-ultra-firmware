/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "config.h"
#include "presets.h"
#include "ultra_protocol.h"

uint8_t crc8_smbus(const uint8_t *d, uint32_t n)
{
    return ultra_crc8(d, n);                          /* the same CRC-8/SMBus */
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
    uint8_t *field = 0;
    uint8_t max = 0;

    switch (cmd) {
    case CMD_PROTOCOL:      field = &s->protocol;      max = 3;  break;
    case CMD_PWM_PRESET:    field = &s->pwm_preset;    max = PWM_PRESET_COUNT - 1u; break;
    case CMD_DITHER_MODE:   field = &s->dither_mode;   max = 2;  break;
    case CMD_FEEDBACK_CH:   field = &s->feedback_ch;   max = 2;  break;
    case CMD_STAGGER:       field = &s->stagger;       max = STAGGER_MODES - 1u;  break;
    case CMD_DIM_FLOOR:     field = &s->dim_floor;     max = 2;  break;
    case CMD_SIGNAL_LOSS:   field = &s->signal_loss;   max = 2;  break;
    case CMD_SMOOTHING:     field = &s->smoothing;     max = 2;  break;
    case CMD_GAMMA:         field = &s->gamma;         max = 2;  break;
    case CMD_MIN_PULSE:
        if (arg < 5u || arg > 100u) {
            return res;
        }
        res.event = (s->min_pulse == arg) ? CFG_SAME : CFG_ACK;
        s->min_pulse = arg;
        return res;                                   /* ACK only, no value code */
    case CMD_DITHER_THRESH:
        if (arg == 0u) {
            return res;
        }
        res.event = (s->dither_thresh == arg) ? CFG_SAME : CFG_ACK;
        s->dither_thresh = arg;
        return res;                                   /* ACK only, no value code */
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
    if (arg > max) {
        return res;
    }
    res.event = (*field == arg) ? CFG_SAME : CFG_ACK;
    *field = arg;
    res.value_code = (uint8_t)(arg + 1u);
    return res;
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
