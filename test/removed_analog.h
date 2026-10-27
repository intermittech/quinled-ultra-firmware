/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Retired analog bytes for negative controls and old traffic fixtures only.
 * These identifiers are deliberately absent from the public protocol header. */
#ifndef TEST_REMOVED_ANALOG_H
#define TEST_REMOVED_ANALOG_H
#include "../src/core/ultra_protocol.h"
#define ULTRA_EFFECT 3u
#define ULTRA_STEP 4u
#define ULTRA_SCENE_EFFECT 10u
#define ULTRA_MAX_STEPS 8u
#define ULTRA_FX_PARAMS 22u
enum {
    ULTRA_FX_NONE = 0,         /* stop: back to the light's state */
    ULTRA_FX_PULSE = 1,        /* on_ms (3) off_ms (3) interval_ms (3) min (2) max (2)   - like ESPHome pulse */
    ULTRA_FX_STROBE = 2,       /* steps (1); steps sent as ULTRA_STEP                    - like ESPHome strobe */
    ULTRA_FX_FLICKER = 3,      /* alpha (2) intensity (2) interval_ms (2)                - like ESPHome flicker */
    ULTRA_FX_RANDOM = 4,       /* transition_ms (3) interval_ms (3)                      - like ESPHome random */
    ULTRA_FX_CANDLE = 5,       /* intensity (2) speed (2, % of nominal)                  - flame flicker */
    ULTRA_FX_BREATHE = 6,      /* period_ms (3) min (2) max (2)                          - sine breathing */
    ULTRA_FX_COLOR_LOOP = 7,   /* period_ms (3) saturation (2)                           - hue rotation (RGB) */
    ULTRA_FX_SUNRISE = 8,      /* duration_ms (3)                                         - warm dawn to the state */
    ULTRA_FX_SUNSET = 9,       /* duration_ms (3)                                         - state to warm dusk, off */
    ULTRA_FX_COUNT = 10,       /* 10-127: future official effects; a board runs an unknown id as ULTRA_FX_NONE */
    ULTRA_FX_CUSTOM = 0x80,    /* 128-255: custom effects */
};
static inline uint32_t ultra_fx_param_bytes(uint8_t fx)
{
    static const uint8_t n[ULTRA_FX_COUNT] = { 0, 13, 1, 6, 6, 4, 7, 5, 3, 3 };
    return fx < ULTRA_FX_COUNT ? n[fx] : 0u;
}
static inline void ultra_slot_effect(uint8_t *s, uint8_t light, uint8_t seq, uint8_t fx, int32_t start_ms,
                                     const uint8_t *params, uint32_t nparams)
{
    uint8_t *p = s + ULTRA_PAYLOAD;
    uint32_t i;
    ultra_slot_begin(s, ULTRA_EFFECT, light, 0, seq);
    p[0] = fx < ULTRA_FX_COUNT ? fx : (uint8_t)ULTRA_FX_NONE;
    ultra_put32(p + 1, (uint32_t)start_ms);
    for (i = 0; i < nparams && i < ultra_fx_param_bytes(p[0]); i++) {
        p[5 + i] = params[i];
    }
    ultra_slot_seal(s);
}
static inline void ultra_slot_step(uint8_t *s, uint8_t light, uint8_t seq, uint8_t index, uint8_t count,
                                   uint16_t dur_ms, uint16_t trans_ms, uint16_t m, const uint16_t *a,
                                   const uint16_t *b)
{
    uint8_t *p = s + ULTRA_PAYLOAD;
    uint32_t i;
    ultra_slot_begin(s, ULTRA_STEP, light, 0, seq);
    p[0] = (uint8_t)((index << 4) | (count & 0x0Fu));
    ultra_put16(p + 1, dur_ms);
    ultra_put16(p + 3, trans_ms);
    ultra_put16(p + 5, m);
    for (i = 0; i < 5u; i++) {
        ultra_put16(p + 7 + 2 * i, a[i]);
        ultra_put16(p + 17 + 2 * i, b[i]);
    }
    ultra_slot_seal(s);
}
static inline void ultra_slot_scene_effect(uint8_t *s, uint8_t light, uint8_t seq, uint8_t fx, const uint8_t *params,
                                           uint32_t nparams)
{
    uint8_t *p = s + ULTRA_PAYLOAD;
    uint32_t i;
    ultra_slot_begin(s, ULTRA_SCENE_EFFECT, light, 0, seq);
    p[0] = fx < ULTRA_FX_COUNT ? fx : (uint8_t)ULTRA_FX_NONE;
    for (i = 0; i < nparams && i < ultra_fx_param_bytes(p[0]); i++) {
        p[1 + i] = params[i];
    }
    ultra_slot_seal(s);
}
#endif
