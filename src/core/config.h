/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
/* Config frames and settings (DESIGN_NOTES "Config frames").
 *
 * Message (40 bits): MAGIC (16) | CMD (8) | ARG (8) | CRC8 (8), CRC-8/SMBus (poly 0x07) over the first 4 bytes.
 *   WS2805 mode: the five channel bytes R, G, B, W1, W2 carry it.
 *   WS2915 mode: first 5 bytes of the 10-byte slot; the other 5 bytes are the bitwise complement.
 * Accepted after CONFIG_REPEAT identical frames in a row. The same command only triggers again after at
 * least one normal (non-config) frame. The UART console applies commands through config_apply() too. */
#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include "proto.h"

#define CONFIG_MAGIC   0xD2A5u
#define CONFIG_REPEAT  3u

enum {
    CMD_PROTOCOL      = 0x01,   /* 0 auto, 1 WS2915, 2 WS2805, 3 Ultra */
    CMD_PWM_PRESET    = 0x02,   /* 0..14 */
    CMD_DITHER_MODE   = 0x03,   /* 0 on, 1 off at low levels, 2 off */
    CMD_DITHER_THRESH = 0x04,   /* 1..255 = 0.1..25.5 % */
    CMD_FEEDBACK_CH   = 0x05,   /* 0 red, 1 all, 2 off */
    CMD_STAGGER       = 0x06,   /* STAGGER_*: 0 off, 1 fixed, 2 fixed + spread, 3 adaptive, 4 adaptive + spread */
    CMD_DIM_FLOOR     = 0x07,   /* 0 skip pulses below the minimum pulse, 1 clamp, 2 off */
    CMD_SIGNAL_LOSS   = 0x08,   /* 0 hold (WS28xx behaviour), 1 off after 2 s, 2 fade off after 2 s */
    CMD_IDENTIFY      = 0x09,   /* ARG 1 = blink the firmware version (CFG_IDENTIFY_VERSION), else the identify blink */
    CMD_FACTORY_RESET = 0x0A,   /* ARG must be 0xA5 */
    CMD_SMOOTHING     = 0x0B,   /* 0 off, 1 WS2805 input only, 2 all input */
    CMD_GAMMA         = 0x0C,   /* WS2805 input: 0 linear, 1 gamma 2.2, 2 gamma 2.8 */
    CMD_MIN_PULSE     = 0x0D,   /* minimum pulse / off-time, 10 ns units, 5..100 (50 ns..1 us) */
    CMD_SCENE         = 0x0E,   /* power-on scene: 0 none (dark), 1 store the outputs shown now (main.c acts on it) */
    CMD_SCENE_FADE    = 0x0F,   /* power-on scene take-over fade, x 100 ms (0..255) */
    CMD_PERIOD_HI     = 0x10,   /* raw period (steps) high byte, applied by CMD_PERIOD_LO */
    CMD_PERIOD_LO     = 0x11,
    /* boards with the input current sense (HW_ISENSE, core/isense.h); NACK on boards without it */
    CMD_ISENSE_LIMIT  = 0x12,   /* board current limit x 50 mA, 0 = the board rating (main.c stores it) */
    CMD_ISENSE_CAL    = 0x13,   /* current calibration: ARG 0 = each channel at 25 / 50 / 75 / 100 %, 5..100 = at ARG % */
};

/* CMD_IDENTIFY's ARG for the version blink (main.c, ack.h). Older firmware shows the identify blink for any ARG. */
#define CFG_IDENTIFY_VERSION 1u

/* stagger modes (CMD_STAGGER / Ultra CONFIG 7), each one followed by its spread-spectrum variant */
enum {
    STAGGER_OFF = 0,            /* all channels turn on together */
    STAGGER_FIXED = 1,          /* timers a third of a period apart */
    STAGGER_FIXED_SPREAD = 2,   /* fixed + spread-spectrum PWM (every period +-6 %) */
    STAGGER_ADAPTIVE = 3,       /* offsets follow the colour mix for the least LED_+ ripple */
    STAGGER_ADAPTIVE_SPREAD = 4,/* adaptive + spread spectrum (default) */
    STAGGER_MODES = 5,
};
static inline int stagger_is_spread(uint8_t m) { return m == STAGGER_FIXED_SPREAD || m == STAGGER_ADAPTIVE_SPREAD; }
static inline int stagger_is_adaptive(uint8_t m) { return m == STAGGER_ADAPTIVE || m == STAGGER_ADAPTIVE_SPREAD; }

typedef struct {
    uint8_t  protocol;          /* proto_mode_t */
    uint8_t  pwm_preset;        /* 0..14, 0xFF = raw period */
    uint8_t  dither_mode;
    uint8_t  dither_thresh;     /* 0.1 % units */
    uint8_t  feedback_ch;
    uint8_t  stagger;
    uint8_t  dim_floor;
    uint8_t  min_pulse;         /* 10 ns units */
    uint8_t  signal_loss;
    uint8_t  smoothing;
    uint8_t  gamma;
    uint8_t  last_proto;        /* auto mode: the protocol locked last (proto_t), used from the next power-up on */
    uint32_t raw_steps;         /* used when pwm_preset == 0xFF: 1440..65535 */
} settings_t;

typedef enum { CFG_NONE = 0, CFG_PENDING, CFG_ACK, CFG_SAME, CFG_NACK, CFG_IDENTIFY } cfg_event_t;

typedef struct {
    uint8_t  last[5];           /* last config message seen */
    uint8_t  count;             /* identical frames in a row */
    uint8_t  done;              /* this run of frames already acted on */
    uint8_t  period_hi;
    uint8_t  period_hi_valid;
} config_rx_t;

typedef struct {
    cfg_event_t event;
    uint8_t     cmd;
    uint8_t     value_code;     /* blink code value (value + 1), 0 = ACK only, 0xFF = factory reset */
    uint8_t     arg;            /* the command's value (CMD_SCENE / CMD_SCENE_FADE: main.c applies it) */
} cfg_result_t;

uint8_t      crc8_smbus(const uint8_t *d, uint32_t n);
void         settings_default(settings_t *s);
uint32_t     settings_steps(const settings_t *s);
void         config_rx_init(config_rx_t *r);
/* one decoded slot per frame. Returns CFG_NONE for a normal frame (outputs follow it). */
cfg_result_t config_process(config_rx_t *r, settings_t *s, const slot_t *slot);
/* apply one command directly (config frames after validation, UART console) */
cfg_result_t config_apply(config_rx_t *r, settings_t *s, uint8_t cmd, uint8_t arg);
/* build a config message into a slot payload (for tests / tools): out = raw bytes (5 or 10) */
void         config_build(uint8_t cmd, uint8_t arg, uint8_t ws2915, uint8_t *out);

#endif
