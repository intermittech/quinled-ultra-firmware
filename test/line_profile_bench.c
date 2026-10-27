/* SPDX-License-Identifier: GPL-3.0-or-later
 * Copyright (C) 2026 QuinLED */
#include "../src/core/ws_decode.h"
ws_line_t state;
uint16_t runs[WS_LINE_WORDS], samples[128];
uint8_t decoded[48];
void b_reset(void) { ws_line_init(&state); }
void b_feed(unsigned n) { ws_line_feed(&state,runs,samples,n); }
int b_decode(unsigned tail) { return ws_line_decode(&state,runs,tail?64:384,decoded,tail); }
unsigned b_profile(void) { return state.profile; }
unsigned b_threshold(unsigned p) { return ws_line_threshold(p); }
