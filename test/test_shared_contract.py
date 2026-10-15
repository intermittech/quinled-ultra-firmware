# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 QuinLED
"""Source guard for ISR publication contracts that the sequential host model cannot exercise.

This is a conservative source check, not a concurrency test or a silicon result.
Changing the synchronization design requires reviewing these guards with it.
"""
from pathlib import Path
import re
import sys

FW = Path(__file__).resolve().parents[1]
failures = []


def source(rel):
    text = (FW / rel).read_text()
    return re.sub(r'/\*[\s\S]*?\*/|//[^\n]*', '', text)


def function(text, name):
    m = re.search(r'\b' + re.escape(name) + r'\s*\([^;{}]*\)\s*\{', text)
    if not m:
        failures.append(f"missing function {name}")
        return ""
    start = end = m.end()
    depth = 1
    while end < len(text) and depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end-1]


def require(condition, why):
    if not condition:
        failures.append(why)


def ordered(text, parts, why):
    pos = 0
    for part in parts:
        m = re.search(part, text[pos:])
        if not m:
            failures.append(why)
            return
        pos += m.end()


header = source("src/core/isense.h")
for type_, member in (("uint32_t", "acc"), ("uint32_t", "acc_n"), ("uint16_t", "fast_raw"),
                      ("uint8_t", "fast_cnt"), ("uint8_t", "tripped"), ("uint16_t", "peak_raw")):
    require(re.search(r'\bvolatile\s+'+type_+r'\s+'+member+r'\s*;', header),
            f"isense.{member}: ISR/main shared field must remain volatile")

def released(text, var, why):
    """every release of a mailbox (var = 0) directly after a BARRIER() (0.6.4: the ISR may refill it at once)"""
    hits = list(re.finditer(r'\b' + var + r'\s*=\s*0\s*;', text))
    require(hits, why + " (no release found)")
    for m in hits:
        require(re.search(r'BARRIER\s*\(\s*\)\s*;\s*$', text[:m.start()]), why)


main = source("src/app/main.c")
barrier = r'\bBARRIER\s*\(\s*\)\s*;'
# (0.6.4, gpt-verify's mailbox ownership: main decodes while it owns the mailbox, the ISR leaves an owned one alone;
# an acquire barrier before the decode, a barrier before the release)
ordered(function(main, "process_capture"), (barrier, r'ws_decode_frame\s*\(', barrier, r'cap_ready\s*=\s*0\s*;'),
        "capture: acquire barrier, decode din_copy while owned, barrier before releasing cap_ready")
ordered(function(main, "process_frame_end"), (barrier, r'ws_decode_last\s*\(', barrier, r'fe_ready\s*=\s*0\s*;'),
        "frame end: acquire barrier, decode tail_copy while owned, barrier before releasing fe_ready")
released(function(main, "process_capture"), "cap_ready", "capture: every release of cap_ready right after a BARRIER")
released(function(main, "process_frame_end"), "fe_ready", "frame end: every release of fe_ready right after a BARRIER")
ordered(function(main, "process_frame_end"), (r'plan_buf\s*\[\s*i\s*\]\s*=', barrier, r'plan_idx\s*=\s*i\s*;'),
        "gate plan: barrier must separate completed plan from published plan_idx")

hw = source("src/hal/hw.c")
hw_cold = source("src/hal/hw_cold.c")       # (round 9) the boot and the flash operation (flash_op)
ordered(function(hw, "pwm_set_phase"), (r'slew_tgt2\s*=', r'slew_tgt3\s*=', barrier,
                                       r'TIM2->DMAINTENR\s*\|=\s*TIM_UIE'),
        "phase: barrier must publish targets before enabling slew interrupts")
ordered(function(hw, "pwm_start"), (r'TIM2->DMAINTENR\s*=\s*0', r'TIM3->DMAINTENR\s*=\s*0', barrier,
                                   r'cur_steps\s*=\s*steps'),
        "PWM restart: stop slew interrupts and barrier before replacing live timer settings")
ordered(function(hw_cold, "flash_op"), (r'__disable_irq\s*\(', r'gate_force_closed\s*\('),   # (0.6.4: flash_op)
        "flash: disable interrupts before closing the gate (poll must not reopen it)")

# (0.6.10) the compressed text's pointer: one in-range index added to cz_text (the bias with the offset first).
# cz_text + CZ_BIAS on its own lies past the blob's end when the blob is shorter than the bias (the receiver's
# release: 1,852 of 2,048 B): undefined in C, and no compiler or sanitizer here reports it (the code is the same)
cli = source("src/core/cli.c")
require(re.search(r'#define\s+cz_at\s*\(\s*z\s*\)\s*\(\s*cz_text\s*\+\s*\(\s*CZ_BIAS\s*\+\s*\(\s*z\s*\)\s*\)\s*\)', cli),
        "cli.c cz_at: cz_text + (CZ_BIAS + (z)), never a pointer formed from the bias alone")
require(len(re.findall(r'\bcz_text\b', cli)) == 1, "cli.c: cz_text used through cz_at only")

if failures:
    for error in failures:
        print("FAIL shared contract:", error)
    sys.exit(1)
print("ISR shared-source contracts passed (6 qualifiers, 5 barriers, mailbox releases, flash gate ordering); "
      "cz_at's pointer")
