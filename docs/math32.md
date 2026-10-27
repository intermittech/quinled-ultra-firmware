# Exact integer arithmetic

Release images use `math32.h/.c` instead of compiler 64-bit division helpers.
All twelve images are checked for signed/unsigned 64-bit multiply, divide,
remainder and shift helper symbols. Wide values remain where their mathematical
range needs them; the compiler implements their additions, comparisons and
constant shifts with native RV32 instructions.

For `a*b = hi*2^32 + lo`, `lo` is the ordinary unsigned product and `hi` is
MULHU. The portable host implementation splits each operand into 16-bit limbs.
Unsigned wrap is intentional. `m32_shr` joins the two words after a shift;
`m32_sar` subtracts `b` from the high word when the first operand is negative.
This preserves the low 32 bits of the old arithmetic right shift, including
negative nonintegral results. `m32_addshr` adds its rounding term with a carry.

`m32_div` computes the low word of an unsigned quotient. Reducing the high word
modulo the denominator removes only a multiple of 2^32 from the quotient.
Normalized base-65536 division then emits two digits, with at most two corrections
per digit. `m32_divwide` normalizes a nonzero high denominator word and uses a
bounded quotient estimate with one final correction. Zero divisors retain the
unsigned all-ones sentinel. `m32_divsat` saturates when high >= denominator.
For positive integer d and k, floor(floor(n/d)/k) = floor(n/(d*k)); this justifies
moving fixed-point shifts ahead of division without losing a rounding bit.

| Call sites | Range/identity and preserved behavior |
|---|---|
| Ultra gamma | `-log2(x) = n*2^31 + 4*q`; the negative signed division truncates toward zero, so its magnitude is the positive floor. `n<=31`, encoded gamma <=65535/1000, hence `n*g_q16` fits u32. Two words preserve the magnitude and the later ceiling. Horner products use MULHU. |
| Ultra lerps, clock conversions, phase and channel products | Joined product words preserve the old low-32-bit result after each signed/unsigned fixed-point shift; add-rounding carries are explicit. Full u32 helper tests cover all operand ranges at these sites. |
| Smoothstep | Endpoint 65536 is handled separately. Below it, p*p fits u32; two words preserve p cubed and the negative polynomial term. The final rounded shift is `(product_hi+128)>>8`. All 65537 legal inputs equal the frozen old expression. |
| Current sensing/regression | Five Q16 squared duties require 35 bits, or 39 after the learning-rate denominator. Predicted current products require 33 bits; their shifted sum is <=100000 mA. Error, absolute error and 4% bound fit s32. Division uses explicit numerator/denominator words; sign is restored after the positive floor. |
| Digital calibration | Admission caps input at 2250 mA and sampling at 80%, so Q8 current <=460800. The six weighted samples sum to <=411955200; `den=171010`, and `den*k*256<=1400913920` for k<=32. These accumulators fit u32. Larger coefficient products use exact word division, including the original rounding addend. Dark interpolation divides the magnitude and restores its sign (truncation toward zero). ADC gain times three fits u32 at the supported gain. |
| Digital limiter | Its wide numerator uses two words. Saturating a quotient >=2^32 before multiplying by cl_q8 is exact after the final 0x7fffff clamp: cl_q8>=8, so even the smallest saturated result exceeds that clamp. Pixel sums are below 2 million, so the 10/9 hysteresis products fit u32. |
| Effect integral | Multiply the two-word `k*sine` by delta, preserving the low 48 bits needed before the 16-bit shift. Legal k<=2441217, sine<=65534 and delta<=65535 also fit the host u64 oracle. Phase products preserve modulo-u32 behavior. |
| Return/strip clock inverses | 2^48 divided by rate is the word pair (65536,0). Invalid huge rate estimates saturate and fail the same admissible-rate interval test. Unwrapped host microseconds remain wide; their millisecond quotient is reduced modulo u32, the same result as the 64-bit expression. |
| Input power and energy | For mv=1000*q+r, floor(mv*ma/1000)=q*ma+floor(r*ma/1000). At 75000 mV and 65535 mA, power is exactly 4915125 mW. The residual microjoule accumulator is <3600000 before adding power, hence <8515125 afterward. Consume up to two mWh per millisecond. Per-channel integration similarly splits the multiplication by the sample count before division by 1000. |

Retained wide accumulators: unwrapped host microseconds and their differences;
regression norm/prediction above; ripple energy, whose squared current times Q16
duty exceeds u32 (at 65 A, even one full-duty squared term needs 49 bits), and
its integer square root; bench-only performance sums. The paired sine integral
for Pacifica also needs wide products; Pacifica is excluded from release images
by `FX_NO_PACIFICA=1`. No precision or timing margin is exchanged for storage.
Lifetime Wh, boot mWh and residual microjoules already have separate counters;
their specified wrap/saturation behavior is retained. The high-power overflow
of a direct u32 millivolt-times-milliamp product is corrected, rather than used
as an equivalence oracle.

`test_math32.c` uses host u64/s64 expressions as independent oracles: 2,000,000
full-range tuples plus edge Cartesian products; limiter and integral equivalence;
131,072,000 power remainder/current cases, with linear quotient proof through
75 V; 10,000 high-power energy steps; every smoothstep input and both lerp
extremes; every encoded gamma times nine boundary inputs and 3,000,000 random
gamma/lerp pairs. All have zero mismatches. The frozen curve reference is
`test/math32_reference.inc`. Existing effect goldens, calibration, strip-clock,
chain and current-sensing suites exercise the integrated call sites.
`test/test_math32_target.py` adds 2,400 comparisons against Python integers on
the actual RV32 division code, including MULHU. Estimated worst helper costs in
that sample are 185/247 cycles for narrow/wide division; these are emulator
estimates, not a silicon timing certificate.
