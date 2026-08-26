#!/usr/bin/env python3
"""Generate + verify the NTC lookup table in rust/src/ina.rs.

The stock temperature model (HARDWARE.md "Temperature (NTC) & D±"):
    avg = sum16 / 16                       # 16-sample 12-bit ADC sum
    R   = avg * 11500 / (4095 - avg)       # ohms
    T   = 1/(1/298.15 + ln(R/10000)/3450) - 273.15   # trunc toward 0

Entry i of the emitted table = floor(sum16 at exactly (T0+i) °C), strictly
decreasing; lookup is a partition_point + the trunc-toward-zero fixup for
negatives. Verified bit-exact against the float model for every in-range
sum16 (single 1-LSB clamp at exactly -40 °C). The LUT replaced the float
path because logf + the soft-float builtins were ~2 KB of flash.
"""
import math

T0, T1 = -40, 125


def sum_at(t):
    r = 10000.0 * math.exp(3450.0 * (1.0 / (t + 273.15) - 1.0 / 298.15))
    return 4095.0 * r / (r + 11500.0) * 16.0


def temp_float(s):
    avg = s / 16.0
    if avg < 1.0 or avg > 4094.0:
        return 0
    r = avg * 11500.0 / (4095.0 - avg)
    return int(1.0 / (1.0 / 298.15 + math.log(r / 10000.0) / 3450.0) - 273.15)


def main():
    tab = [min(65535, int(math.floor(sum_at(t)))) for t in range(T0, T1 + 1)]
    assert all(a > b for a, b in zip(tab, tab[1:])), "not strictly decreasing"

    def temp_lut(s):
        if s < 16 or s > 16 * 4094:
            return 0
        if s >= tab[0]:
            return T0
        if s < tab[-1]:
            return T1
        lo, hi = 0, len(tab)
        while lo < hi:
            m = (lo + hi) // 2
            if tab[m] >= s:
                lo = m + 1
            else:
                hi = m
        t = T0 + lo - 1
        return t + 1 if t < 0 else t

    bad = [s for s in range(tab[-1], tab[0] + 1) if temp_float(s) != temp_lut(s)]
    assert bad == [tab[0]], "in-range mismatches beyond the -40 clamp: %r" % bad[:5]

    print("const NTC_LUT_T0: i32 = %d;" % T0)
    print("const NTC_LUT: [u16; %d] = [" % len(tab))
    for i in range(0, len(tab), 10):
        print("    %s," % ", ".join(str(v) for v in tab[i : i + 10]))
    print("];")


if __name__ == "__main__":
    main()
