#pragma once
// The chip's clock arithmetic, free of registers so the host tests (tests/) can include it: the PLL
// solver and the flash/regulator tables (Rcc.hpp), timer, SPI, ADC and watchdog dividers. Every
// function is consteval/constexpr; the citations are at each function.
#include "kvasir/Util/Prescaler.hpp"

#include <array>
#include <cstdint>
#include <optional>

namespace Kvasir { namespace Chip {
    /// Prescaler::assertInTolerance as a value, for a driver's class-scope check:
    /// `static_assert(Chip::checkedInTolerance<divider.achieved, Baud, Tol, "USART baud rate">);` - a rate outside
    /// the tolerance fails with Prescaler's message (wanted, got, the error and the allowance in ppm).
    template<Prescaler::Rational     Achieved,
             std::uint64_t           Wanted,
             Prescaler::Tolerance    Tol,
             Diagnostic::FixedString What>
    inline constexpr bool checkedInTolerance
      = (Prescaler::assertInTolerance<Achieved, Wanted, Tol, What>(), true);
}}   // namespace Kvasir::Chip

namespace Kvasir { namespace Rcc {
    namespace Detail {
        // Not constexpr and never defined: reached in a constant evaluation it is a compile error
        // whose note shows the message (no exceptions in these builds).
        void clockConfigError(char const* why);
    }   // namespace Detail

    inline constexpr std::uint32_t hsiHz = 16'000'000;   // RM0430 6.2.2 "HSI clock"

    // The supply range the board runs from: the flash wait states depend on it (RM0430 Table 6).
    enum class Supply { from2V7, from2V4, from2V1, from1V7 };

    // RM0430 Rev 8 3.4.1 Table 6 "Number of wait states according to CPU clock (HCLK)
    // frequency": the highest HCLK in MHz for 0, 1, ... wait states, per supply range.
    consteval unsigned flashWaitStates(std::uint32_t hclkHz,
                                       Supply        supply) {
        constexpr std::array<std::array<unsigned, 7>, 4> maxMHz{
          {{25, 50, 75, 100, 0, 0, 0},
           {20, 40, 60, 80, 100, 0, 0},
           {18, 36, 54, 72, 90, 100, 0},
           {16, 32, 48, 64, 80, 96, 100}}
        };
        auto const& row = maxMHz[static_cast<std::size_t>(supply)];
        for(unsigned ws = 0; ws < row.size(); ++ws) {
            if(row[ws] != 0 && hclkHz <= row[ws] * 1'000'000U) { return ws; }
        }
        Detail::clockConfigError(
          "HCLK above 100 MHz: no flash wait state setting for it (RM0430 Table 6)");
        return 0;
    }

    // PWR_CR VOS (RM0430 5.4.1): the regulator scale the HCLK needs - "01: Scale 3 mode <= 64
    // MHz, 10: Scale 2 mode (reset value) <= 84 MHz, 11: Scale 1 mode <= 100 MHz".
    consteval std::uint32_t voltageScale(std::uint32_t hclkHz) {
        if(hclkHz <= 64'000'000) { return 0b01; }
        if(hclkHz <= 84'000'000) { return 0b10; }
        if(hclkHz <= 100'000'000) { return 0b11; }
        Detail::clockConfigError("HCLK above 100 MHz (RM0430 5.4.1)");
        return 0;
    }

    struct Pll {
        unsigned      m;   // 2..63, VCO input 1..2 MHz
        unsigned      n;   // 50..432, VCO output 100..432 MHz
        unsigned      p;   // 2, 4, 6, 8
        unsigned      q;   // 2..15
        unsigned      r;   // 2..7
        std::uint32_t vcoHz;
    };

    // The PLL for an exact SYSCLK (RM0430 6.3.2): VCO input 1 to 2 MHz, "it is recommended to
    // select a frequency of 2 MHz to limit PLL jitter" - so the smallest M, i.e. the highest VCO
    // input, wins; VCO 100 to 432 MHz; SYSCLK = VCO / P at most 100 MHz. Q is the smallest
    // divider that keeps the 48 MHz domain at or below 48 MHz ("The SDIO and the random number
    // generator need a frequency lower than or equal to 48 MHz"), exactly 48 MHz when usb48 is
    // asked for. R (I2S/DFSDM) keeps its reset value 2.
    consteval std::optional<Pll> solvePll(std::uint32_t inputHz,
                                          std::uint32_t sysclkHz,
                                          bool          usb48 = false) {
        if(sysclkHz > 100'000'000) { return std::nullopt; }
        for(unsigned m = 2; m <= 63; ++m) {
            if(inputHz % m != 0) { continue; }
            std::uint32_t const vcoIn = inputHz / m;
            if(vcoIn < 1'000'000 || vcoIn > 2'000'000) { continue; }
            for(unsigned p = 2; p <= 8; p += 2) {
                std::uint64_t const vco = std::uint64_t{sysclkHz} * p;
                if(vco < 100'000'000 || vco > 432'000'000 || vco % vcoIn != 0) { continue; }
                auto const n = static_cast<unsigned>(vco / vcoIn);
                if(n < 50 || n > 432) { continue; }
                for(unsigned q = 2; q <= 15; ++q) {
                    if(usb48 ? vco == std::uint64_t{48'000'000} * q
                             : vco <= std::uint64_t{48'000'000} * q)
                    {
                        return Pll{m, n, p, q, 2, static_cast<std::uint32_t>(vco)};
                    }
                }
            }
        }
        return std::nullopt;
    }

    // The smallest APB prescaler (1, 2, 4, 8, 16) that keeps the bus at or below its maximum.
    consteval unsigned apbDivider(std::uint32_t hclkHz,
                                  std::uint32_t maxHz) {
        for(unsigned d = 1; d <= 16; d *= 2) {
            if(hclkHz / d <= maxHz && hclkHz % d == 0) { return d; }
        }
        Detail::clockConfigError("no APB prescaler fits");
        return 0;
    }

    // RCC_CFGR PPREx encoding (6.3.3): 0xx = /1, 100 = /2, 101 = /4, 110 = /8, 111 = /16.
    consteval std::uint32_t ppreBits(unsigned divider) {
        switch(divider) {
        case 1:  return 0b000;
        case 2:  return 0b100;
        case 4:  return 0b101;
        case 8:  return 0b110;
        case 16: return 0b111;
        default: Detail::clockConfigError("APB divider is 1, 2, 4, 8 or 16"); return 0;
        }
    }

    // The timers' kernel clock with RCC_DCKCFGR.TIMPRE = 0 (its reset value; 6.3.27): HCLK while
    // PPREx divides by 1 or 2, else 2 x PCLKx.
    consteval std::uint32_t timerClock(std::uint32_t hclkHz,
                                       unsigned      apbDivider) {
        return apbDivider <= 2 ? hclkHz : 2 * (hclkHz / apbDivider);
    }

}}   // namespace Kvasir::Rcc

namespace Kvasir { namespace Timer {
    struct Division {
        std::uint32_t       psc;   // the register value: divides by psc + 1
        std::uint64_t       arr;   // the register value: period arr + 1 counts
        Prescaler::Rational achieved;
    };

    /// PSC and ARR for an update rate: the smallest prescaler whose period fits the counter, the
    /// period rounded to the nearest count (exact whenever clk / rate divides).
    consteval Division divisionFor(std::uint32_t clkHz,
                                   std::uint64_t rateHz,
                                   unsigned      bits) {
        std::uint64_t const maxPeriod = std::uint64_t{1} << bits;
        for(std::uint64_t psc = 0; psc <= 0xFFFF; ++psc) {
            // clk / ((psc + 1) x rate), rounded once
            std::uint64_t const den    = (psc + 1) * rateHz;
            std::uint64_t const period = (clkHz + den / 2) / den;
            if(period >= 2 && period <= maxPeriod) {
                return Division{
                  static_cast<std::uint32_t>(psc),
                  period - 1,
                  Prescaler::Rational{clkHz, (psc + 1) * period}
                };
            }
        }
        Prescaler::rateOutOfReach(rateHz, 0, 0);
        return {};
    }

}}   // namespace Kvasir::Timer

namespace Kvasir { namespace Spi {
    // BR for the fastest SCK <= wanted: PCLK / 2^(BR + 1).
    consteval unsigned baudBits(std::uint32_t pclkHz,
                                std::uint32_t wantedHz) {
        for(unsigned br = 0; br <= 7; ++br) {
            if(pclkHz >> (br + 1) <= wantedHz) { return br; }
        }
        return 7;
    }

}}   // namespace Kvasir::Spi

namespace Kvasir { namespace Adc { namespace Detail {
    consteval unsigned prescalerBits(std::uint32_t pclk2Hz) {
        for(unsigned bits = 0; bits <= 3; ++bits) {
            if(pclk2Hz / (2 * (bits + 1)) <= 36'000'000) { return bits; }
        }
        return 3;
    }

}}}   // namespace Kvasir::Adc::Detail

namespace Kvasir { namespace Iwdg {
    struct Setting {
        std::uint32_t pr;    // divider 4 << pr
        std::uint32_t rlr;   // 0..4095
    };

    inline constexpr std::uint32_t lsiMinHz = 16'100;
    inline constexpr std::uint32_t lsiMaxHz = 47'000;

    consteval Setting settingFor(std::uint64_t timeoutUs) {
        for(std::uint32_t pr = 0; pr <= 6; ++pr) {
            std::uint64_t const divider = 4U << pr;
            // ticks at the fastest LSI that make up the timeout, rounded up
            std::uint64_t const ticks
              = (timeoutUs * lsiMaxHz + divider * 1'000'000 - 1) / (divider * 1'000'000);
            if(ticks >= 1 && ticks <= 4096) {
                return Setting{pr, static_cast<std::uint32_t>(ticks - 1)};
            }
        }
        return Setting{6, 4095};
    }

    /// The timeout a setting gives at an LSI of `lsiHz`, in microseconds (rounded down): (RLR + 1) counts of the
    /// counter clock LSI / (4 << PR) (RM0430 Rev 8 22.3, 22.4.2; md l.29060, l.29174): settingFor()'s inverse,
    /// and the watchdog's shortest/longest timeout (Watchdog.hpp).
    constexpr std::uint64_t periodUs(Setting       s,
                                     std::uint32_t lsiHz) {
        return std::uint64_t{4U << s.pr} * (s.rlr + 1) * 1'000'000 / lsiHz;
    }

}}   // namespace Kvasir::Iwdg

// bxCAN's bit timing (Can.hpp Can::Bus solves its CAN_BTR from it; tests/can_timing_test.cpp). RM0430 Rev 8: a bit
// is SYNC_SEG (one quantum, 32.7.7, md l.47301) + BS1 + BS2; CAN_BTR (32.9.2, md l.48001-48061) gives
// tq = (BRP + 1) x tPCLK (BRP[9:0]: a divider of 1..1024), tBS1 = tq x (TS1 + 1) (TS1[3:0]: 1..16 quanta),
// tBS2 = tq x (TS2 + 1) (TS2[2:0]: 1..8 quanta); the sample point is after SYNC_SEG + BS1. Can::Bus writes SJW 0:
// one quantum (tRJW = tq x (SJW + 1), l.48043).
namespace Kvasir { namespace Can {
    struct BitTiming {
        unsigned brp;   // the divider, BRP + 1
        unsigned ts1;   // quanta, TS1 + 1
        unsigned ts2;   // quanta, TS2 + 1
        unsigned samplePermille;
    };

    // Every exact divider and split, the sample point closest to the wanted one; brp 0 = none.
    constexpr BitTiming solveBitTiming(std::uint32_t pclkHz,
                                       std::uint32_t bitrate,
                                       unsigned      wantPermille) {
        BitTiming best{0, 0, 0, 0};
        unsigned  bestDiff = ~0U;
        for(unsigned brp = 1; brp <= 1024; ++brp) {
            if(pclkHz % brp != 0 || (pclkHz / brp) % bitrate != 0) { continue; }
            unsigned const quanta = pclkHz / brp / bitrate;
            for(unsigned ts2 = 1; ts2 <= 8; ++ts2) {
                if(quanta < 1 + 1 + ts2) { break; }
                unsigned const ts1 = quanta - 1 - ts2;
                if(ts1 > 16) { continue; }
                unsigned const sample = (1 + ts1) * 1000 / quanta;
                unsigned const diff
                  = sample > wantPermille ? sample - wantPermille : wantPermille - sample;
                if(diff < bestDiff) {
                    bestDiff = diff;
                    best     = BitTiming{brp, ts1, ts2, sample};
                }
            }
        }
        return best;
    }
}}   // namespace Kvasir::Can
