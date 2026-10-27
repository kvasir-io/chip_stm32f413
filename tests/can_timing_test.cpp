// The bxCAN bit timing solver (ClockMath.hpp Can::solveBitTiming) against RM0430 Rev 8: a bit is 1 + tBS1 + tBS2
// quanta (SYNC_SEG one quantum, 32.7.7 md l.47301), tq = (BRP + 1) PCLK periods with a divider of 1..1024, tBS1 1..16
// and tBS2 1..8 quanta (CAN_BTR, 32.9.2 md l.48047-48061); the sample point is after SYNC_SEG + BS1. Each result gives
// the bit rate exactly, every field in its range, and no exact setting has a sample point closer to the one asked for
// (an exhaustive search here). Also the timing the conformance tree's Can::Bus computes (CAN1 at 50 MHz PCLK1 -
// 100 MHz SYSCLK over APB1's 50 MHz ceiling -, 500 kbit/s, 87.5 %; instantiate.cpp asserts the same numbers in the
// target build). Building is the test.
#include "ClockMath.hpp"

#include <cstdint>

namespace {
using Kvasir::Can::BitTiming;
using Kvasir::Can::solveBitTiming;

constexpr unsigned quanta(BitTiming const& t) { return 1 + t.ts1 + t.ts2; }

constexpr bool exact(std::uint32_t pclk,
                     std::uint32_t bitrate,
                     BitTiming     t) {
    return t.brp >= 1 && t.brp <= 1024 && t.ts1 >= 1 && t.ts1 <= 16 && t.ts2 >= 1 && t.ts2 <= 8
        && std::uint64_t{t.brp} * quanta(t) * bitrate == pclk
        && t.samplePermille == (1 + t.ts1) * 1000 / quanta(t);
}

// the closest sample point any exact setting reaches: every divider whose quantum count is whole, every split of it
// into BS1 + BS2 within their ranges (the fields' whole space; a divider that leaves no whole count has no setting)
constexpr unsigned bestDiff(std::uint32_t pclk,
                            std::uint32_t bitrate,
                            unsigned      want) {
    unsigned best = ~0U;
    for(unsigned brp = 1; brp <= 1024; ++brp) {
        if(pclk % (std::uint64_t{brp} * bitrate) != 0) { continue; }
        auto const q = static_cast<unsigned>(pclk / (std::uint64_t{brp} * bitrate));
        for(unsigned ts1 = 1; ts1 <= 16; ++ts1) {
            for(unsigned ts2 = 1; ts2 <= 8; ++ts2) {
                if(1 + ts1 + ts2 != q) { continue; }
                unsigned const sample = (1 + ts1) * 1000 / q;
                unsigned const diff   = sample > want ? sample - want : want - sample;
                best                  = diff < best ? diff : best;
            }
        }
    }
    return best;
}

constexpr bool closest(std::uint32_t pclk,
                       std::uint32_t bitrate,
                       unsigned      want) {
    auto const     t = solveBitTiming(pclk, bitrate, want);
    unsigned const diff
      = t.samplePermille > want ? t.samplePermille - want : want - t.samplePermille;
    return exact(pclk, bitrate, t) && diff == bestDiff(pclk, bitrate, want);
}

// the conformance tree's Can::Bus (tests/conformance/instantiate.cpp): 50 MHz PCLK1, 500 kbit/s, 875 permille - 100
// quanta a bit at BRP 1: too many for BS1 + BS2; BRP 5 gives 20 (1 + 16 + 3), 85.0 %
constexpr auto conf = solveBitTiming(50'000'000, 500'000, 875);
static_assert(conf.brp == 5 && conf.ts1 == 16 && conf.ts2 == 3 && conf.samplePermille == 850);
// 42 MHz: BRP 6, 14 quanta (1 + 11 + 2), 85.7 %
constexpr auto at42 = solveBitTiming(42'000'000, 500'000, 875);
static_assert(at42.brp == 6 && at42.ts1 == 11 && at42.ts2 == 2 && at42.samplePermille == 857);
static_assert(closest(42'000'000,
                      500'000,
                      875));
// other bus clocks and rates
static_assert(closest(50'000'000,
                      500'000,
                      875)
              && closest(50'000'000,
                         1'000'000,
                         750)
              && closest(48'000'000,
                         250'000,
                         800));
static_assert(closest(42'000'000,
                      125'000,
                      875)
              && closest(25'000'000,
                         100'000,
                         875));
// no exact setting: brp 0
static_assert(solveBitTiming(42'000'000,
                             333'333,
                             875)
                .brp
              == 0);
static_assert(solveBitTiming(1'000'000,
                             1'000'000,
                             875)
                .brp
              == 0);   // one quantum a bit is no bit
}   // namespace

int main() { return 0; }
