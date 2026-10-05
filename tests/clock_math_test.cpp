// src/chip/ClockMath.hpp: the numbers the psu302 board runs with, and the edges of every table.
#include "ClockMath.hpp"

#include <cstdio>

namespace {
using namespace Kvasir;

// The board: 8 MHz crystal to 100 MHz - the PLL its CubeMX project used (M 4, N 100, P 2), VCO
// input 2 MHz as RM0430 6.3.2 recommends, Q 5 keeps the 48 MHz domain at 40 MHz.
constexpr auto board = Rcc::solvePll(8'000'000, 100'000'000);
static_assert(board && board->m == 4 && board->n == 100 && board->p == 2 && board->q == 5);
static_assert(board->vcoHz == 200'000'000);
// USB: 48 MHz exactly needs a VCO that both 100 MHz x P and 48 MHz x Q reach - 96 MHz does (P 4,
// Q 8 at 384 MHz), 100 MHz does not.
static_assert(!Rcc::solvePll(8'000'000,
                             100'000'000,
                             true));
constexpr auto usb = Rcc::solvePll(8'000'000, 96'000'000, true);
static_assert(usb && usb->vcoHz / usb->q == 48'000'000 && usb->vcoHz / usb->p == 96'000'000);
// above 100 MHz: none
static_assert(!Rcc::solvePll(8'000'000,
                             120'000'000));
// a 25 MHz crystal: the smallest M that divides it into 1..2 MHz is 16 (1.5625 MHz), then N 128 P 2
constexpr auto c25 = Rcc::solvePll(25'000'000, 100'000'000);
static_assert(c25 && c25->m == 16 && c25->n == 128 && c25->p == 2);

// RM0430 Table 6: 3 wait states at 100 MHz from 2.7 V, 0 at 25, 1 just above; 6 at 100 MHz
// below 2.1 V.
static_assert(Rcc::flashWaitStates(100'000'000,
                                   Rcc::Supply::from2V7)
              == 3);
static_assert(Rcc::flashWaitStates(25'000'000,
                                   Rcc::Supply::from2V7)
              == 0);
static_assert(Rcc::flashWaitStates(25'000'001,
                                   Rcc::Supply::from2V7)
              == 1);
static_assert(Rcc::flashWaitStates(100'000'000,
                                   Rcc::Supply::from1V7)
              == 6);
// RM0430 5.4.1: scale 3 to 64 MHz, 2 to 84, 1 to 100
static_assert(Rcc::voltageScale(64'000'000) == 0b01 && Rcc::voltageScale(84'000'000) == 0b10
              && Rcc::voltageScale(100'000'000) == 0b11);
// APB1 50 MHz at most, APB2 100
static_assert(Rcc::apbDivider(100'000'000,
                              50'000'000)
              == 2);
static_assert(Rcc::apbDivider(100'000'000,
                              100'000'000)
              == 1);
static_assert(Rcc::ppreBits(1) == 0 && Rcc::ppreBits(2) == 0b100 && Rcc::ppreBits(16) == 0b111);
// TIMPRE = 0: the timer clock is HCLK for /1 and /2, 2 x PCLK beyond
static_assert(Rcc::timerClock(100'000'000,
                              2)
              == 100'000'000);
static_assert(Rcc::timerClock(100'000'000,
                              4)
              == 50'000'000);

// Timers: the DAC tick at 50 kHz exactly (PSC 0, ARR 1999), the fan at 25 kHz (ARR 3999)
constexpr auto tick = Timer::divisionFor(100'000'000, 50'000, 16);
static_assert(tick.psc == 0 && tick.arr == 1999);
constexpr auto fan = Timer::divisionFor(100'000'000, 25'000, 16);
static_assert(fan.psc == 0 && fan.arr == 3999);
// 1 Hz on a 16-bit timer needs a prescaler: the smallest that fits
constexpr auto slow = Timer::divisionFor(100'000'000, 1, 16);
// (100 MHz / 1526 = 65530.8 counts a second, rounded to 65531)
static_assert(slow.psc == 1525 && slow.arr == 65530);

// SPI: PCLK / 2^(BR+1) at or below what is asked; 25 MHz from APB1's 50 MHz is BR 0
static_assert(Spi::baudBits(50'000'000,
                            25'000'000)
              == 0);
static_assert(Spi::baudBits(100'000'000,
                            2'000'000)
              == 5);   // 1.5625 MHz
static_assert(Spi::baudBits(100'000'000,
                            1)
              == 7);   // the slowest there is

// ADC clock at most 36 MHz: PCLK2 100 MHz / 4 = 25 MHz (ADCPRE 01)
static_assert(Adc::Detail::prescalerBits(100'000'000) == 1);
static_assert(Adc::Detail::prescalerBits(50'000'000) == 0);

// Watchdog: a setting whose shortest timeout (LSI 47 kHz) is at least the one asked
constexpr auto w500 = Iwdg::settingFor(500'000);
static_assert(std::uint64_t{4U << w500.pr} * (w500.rlr + 1) * 1'000'000 / Iwdg::lsiMaxHz
              >= 500'000);
constexpr auto w3s = Iwdg::settingFor(3'000'000);
static_assert(std::uint64_t{4U << w3s.pr} * (w3s.rlr + 1) * 1'000'000 / Iwdg::lsiMaxHz
              >= 3'000'000);
}   // namespace

int main() {
    std::printf(
      "PLL M %u N %u P %u Q %u; tick PSC %u ARR %u; slow PSC %u ARR %u; IWDG 500 ms PR %u RLR %u, "
      "3 s PR %u RLR %u\n",
      board->m,
      board->n,
      board->p,
      board->q,
      tick.psc,
      static_cast<unsigned>(tick.arr),
      slow.psc,
      static_cast<unsigned>(slow.arr),
      w500.pr,
      w500.rlr,
      w3s.pr,
      w3s.rlr);
    return 0;
}
