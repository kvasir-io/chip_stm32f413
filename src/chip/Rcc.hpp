#pragma once

#include "ClockMath.hpp"
#include "WaitBounds.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/FLASH.hpp"
#include "peripherals/PWR.hpp"
#include "peripherals/RCC.hpp"

#include <array>
#include <cstdint>
#include <optional>

// The clock tree of the STM32F413 (RM0430 Rev 8 chapter 6, "Reset and clock control"): HSE (or
// HSI) -> /PLLM -> VCO x PLLN -> /PLLP = SYSCLK, /PLLQ = 48 MHz domain (USB OTG FS, SDIO, RNG);
// SYSCLK -> /HPRE = HCLK -> /PPRE1 = PCLK1 (APB1, at most 50 MHz), /PPRE2 = PCLK2 (APB2, at most
// 100 MHz). Everything is solved at compile time from the crystal and the wanted SYSCLK:
//
//     using Tree = Kvasir::Rcc::ClockTree<8'000'000, 100'000'000>;
//     static_assert(Tree::pll.m == 4 && Tree::pll.n == 100);
//     Tree::init();   // in ClockSettings::coreClockInit
//
// init() never waits for ever: an HSE that does not start leaves the chip on the 16 MHz HSI, and
// Rcc::initResult() says so once logging runs (a board without its crystal still boots and logs -
// at the wrong clock, which the log line shows).
//
// init() runs in ClockSettings::coreClockInit(), before Startup sets up .data and .bss (so that
// runs at full speed): it writes registers only. A variable it set would be wiped by the .bss
// clear (psu302, 2026-10-06: "clock init notRun" with the PLL running). How far it got is read
// back from RCC instead - every reset but a power-on one puts RCC_CR and RCC_CFGR back to their
// reset values too (RM0430 Rev 8 6.1.2, "A system reset sets all registers to their reset values
// except the reset flags in the clock controller CSR register and the registers in the Backup
// domain").
namespace Kvasir { namespace Rcc {
    enum class InitResult : std::uint8_t {
        notRun,
        ok,
        hseDidNotStart,   // stayed on HSI, the HSE left enabled (it may still start: unused then)
        pllDidNotLock,    // stayed on HSI (with the flash wait states already raised)
        switchDidNotTakeEffect,
    };

    namespace Detail {
        using RCC   = Kvasir::Peripheral::RCC::Registers<>;
        using FLASH = Kvasir::Peripheral::FLASH::Registers<>;
        using PWR   = Kvasir::Peripheral::PWR::Registers<>;

        // A bounded wait: ~200 000 polls are several ms even at 100 MHz; the HSE needs about
        // 2 ms to start (DS11581 Rev 7 Table 43 "HSE 4-26 MHz oscillator characteristics", tSU(HSE)
        // typ. 2 ms), the PLL far less.
        template<typename F>
        bool waitFor(F f) {
            for(unsigned i = 0; i < 200'000; ++i) {
                if(f()) { return true; }
            }
            return false;
        }
    }   // namespace Detail

    /// How far ClockTree::init() got, from the registers it leaves behind (RM0430 Rev 8 6.3.1
    /// RCC_CR: HSEON bit 16, PLLON 24, PLLRDY 25, reset 0x0000 XX81 = both off; 6.3.3 RCC_CFGR:
    /// SWS bits 3:2, 10 = PLL). Each failure leaves a state no earlier step does: init() turns the
    /// HSE on first and the PLL on only after HSERDY. Read after a later clock change it describes
    /// that one.
    inline InitResult initResult() {
        using RCC = Detail::RCC;
        if(apply(read(RCC::CFGR::sws)) == 0b10) { return InitResult::ok; }
        if(apply(read(RCC::CR::hseon)) == 0) { return InitResult::notRun; }
        if(apply(read(RCC::CR::pllon)) == 0) { return InitResult::hseDidNotStart; }
        if(apply(read(RCC::CR::pllrdy)) == 0) { return InitResult::pllDidNotLock; }
        return InitResult::switchDidNotTakeEffect;
    }

    template<std::uint32_t InputHz,
             std::uint32_t SysclkHz,
             Supply        SupplyRange = Supply::from2V7,
             bool          HseBypass   = false,
             bool          Usb48       = false,
             std::uint32_t Apb2MaxHz   = 100'000'000>
    struct ClockTree {
        static constexpr std::uint32_t inputHz = InputHz;
        static_assert(InputHz >= 4'000'000 && InputHz <= 26'000'000,
                      "HSE crystal/clock is 4 to 26 MHz (DS11581 Rev 7 Table 43)");

        static constexpr auto solved = solvePll(InputHz, SysclkHz, Usb48);
        static_assert(solved.has_value(),
                      "no PLL setting gives this SYSCLK exactly from this crystal (RM0430 6.3.2)");
        static constexpr Pll pll = *solved;

        static constexpr std::uint32_t sysclkHz = SysclkHz;
        static constexpr std::uint32_t hclkHz   = SysclkHz;   // HPRE /1
        // RM0430 6.3.3: PPRE1 "not to exceed 50 MHz", PPRE2 "not to exceed 100 MHz".
        static constexpr unsigned apb1Divider = apbDivider(hclkHz, 50'000'000);
        // APB2 may be held lower: an SPI master that receives on SPI1/4/5 needs APB2 <= 84 MHz
        // (errata ES0372 Rev 8 2.14.4, Table 5) - Spi.hpp checks it.
        static_assert(Apb2MaxHz <= 100'000'000,
                      "APB2 is at most 100 MHz (RM0430 6.3.3)");
        static constexpr unsigned      apb2Divider = apbDivider(hclkHz, Apb2MaxHz);
        static constexpr std::uint32_t pclk1Hz     = hclkHz / apb1Divider;
        static constexpr std::uint32_t pclk2Hz     = hclkHz / apb2Divider;
        static constexpr std::uint32_t timer1Hz
          = timerClock(hclkHz, apb1Divider);   // TIM2-7, 12-14
        static constexpr std::uint32_t timer2Hz   = timerClock(hclkHz, apb2Divider);   // TIM1, 8-11
        static constexpr std::uint32_t pll48Hz    = pll.vcoHz / pll.q;
        static constexpr unsigned      waitStates = flashWaitStates(hclkHz, SupplyRange);
        static constexpr std::uint32_t vos        = voltageScale(hclkHz);

        /// The whole switch-over, from reset (HSI 16 MHz, 0 wait states). The order is RM0430's:
        /// the regulator scale while the PLL is off (5.4.1), the wait states up before the clock
        /// goes up and read back (3.4.1 "Increasing the CPU frequency"), then the PLL, then the
        /// bus prescalers, then SYSCLK.
        static void init() {
            using namespace Detail;
            // PWR's clock, then the VOS write. Errata ES0372 Rev 8 2.2.7 "Delay after an RCC
            // peripheral clock enabling": a DSB before the first access to the peripheral.
            apply(write(RCC::APB1ENR::pwren, Register::value<std::uint32_t, 1>()));
            asm volatile("dsb" ::: "memory");
            apply(write(PWR::CR::vos, Register::value<std::uint32_t, vos>()));

            apply(write(RCC::CR::hsebyp, Register::value<std::uint32_t, HseBypass ? 1 : 0>()));
            apply(write(RCC::CR::hseon, Register::value<std::uint32_t, 1>()));
            if(!waitFor([] { return apply(read(RCC::CR::hserdy)) != 0; })) {
                return;   // HSEON stays set: initResult() tells this from "never ran" by it
            }

            // Flash: wait states, instruction and data cache, prefetch (3.8.1 FLASH_ACR). A whole
            // write from the reset value 0: ICRST is write-only ("w") and "can be written only
            // when the I cache is disabled", so a field write must not put back what a read of
            // it returns; ICRST and DCRST go out as 0 here.
            apply(FLASH::ACR::overrideDefaults(
              write(FLASH::ACR::latency, Register::value<std::uint32_t, waitStates>()),
              write(FLASH::ACR::icen, Register::value<std::uint32_t, 1>()),
              write(FLASH::ACR::dcen, Register::value<std::uint32_t, 1>()),
              write(FLASH::ACR::prften, Register::value<std::uint32_t, 1>())));
            Kvasir::Register::waitUntil<Kvasir::Chip::ReadBackBound>(
              [] { return !(apply(read(FLASH::ACR::latency)) != waitStates); });

            apply(write(RCC::PLLCFGR::pllsrc, Register::value<std::uint32_t, 1>()),   // HSE
                  write(RCC::PLLCFGR::pllm, Register::value<std::uint32_t, pll.m>()),
                  write(RCC::PLLCFGR::plln, Register::value<std::uint32_t, pll.n>()),
                  write(RCC::PLLCFGR::pllp, Register::value<std::uint32_t, pll.p / 2 - 1>()),
                  write(RCC::PLLCFGR::pllq, Register::value<std::uint32_t, pll.q>()),
                  write(RCC::PLLCFGR::pllr, Register::value<std::uint32_t, pll.r>()));
            apply(write(RCC::CR::pllon, Register::value<std::uint32_t, 1>()));
            if(!waitFor([] { return apply(read(RCC::CR::pllrdy)) != 0; })) { return; }
            // PWR_CSR VOSRDY (5.4.2): the new scale is applied once the PLL runs.
            waitFor([] { return apply(read(PWR::CSR::vosrdy)) != 0; });

            apply(write(RCC::CFGR::hpre, Register::value<std::uint32_t, 0>()),
                  write(RCC::CFGR::ppre1, Register::value<std::uint32_t, ppreBits(apb1Divider)>()),
                  write(RCC::CFGR::ppre2, Register::value<std::uint32_t, ppreBits(apb2Divider)>()));
            apply(write(RCC::CFGR::sw, Register::value<std::uint32_t, 0b10>()));   // PLL
            waitFor([] { return apply(read(RCC::CFGR::sws)) == 0b10; });   // initResult() reads it
        }
    };
}}   // namespace Kvasir::Rcc
