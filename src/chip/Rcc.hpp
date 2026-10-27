#pragma once

#include "Blocks.hpp"
#include "ClockMath.hpp"
#include "Clocks.hpp"
#include "Instances.hpp"
#include "WaitBounds.hpp"
#include "core/Barrier.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "peripherals/FLASH.hpp"
#include "peripherals/PWR.hpp"
#include "peripherals/RCC.hpp"

#include <array>
#include <cstdint>
#include <optional>
#include <type_traits>

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
// at the wrong clock, which the log line shows). It is not a latch: `Rcc::Recovery<Tree, Clock>`
// (chip/ClockRecovery.hpp), a Hook::MainLoop entry, finishes the switch-over at run time once the
// HSE runs (restarting it on a doubling pause meanwhile).
//
// init() runs in ClockSettings::coreClockInit(), before Startup sets up .data and .bss (so that
// runs at full speed): it writes registers only, and every wait in it is bounded with
// OnTimeout::Report whatever wait policy the application injects (an injected Count, Log or Panic
// would write .bss or a record there that initMemory() then wipes). A variable it set would be
// wiped by the .bss clear (such a result variable reads "not run" with the PLL running). How far
// it got is read back from RCC and FLASH instead - every reset but a power-on one puts RCC_CR,
// RCC_CFGR and FLASH_ACR back to their reset values too (RM0430 Rev 8 6.1.2, "A system reset sets
// all registers to their reset values except the reset flags in the clock controller CSR register
// and the registers in the Backup domain").
namespace Kvasir { namespace Rcc {
    enum class InitResult : std::uint8_t {
        notRun,
        ok,
        hseDidNotStart,   // stayed on HSI, the HSE left enabled (Recovery switches over once it runs)
        flashWaitStatesNotTaken,   // stayed on HSI: FLASH_ACR.LATENCY did not read back, the PLL left off
        pllDidNotLock,             // stayed on HSI (with the flash wait states already raised)
        switchDidNotTakeEffect,
    };

    namespace Detail {
        using RCC   = Kvasir::Peripheral::RCC::Registers<>;
        using FLASH = Kvasir::Peripheral::FLASH::Registers<>;
        using PWR   = Kvasir::Peripheral::PWR::Registers<>;
    }   // namespace Detail

    /// How far ClockTree::init() got, from the registers it leaves behind (RM0430 Rev 8 6.3.1
    /// RCC_CR: HSEON bit 16, PLLON 24, PLLRDY 25, reset 0x0000 XX81 = both off; 6.3.3 RCC_CFGR:
    /// SWS bits 3:2, 10 = PLL; 3.8.1 FLASH_ACR: ICEN bit 9, reset 0x0000 0000). Each failure leaves
    /// a state no earlier step does: init() turns the HSE on first, writes FLASH_ACR (ICEN set) only
    /// after HSERDY and the PLL on only after the wait states read back. Read after a later clock
    /// change it describes that one.
    inline InitResult initResult() {
        using RCC   = Detail::RCC;
        using FLASH = Detail::FLASH;
        if(apply(read(RCC::CFGR::sws)) == 0b10) { return InitResult::ok; }
        if(apply(read(RCC::CR::hseon)) == 0) { return InitResult::notRun; }
        if(apply(read(RCC::CR::pllon)) == 0) {
            return apply(read(FLASH::ACR::icen)) == 0 ? InitResult::hseDidNotStart
                                                      : InitResult::flashWaitStatesNotTaken;
        }
        if(apply(read(RCC::CR::pllrdy)) == 0) { return InitResult::pllDidNotLock; }
        return InitResult::switchDidNotTakeEffect;
    }

    /// One peripheral instance's bus and its clock enable (`OnBusOf<Peripheral::SPI::Registers<1>>`): the register
    /// half of Instances.hpp. The bus comes from the block's base address (Chip::busOfBase, RM0430 Table 1), the
    /// enable action is the block's row in Blocks.hpp (the gate check's table, so the two cannot disagree); it must
    /// sit in that bus's enable register (RM0430 Rev 8 6.3.15 RCC_APB1ENR, 6.3.16 RCC_APB2ENR), which Blocks.hpp
    /// asserts for every row and this again for the instance. A driver's `Instance<N>` derives from it and takes
    /// `bus` and `powerClockEnable` from it.
    template<typename Regs>
    struct OnBusOf {
        static_assert(
          Chip::onApb1(Regs::baseAddr) || Chip::onApb2(Regs::baseAddr),
          "this register set is on no APB bus (RM0430 Table 1): no RCC_APBxENR enables it");
        static constexpr Bus bus = Chip::busOfBase(Regs::baseAddr);
        using Enable             = Chip::Blocks::Table::OpenActions<Regs::baseAddr>;
        static_assert(Chip::Blocks::Table::rowsOf(Regs::baseAddr) == 1,
                      "this register set has no row in chip/Blocks.hpp");
        static_assert(Hal::actionWriteOf<Enable>.address
                        == Register::RegisterTraits<
                          typename std::conditional_t<bus == Bus::apb1,
                                                      Detail::RCC::APB1ENR,
                                                      Detail::RCC::APB2ENR>::Addr>::address,
                      "this enable bit is not in the RCC_APBxENR of the instance's bus (RM0430 "
                      "6.3.15/6.3.16)");
        static constexpr auto powerClockEnable = list(Enable{});
    };

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
        static constexpr std::uint32_t pll48Hz     = pll.vcoHz / pll.q;
        static constexpr unsigned      waitStates  = flashWaitStates(hclkHz, SupplyRange);
        static constexpr std::uint32_t vos         = voltageScale(hclkHz);
        static constexpr Supply        supply      = SupplyRange;

        /// A bus's peripheral clock (PCLK1 on APB1, PCLK2 on APB2) and its timers' kernel clock (APB1: TIM2-7,
        /// 12-14; APB2: TIM1, 8-11) (6.2), for a driver that knows its instance's bus (Instances.hpp):
        /// `Clocks::template pclkHz<I::bus>`.
        template<Bus B>
        static constexpr std::uint32_t pclkHz
          = hclkHz / (B == Bus::apb1 ? apb1Divider : apb2Divider);
        template<Bus B>
        static constexpr std::uint32_t timerHz
          = timerClock(hclkHz, B == Bus::apb1 ? apb1Divider : apb2Divider);

        /// What init() programs, as Startup resources (Clocks.hpp): a firmware's ClockSettings
        /// says `using Provides = ClockTree::Provides;`, and every driver claims its bus clock.
        using Provides = brigand::list<Startup::ProcessorClock<hclkHz>,
                                       Hal::Clk<Clocks::ClkApb1, pclkHz<Bus::apb1>>,
                                       Hal::Clk<Clocks::ClkApb2, pclkHz<Bus::apb2>>,
                                       Hal::Clk<Clocks::ClkTim1, timerHz<Bus::apb1>>,
                                       Hal::Clk<Clocks::ClkTim2, timerHz<Bus::apb2>>>;

        /// The whole switch-over, from reset (HSI 16 MHz, 0 wait states). The order is RM0430's:
        /// the regulator scale while the PLL is off (5.4.1), the wait states up before the clock
        /// goes up and read back (3.4.1 "Increasing the CPU frequency"), then the PLL, then the
        /// bus prescalers, then SYSCLK. Registers only, every wait Report (header comment).
        static void init() {
            using namespace Detail;
            // PWR's clock, then the VOS write. Errata ES0372 Rev 8 2.2.7 "Delay after an RCC
            // peripheral clock enabling": a DSB before the first access to the peripheral.
            apply(set(RCC::APB1ENR::pwren));
            Kvasir::Core::dsb();
            apply(write(PWR::CR::vos, Register::value<std::uint32_t, vos>()));

            apply(write(RCC::CR::hsebyp, Register::value<std::uint32_t, HseBypass ? 1 : 0>()));
            apply(set(RCC::CR::hseon));
            // Bounded (WaitBounds.hpp): init() gives up on a clock that does not come, it never panics.
            if(!Register::waitUntil<Kvasir::Chip::HseReadyBound, Register::OnTimeout::Report>(
                 Register::isSet(RCC::CR::hserdy)))
            {
                return;   // HSEON stays set: initResult() tells this from "never ran" by it
            }
            static_cast<void>(switchToPll());
        }

        /// init()'s second half, from a running HSE and the PLL off: the wait states, the PLL, the bus
        /// prescalers, SYSCLK = PLL. Also what Recovery runs at run time. Registers only, every wait
        /// Report; false where it stopped (initResult() says which step).
        static bool switchToPll() {
            using namespace Detail;
            using Register::isSet;
            using Register::waitUntil;
            using Register::OnTimeout::Report;
            // Flash: wait states, instruction and data cache, prefetch (3.8.1 FLASH_ACR). A whole
            // write from the reset value 0: ICRST is write-only ("w") and "can be written only
            // when the I cache is disabled", so a field write must not put back what a read of
            // it returns; ICRST and DCRST go out as 0 here.
            apply(FLASH::ACR::overrideDefaults(
              write(FLASH::ACR::latency, Register::value<std::uint32_t, waitStates>()),
              set(FLASH::ACR::icen),
              set(FLASH::ACR::dcen),
              set(FLASH::ACR::prften)));
            // the new latency must read back before the clock goes up (3.4.1): else the PLL stays off
            if(!waitUntil<Kvasir::Chip::ReadBackBound, Report>(
                 Register::FieldValue<std::remove_cvref_t<decltype(FLASH::ACR::latency)>,
                                      waitStates>{}))
            {
                return false;
            }

            apply(set(RCC::PLLCFGR::pllsrc),   // HSE
                  write(RCC::PLLCFGR::pllm, Register::value<std::uint32_t, pll.m>()),
                  write(RCC::PLLCFGR::plln, Register::value<std::uint32_t, pll.n>()),
                  write(RCC::PLLCFGR::pllp, Register::value<std::uint32_t, pll.p / 2 - 1>()),
                  write(RCC::PLLCFGR::pllq, Register::value<std::uint32_t, pll.q>()),
                  write(RCC::PLLCFGR::pllr, Register::value<std::uint32_t, pll.r>()));
            apply(set(RCC::CR::pllon));
            if(!waitUntil<Kvasir::Chip::PllLockBound, Report>(isSet(RCC::CR::pllrdy))) {
                return false;
            }
            // PWR_CSR VOSRDY (5.4.2): the new scale is applied once the PLL runs.
            static_cast<void>(
              waitUntil<Kvasir::Chip::VosReadyBound, Report>(isSet(PWR::CSR::vosrdy)));

            apply(write(RCC::CFGR::hpre, Register::value<std::uint32_t, 0>()),
                  write(RCC::CFGR::ppre1, Register::value<std::uint32_t, ppreBits(apb1Divider)>()),
                  write(RCC::CFGR::ppre2, Register::value<std::uint32_t, ppreBits(apb2Divider)>()));
            apply(write(RCC::CFGR::sw, Register::value<std::uint32_t, 0b10>()));   // PLL
            return static_cast<bool>(waitUntil<Kvasir::Chip::SwsBound, Report>(
              Register::FieldValue<std::remove_cvref_t<decltype(RCC::CFGR::sws)>, 0b10U>{}));
        }
    };
}}   // namespace Kvasir::Rcc
