#pragma once
// The run-time half of Rcc::ClockTree::init() (Rcc.hpp): init() runs before the memory init and gives up on a clock
// that does not come - the chip then runs on the 16 MHz HSI. That is no latch: this Hook::MainLoop entry finishes
// the switch-over once the cause is gone.
//
//     using ClockRecovery = Kvasir::Rcc::Recovery<HW::ClockTree, Clock>;
//     Startup<..., ClockRecovery>;   // a loop that runs Startup::run<Kvasir::Hook::MainLoop>()
//
// While Rcc::initResult() is not `ok`, turn() tries again on a doubling pause (Kvasir::Backoff on TimeClock: 100 ms up
// to 10 s; a SysTick clock set up for the tree's SYSCLK runs SYSCLK / 16 MHz times slow on the HSI, and its pauses
// with it): with HSERDY set it runs ClockTree::switchToPll() (the wait states, the PLL, the prescalers, SYSCLK = PLL);
// with HSERDY clear it restarts the HSE (HSEON off and on: a crystal that did not start gets a new start-up).
// A PLL that did not lock or a switch that did not take is undone first (SW back to HSI, PLLON off: RM0430 Rev 8
// 6.3.1 "This bit cannot be reset if PLL clock is used as the system clock", md l.5661; PLLCFGR "can be written only
// when PLL is disabled", 6.3.2 md l.5807/5813). Nothing once it is `ok`, and nothing for `notRun` (init() did not
// run: the clocks are someone else's).
//
// Every clock follows SYSCLK: until the switch the core and the buses run at 16 MHz (the reset prescalers), slower
// than every driver was configured for (a UART's baud rate, a timer's period), from the switch on at the configured
// rates. A byte on a wire at that moment can be garbled once. `recoveries()` counts the switches it made, `attempts()`
// the tries of the current outage.
#include "Rcc.hpp"
#include "kvasir/Port/Extends.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Backoff.hpp"
#include "peripherals/RCC.hpp"

#include <chrono>
#include <cstdint>

namespace Kvasir { namespace Rcc {
    /// The pauses between two tries: the first after 100 ms (an HSE start-up is typ. 2 ms, DS11581 Rev 7 Table 43),
    /// at most 10 s.
    struct RecoveryPauses {
        static constexpr std::chrono::milliseconds first{100};
        static constexpr std::chrono::seconds      longest{10};
    };

    template<typename Tree, typename TimeClock>
    struct Recovery {
        using RCC = Detail::RCC;

        /// The Hook::MainLoop entry: one register read while the clock is ok.
        static void turn() {
            auto const state = initResult();
            if(state == InitResult::ok || state == InitResult::notRun) { return; }
            auto const now = TimeClock::now();
            // the failure seen for the first time: the first pause starts (the HSE may still be starting)
            if(backoff_.failures == 0) {
                backoff_.failed(now);
                return;
            }
            if(!backoff_.due(now)) { return; }
            if(attempt(state)) {
                backoff_.succeeded();
                recoveries_ = recoveries_ + 1;
            } else {
                backoff_.failed(now);
            }
        }

        using Extends = Startup::Extend<Kvasir::Hook::MainLoop, &Recovery::turn>;

        /// Switches back to the PLL this entry made since boot.
        [[nodiscard]] static std::uint32_t recoveries() { return recoveries_; }

        /// Tries since the outage was seen (0 while the clock is ok).
        [[nodiscard]] static std::uint32_t attempts() { return backoff_.failures; }

    private:
        static bool attempt(InitResult state) {
            if(state == InitResult::hseDidNotStart && apply(read(RCC::CR::hserdy)) == 0) {
                // HSEBYP is kept: it may only change with the HSE off (6.3.1, md l.5687), and init() set it
                apply(clear(RCC::CR::hseon));
                apply(set(RCC::CR::hseon));
                return false;   // the next due turn finds HSERDY
            }
            if(apply(read(RCC::CR::hserdy)) == 0) {
                return false;
            }   // the HSE stopped since: wait for it
            if(state == InitResult::pllDidNotLock || state == InitResult::switchDidNotTakeEffect) {
                apply(write(RCC::CFGR::sw, Register::value<std::uint32_t, 0b00>()));   // HSI
                apply(clear(RCC::CR::pllon));
            }
            return Tree::switchToPll();
        }

        static inline Kvasir::Backoff<TimeClock, RecoveryPauses> backoff_{};
        static inline std::uint32_t volatile recoveries_{};
    };
}}   // namespace Kvasir::Rcc
