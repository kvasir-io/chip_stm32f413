#pragma once

#include "ClockMath.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/DBG.hpp"
#include "peripherals/IWDG.hpp"

#include <chrono>
#include <cstdint>

// The independent watchdog (RM0430 Rev 8 chapter 22): a 12-bit down-counter on the LSI through a
// prescaler of 4 to 256; KR 0xCCCC starts it (and the LSI), 0xAAAA reloads it, 0x5555 unlocks PR
// and RLR (22.4.1). Once started nothing but a reset stops it.
//
// The LSI is anywhere from 16.1 to 47 kHz (DS11581 Rev 7 Table 46 "LSI oscillator
// characteristics"), so the setting is chosen for the SHORTEST timeout (LSI at 47 kHz) to be at
// least the one asked for; the longest (16.1 kHz) is about 2.9 times that and is `longest`.
//
//     using Watchdog = Kvasir::Iwdg::Watchdog<500>;   // ms
//     Watchdog::start();
//     Watchdog::feed();   // once a main-loop turn
//
// It is held while a debugger halts the core (DBGMCU_APB1_FZ.DBG_IWDG_STOP, 34.16.4), so a
// breakpoint does not reset the board.
namespace Kvasir { namespace Iwdg {
    template<std::uint64_t TimeoutMs>
    struct Watchdog {
        static constexpr std::uint64_t timeoutUs = TimeoutMs * 1000;
        static constexpr Setting       setting   = settingFor(timeoutUs);
        static_assert(std::uint64_t{4U << setting.pr} * (setting.rlr + 1) * 1'000'000 / lsiMaxHz
                        >= timeoutUs,
                      "the watchdog's longest timeout is 4096 x 256 / 47 kHz = 22 s");

        static constexpr std::uint64_t shortestUs
          = std::uint64_t{4U << setting.pr} * (setting.rlr + 1) * 1'000'000 / lsiMaxHz;
        static constexpr std::uint64_t longestUs
          = std::uint64_t{4U << setting.pr} * (setting.rlr + 1) * 1'000'000 / lsiMinHz;

        using IWDG = Kvasir::Peripheral::IWDG::Registers<>;
        using DBG  = Kvasir::Peripheral::DBG::Registers<>;

        // KR 0xCCCC first (it starts the LSI), then 0x5555 and the setting; PR/RLR take effect once
        // SR.PVU/RVU drop (22.4.4), which the first feed() does not need to wait for.
        static void start() {
            apply(write(DBG::DBGMCU_APB1_FZ::iwdeg_stop, Register::value<std::uint32_t, 1>()));
            apply(write(IWDG::KR::key, Register::value<std::uint32_t, 0xCCCC>()));
            apply(write(IWDG::KR::key, Register::value<std::uint32_t, 0x5555>()));
            apply(write(IWDG::PR::pr, Register::value<std::uint32_t, setting.pr>()));
            apply(write(IWDG::RLR::rl, Register::value<std::uint32_t, setting.rlr>()));
            feed();
        }

        static void feed() {
            apply(write(IWDG::KR::key, Register::value<std::uint32_t, 0xAAAA>()));
        }
    };
}}   // namespace Kvasir::Iwdg
