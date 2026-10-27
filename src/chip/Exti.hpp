#pragma once

#include "Instances.hpp"
#include "Interrupt.hpp"
#include "Io.hpp"
#include "Rcc.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "kvasir/StartUp/SharedIsr.hpp"
#include "peripherals/EXTI.hpp"
#include "peripherals/SYSCFG.hpp"

#include <cstdint>

// An edge on a GPIO as an interrupt (RM0430 Rev 8 chapter 10.2 "External interrupt/event
// controller"): EXTI line n listens to pin n of ONE port, chosen in SYSCFG_EXTICR (8.2.3-8.2.6, 4
// bits a line: 0 = A ... 7 = H); RTSR/FTSR pick the edges (10.3.3, 10.3.4), IMR unmasks it
// (10.3.1), PR latches it and is cleared by writing 1 (10.3.6, rc_w1). Lines 0-4 have a vector
// each, 5-9 and 10-15 share one (Table 40) - so each line is a sub-interrupt of its vector
// (kvasir/StartUp/SharedIsr.hpp) and Startup generates the dispatcher:
//
//     void onTach();
//     using FanTach = Kvasir::Exti::Line<HW::Board::fanSense, Kvasir::Exti::Edge::rising, &onTach>;
//     using Startup = Kvasir::Startup::Startup<..., FanTach>;
namespace Kvasir { namespace Exti {
    enum class Edge { rising, falling, both };

    namespace Detail {
        using EXTI   = Kvasir::Peripheral::EXTI::Registers<>;
        using SYSCFG = Kvasir::Peripheral::SYSCFG::Registers<>;

        // the line's own generated field of IMR, RTSR, FTSR (MRn, TRn) and PR (PRn, write-one-to-clear in chip.svd)
        template<typename Reg, unsigned Line>
        using Bit = Register::FieldAt<Reg, Line>;

        // SYSCFG_EXTICR1-4 hold lines 0-3, 4-7, 8-11, 12-15, four bits a line (8.2.3-8.2.6)
        using Exticrs
          = brigand::list<SYSCFG::EXTICR1, SYSCFG::EXTICR2, SYSCFG::EXTICR3, SYSCFG::EXTICR4>;

        template<unsigned Line>
        using PortSelect
          = Register::FieldAt<brigand::at_c<Exticrs, Line / 4>, (Line % 4) * 4>;   // EXTIn
    }   // namespace Detail

    /// One EXTI line listens to one port (SYSCFG_EXTICRx, RM0430 Rev 8 8.2.3-8.2.6): a second Line on
    /// the same line number - another port's pin n - would rewrite EXTICR (the last one wins) and
    /// run both callbacks on every edge, so each Line provides its line and two are a build error
    /// ("EXTI line 4"). The SYSCFG and EXTI clock enables are shared by every line and provide nothing.
    struct ExtiLineTag {
        static constexpr char const* describeAs = "EXTI line";
    };

    template<typename Pin,
             Edge                  E,
             auto                  Fn,
             int                   Priority = -1,
             Io::PullConfiguration Pull     = Io::PullConfiguration::pullNone>
    struct Line {
        static constexpr unsigned line = static_cast<unsigned>(Pin::pin);
        static constexpr unsigned port = static_cast<unsigned>(Pin::port);
        static_assert(line <= 15,
                      "GPIO lines are EXTI 0 to 15");

        using Vector = Interrupt::ExtiVector<line>;

        using Provides = brigand::list<Startup::Resource<ExtiLineTag, line>>;

        static constexpr Detail::Bit<Detail::EXTI::IMR, line>  imr{};
        static constexpr Detail::Bit<Detail::EXTI::PR, line>   pr{};
        static constexpr Detail::Bit<Detail::EXTI::RTSR, line> rtsr{};
        static constexpr Detail::Bit<Detail::EXTI::FTSR, line> ftsr{};
        static constexpr Detail::PortSelect<line>              portSelect{};

        // SYSCFG for EXTICR, and EXTI's own APB clock: RCC_APB2ENR bit 15 EXITEN (RM0430 Rev 8
        // 6.3.16; missing from ST's SVD, added in chip.svd). Without it the EXTI registers take no
        // write - IMR/RTSR read back 0 and no edge ever interrupts. Both are Blocks.hpp rows.
        static constexpr auto powerClockEnable
          = list(Rcc::OnBusOf<Detail::SYSCFG>::powerClockEnable,
                 Rcc::OnBusOf<Detail::EXTI>::powerClockEnable);
        static constexpr auto initStepPinConfig = list(action(Io::Action::Input<Pull>{}, Pin{}));
        static constexpr auto initStepPeripheryConfig = list(
          write(portSelect, Register::value<std::uint32_t, port>()),   // write() shifts into place
          write(rtsr, Register::value<std::uint32_t, E != Edge::falling ? 1U : 0U>()),
          write(ftsr, Register::value<std::uint32_t, E != Edge::rising ? 1U : 0U>()));

        using SubIsrs = brigand::list<Nvic::SubIsr<Vector,
                                                   Fn,
                                                   Nvic::RawStatus<pr, imr>,
                                                   Nvic::ClearFirst<pr>,
                                                   Nvic::Enable<imr>,
                                                   Priority>>;
    };
}}   // namespace Kvasir::Exti
