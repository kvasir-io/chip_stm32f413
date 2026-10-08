#pragma once

#include "Interrupt.hpp"
#include "Io.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/SharedIsr.hpp"
#include "peripherals/EXTI.hpp"
#include "peripherals/RCC.hpp"
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
        using RCC    = Kvasir::Peripheral::RCC::Registers<>;

        template<typename Reg, unsigned Line>
        using Bit = Register::
          FieldLocation<typename Reg::Addr, (1U << Line), Register::ReadWriteAccess, std::uint32_t>;

        template<unsigned Line>
        struct Exticr;

        template<>
        struct Exticr<0> {
            using type = SYSCFG::EXTICR1;
        };

        template<>
        struct Exticr<1> {
            using type = SYSCFG::EXTICR2;
        };

        template<>
        struct Exticr<2> {
            using type = SYSCFG::EXTICR3;
        };

        template<>
        struct Exticr<3> {
            using type = SYSCFG::EXTICR4;
        };

        template<unsigned Line>
        using PortSelect = Register::FieldLocation<typename Exticr<Line / 4>::type::Addr,
                                                   (0xFU << ((Line % 4) * 4)),
                                                   Register::ReadWriteAccess,
                                                   std::uint32_t>;
    }   // namespace Detail

    template<typename Pin,
             Edge                  E,
             auto                  Fn,
             int                   Priority = -1,
             Io::PullConfiguration Pull     = Io::PullConfiguration::PullNone>
    struct Line {
        static constexpr unsigned line = static_cast<unsigned>(Pin::pin);
        static constexpr unsigned port = static_cast<unsigned>(Pin::port);
        static_assert(line <= 15,
                      "GPIO lines are EXTI 0 to 15");

        using Vector = Interrupt::ExtiVector<line>;

        static constexpr Detail::Bit<Detail::EXTI::IMR, line>  imr{};
        static constexpr Detail::Bit<Detail::EXTI::PR, line>   pr{};
        static constexpr Detail::Bit<Detail::EXTI::RTSR, line> rtsr{};
        static constexpr Detail::Bit<Detail::EXTI::FTSR, line> ftsr{};
        static constexpr Detail::PortSelect<line>              portSelect{};

        // SYSCFG for EXTICR, and EXTI's own APB clock: RCC_APB2ENR bit 15 EXITEN (RM0430 Rev 8
        // 6.3.16; missing from ST's SVD, added in chip.svd). Without it the EXTI registers take no
        // write - IMR/RTSR read back 0 and no edge ever interrupts.
        static constexpr auto powerClockEnable
          = list(write(Detail::RCC::APB2ENR::syscfgen, Register::value<std::uint32_t, 1>()),
                 write(Detail::RCC::APB2ENR::extiten, Register::value<std::uint32_t, 1>()));
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
