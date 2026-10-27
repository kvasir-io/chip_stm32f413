#pragma once

// The STM32F413's bus and timer clocks as Startup resources (kvasir/Hal/Clocks.hpp): the
// ClockTree provides each at the rate it programs (`Rcc::ClockTree::Provides`), every driver
// claims the clock its block counts at the rate it computed its dividers from - so a driver
// handed another tree than the one ClockSettings programs does not build. RM0430 Rev 8 6.2:
// PCLK1 (APB1) and PCLK2 (APB2) feed the peripherals, the timers run from TIMxCLK (PCLKx, x2
// when the APB prescaler is not 1; 6.2 "timer clock frequencies").
//
// Mandatory providers: a firmware's ClockSettings says `using Provides = ClockTree::Provides;`.

#include "Instances.hpp"
#include "kvasir/Hal/Clocks.hpp"

#include <type_traits>

namespace Kvasir { namespace Clocks {
    struct ClkApb1 : Hal::ClockTag {};   // PCLK1

    struct ClkApb2 : Hal::ClockTag {};   // PCLK2

    struct ClkTim1 : Hal::ClockTag {};   // the APB1 timers' kernel clock (TIM2-7, 12-14)

    struct ClkTim2 : Hal::ClockTag {};   // the APB2 timers' (TIM1, 8-11)

    /// The tag of a bus's peripheral clock / its timers' kernel clock (Instances.hpp `Rcc::Bus`).
    template<Rcc::Bus B>
    using ClkApb = std::conditional_t<B == Rcc::Bus::apb1, ClkApb1, ClkApb2>;

    template<Rcc::Bus B>
    using ClkTim = std::conditional_t<B == Rcc::Bus::apb1, ClkTim1, ClkTim2>;
}}   // namespace Kvasir::Clocks
