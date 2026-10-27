#pragma once
// Which APB a peripheral block of the STM32F413 sits on (by its base address), and the SPI clock limits: the facts
// the drivers' `Instance<N>` rows take (Spi.hpp, Usart.hpp, Timer.hpp, I2c.hpp, Can.hpp, Adc.hpp, Exti.hpp). Free of
// registers so the host tests (tests/instances_test.cpp) include it; the register half - which RCC_APBxENR bit
// enables an instance - is `Rcc::OnBusOf<Regs>` in Rcc.hpp.
#include "ClockMath.hpp"

#include <cstdint>

namespace Kvasir { namespace Rcc {
    /// The two peripheral buses (RM0430 Rev 8 2.2.2 Table 1 "Register boundary addresses"): APB1 at
    /// PCLK1, APB2 at PCLK2 (6.2).
    enum class Bus { apb1, apb2 };
}}   // namespace Kvasir::Rcc

namespace Kvasir { namespace Chip {
    // Where a peripheral's registers are says which bus it is on (RM0430 Rev 8 2.2.2 Table 1 "Register boundary
    // addresses"): APB1 0x4000 0000 - 0x4000 7FFF (TIM2 at its bottom, md l.2767, UART8 at its top, l.2736), APB2
    // 0x4001 0000 - 0x4001 FFFF (TIM1 to the reserved block after DFSDM2, picture text l.2723), AHB1 0x4002 0000 -
    // 0x4002 67FF (GPIOA l.2710 to DMA2 l.2695).
    constexpr bool onApb1(std::uint32_t base) {
        return base >= 0x4000'0000U && base <= 0x4000'7FFFU;
    }

    constexpr bool onApb2(std::uint32_t base) {
        return base >= 0x4001'0000U && base <= 0x4001'FFFFU;
    }

    constexpr bool onAhb1(std::uint32_t base) {
        return base >= 0x4002'0000U && base <= 0x4002'67FFU;
    }

    // Not constexpr: reached in busOfBase(), the compile error names it.
    inline void baseIsOnNoApbBus() {}

    /// The APB of the peripheral block at `base` (its generated `Regs::baseAddr`): what Rcc::OnBusOf takes the bus
    /// and the enable register from, so an instance row cannot name a wrong bus.
    consteval Rcc::Bus busOfBase(std::uint32_t base) {
        if(onApb1(base)) { return Rcc::Bus::apb1; }
        if(!onApb2(base)) { baseIsOnNoApbBus(); }
        return Rcc::Bus::apb2;
    }

    /// The fastest SCK an SPI master may run (DS11581 Rev 7 Table 67 "SPI dynamic characteristics"):
    /// SPI2/3 25 MHz; SPI1/4/5 as a master that receives 50 MHz from 3.0 V, 42 MHz from 2.7 V, 25 MHz
    /// from 1.7 V, and as a master transmitter (nothing received: no MISO) 50 MHz from 1.71 V.
    /// `Supply` has no 3.0 V range (RM0430 Table 6 splits at 2.7 V), so a receiving master gets 42 MHz
    /// at `from2V7`.
    consteval std::uint32_t spiMaxSckHz(unsigned    n,
                                        Rcc::Supply supply,
                                        bool        receives) {
        if(n == 2 || n == 3) { return 25'000'000; }   // SPI2/3, the APB1 ones
        if(!receives) { return 50'000'000; }
        return supply == Rcc::Supply::from2V7 ? 42'000'000 : 25'000'000;
    }
}}   // namespace Kvasir::Chip
