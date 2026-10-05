#pragma once

#include "Io.hpp"
#include "core/core.hpp"
#include "kvasir/Common/Core.hpp"
#include "peripherals/RCC.hpp"

namespace Kvasir { namespace Startup {
    template<typename... Ts>
    struct FirstInitStep<Tag::User, Ts...> {
        void operator()() {
            Core::startup();   // FPU access (core_cortex_m4)

            // The clock of every port the package bonds (RCC_AHB1ENR, RM0430 Rev 8 6.3.11): every
            // pin action after this - and Io/Read of a pin nobody configured - reaches its port.
            // The DSB is errata ES0372 Rev 8 2.2.7 "Delay after an RCC peripheral clock enabling".
            using AHB1ENR = Kvasir::Peripheral::RCC::Registers<>::AHB1ENR;
            static_assert(Io::PinLocationTraits<void>::PortsBonded
                          == std::array<int, 5>{0, 1, 2, 3, 7});
            apply(write(AHB1ENR::gpioaen, Register::value<std::uint32_t, 1>()),
                  write(AHB1ENR::gpioben, Register::value<std::uint32_t, 1>()),
                  write(AHB1ENR::gpiocen, Register::value<std::uint32_t, 1>()),
                  write(AHB1ENR::gpioden, Register::value<std::uint32_t, 1>()),
                  write(AHB1ENR::gpiohen, Register::value<std::uint32_t, 1>()));
            asm volatile("dsb" ::: "memory");
        }
    };
}}   // namespace Kvasir::Startup

#include "kvasir/StartUp/StartUp.hpp"
