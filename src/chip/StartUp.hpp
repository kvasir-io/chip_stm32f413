#pragma once

#include "Io.hpp"
#include "chip/Interrupt.hpp"   // the vector table below reads InterruptOffsetTraits
#include "core/core.hpp"
#include "kvasir/Port/Hooks.hpp"
#include "peripherals/RCC.hpp"

#include <cstddef>
#include <tuple>
#include <utility>

namespace Kvasir { namespace Startup {
    template<typename... Ts>
    struct FirstInitStep<Tag::User, Ts...> {
        void operator()() {
            Core::startup();   // FPU access (core_cortex_m4)

            // The clock of every port the package bonds (RCC_AHB1ENR, RM0430 Rev 8 6.3.11): every
            // pin action after this - and Io/Read of a pin nobody configured - reaches its port.
            // The DSB is errata ES0372 Rev 8 2.2.7 "Delay after an RCC peripheral clock enabling".
            enableBondedPorts(
              std::make_index_sequence<Io::PinLocationTraits<void>::portsBonded.size()>{});
            Kvasir::Core::dsb();
        }

    private:
        // AHB1ENR.GPIOxEN of ports A (0) to H (7), one write for the ports Io.hpp's PortsBonded lists
        template<std::size_t... I>
        static void enableBondedPorts(std::index_sequence<I...>) {
            using AHB1ENR = Kvasir::Peripheral::RCC::Registers<>::AHB1ENR;
            constexpr std::tuple portEnables{AHB1ENR::gpioaen,
                                             AHB1ENR::gpioben,
                                             AHB1ENR::gpiocen,
                                             AHB1ENR::gpioden,
                                             AHB1ENR::gpioeen,
                                             AHB1ENR::gpiofen,
                                             AHB1ENR::gpiogen,
                                             AHB1ENR::gpiohen};
            apply(set(std::get<Io::PinLocationTraits<void>::portsBonded[I]>(portEnables)...));
        }
    };
}}   // namespace Kvasir::Startup

// An address no bus slave answers at, for Kvasir::Fault::Inject (kvasir/Runtime/FaultInject.hpp): 0xE0100000 -
// 0xFFFFFFFF is "Reserved" (RM0430 Rev 8 2.2.2 Table 1), and "all the memory map areas that are not allocated to
// on-chip memories and peripherals are considered Reserved" (2.2.2). That such an access is a bus error is not said
// in words there (12.3.7 says it for the QUADSPI bank past FSIZE: "bus fault exception").
template<>
struct Kvasir::Port::UnmappedAddress<Kvasir::Tag::User> {
    static constexpr std::uintptr_t value = 0xF000'0000U;
};
