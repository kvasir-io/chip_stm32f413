#pragma once

#include "OdrToggle.hpp"
#include "kvasir/Hal/Pins.hpp"
#include "kvasir/Io/Claims.hpp"
#include "kvasir/Io/Io.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "peripherals/GPIO.hpp"
#include "peripherals/GPIOA.hpp"
#include "peripherals/GPIOB.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <type_traits>
#include <utility>

// GPIO of the STM32F413 (RM0430 Rev 8 chapter 7, "General-purpose I/Os"). A pin is configured by
// four 2-bit or 1-bit fields of its port - MODER (input/output/alternate/analog), OTYPER
// (push-pull/open-drain), OSPEEDR, PUPDR - and, for an alternate function, its 4-bit AFRL/AFRH
// field. Outputs are set and cleared through BSRR, which has no read-modify-write: bits 0-15 set,
// 16-31 reset, a 0 does nothing (7.4.7).
//
// The ports need their clock (RCC AHB1ENR, 6.3.11) before any of this reaches them: StartUp.hpp
// opens every bonded port first thing at boot, so a pin action never has to.
namespace Kvasir { namespace Io {
    template<typename>
    struct PinLocationTraits {
        /// The port layout: OdrToggle.hpp (Stm32::Gpio).
        static constexpr unsigned baseAddress = Stm32::Gpio::baseAddress;
        static constexpr unsigned portOffset  = Stm32::Gpio::portOffset;
        static constexpr int      portBegin   = 0;
        static constexpr int      portEnd     = Stm32::Gpio::portCount;
        static constexpr int      pinBegin    = 0;
        static constexpr int      pinEnd      = 16;
        /// What the LQFP64 package bonds, one bit per pin (DS11581 Rev 7 Figure "STM32F413xG/H LQFP64
        /// pinout", PDF page 44: PA0-15, PB0-10, PB12-15, PC0-15, PD2, PH0-1 - 50 I/Os). PB11
        /// is not there (its pad is VCAP_1), D has only PD2, E to G have none, H only PH0/PH1.
        static constexpr std::array<std::uint64_t, portEnd - portBegin>
          bonded{0xFFFF, 0xF7FF, 0xFFFF, 0x0004, 0x0000, 0x0000, 0x0000, 0x0003};
        /// The ports with at least one bonded pin, for the clock enable at boot.
        static constexpr auto portsBonded = [] {
            std::array<int, 5> ports{};
            std::size_t        n = 0;
            for(int p = portBegin; p < portEnd; ++p) {
                if(bonded[static_cast<std::size_t>(p - portBegin)] != 0) { ports.at(n++) = p; }
            }
            return ports;
        }();
    };

    template<int Port,
             int Pin>
    constexpr bool isValidPinLocation() {
        return Hal::Pins::isBonded<PinLocationTraits<void>>(Port, Pin);
    }

    namespace Detail {
        using namespace Register;
        using Traits = PinLocationTraits<void>;

        // The generated register set of a port: GPIOA and GPIOB have their own (other reset values, RM0430 7.4.1),
        // C to H are instances of GPIO (RM0430 2.2.2 Table 1).
        template<int Port>
        struct PortRegs {
            using type = Kvasir::Peripheral::GPIO::Registers<static_cast<unsigned>(Port)>;
        };

        template<>
        struct PortRegs<0> {
            using type = Kvasir::Peripheral::GPIOA::Registers<>;
        };

        template<>
        struct PortRegs<1> {
            using type = Kvasir::Peripheral::GPIOB::Registers<>;
        };

        template<int Port>
        using Regs = typename PortRegs<Port>::type;

        // the layout constants OdrToggle.hpp keeps (it is read in the middle of the register layer and cannot include
        // a generated header) against the generated bases
        template<int... Ports>
        consteval bool layoutMatches(std::integer_sequence<int,
                                                           Ports...>) {
            return (
              (Regs<Ports>::baseAddr
                 == Stm32::Gpio::baseAddress
                      + static_cast<unsigned>(Ports) * Stm32::Gpio::portOffset
               && Regs<Ports>::ODR::Addr::value == Regs<Ports>::baseAddr + Stm32::Gpio::odrOffset
               && Regs<Ports>::BSRR::Addr::value == Regs<Ports>::baseAddr + Stm32::Gpio::bsrrOffset)
              && ...);
        }

        static_assert(layoutMatches(std::make_integer_sequence<int,
                                                               Stm32::Gpio::portCount>{}),
                      "OdrToggle.hpp's GPIO layout differs from the generated GPIO registers");

        // The pin's own field of a per-pin register: the generated field holding bit Width * Pin (MODER0..15,
        // OT0..15, ...), written with a literal - a read-modify-write of the port register (the other pins' fields
        // are kept).
        template<typename R, int Pin, unsigned Width, unsigned Value>
        using PinField
          = decltype(Register::write(Register::fieldAt<static_cast<unsigned>(Pin) * Width>(R{}),
                                     Register::value<std::uint32_t, Value>()));

        // BSRR: write-only, a 0 bit does nothing - no read needed (7.4.7): BSn at bit n, BRn at n + 16.
        template<int Port, int Bit>
        using Bsrr = decltype(Register::set(
          Register::fieldAt<static_cast<unsigned>(Bit)>(typename Regs<Port>::BSRR{})));

        // MODER (7.4.1): 00 input, 01 general purpose output, 10 alternate function, 11 analog.
        // Plain enum: PinField's `unsigned Value`.
        enum Mode : unsigned { input = 0b00, output = 0b01, alternate = 0b10, analog = 0b11 };

        template<int Port, int Pin, Mode M>
        using Moder = PinField<typename Regs<Port>::MODER, Pin, 2, M>;

        // OTYPER (7.4.2): 0 push-pull, 1 open-drain.
        template<int Port, int Pin, OutputType OT>
        using Otyper
          = PinField<typename Regs<Port>::OTYPER, Pin, 1, OT == OutputType::openDrain ? 1U : 0U>;

        // OSPEEDR (7.4.3): 00 low, 01 medium, 10 fast, 11 high.
        template<int Port, int Pin, OutputSpeed OS>
        using Ospeedr = PinField<typename Regs<Port>::OSPEEDR,
                                 Pin,
                                 2,
                                 OS == OutputSpeed::low      ? 0b00U
                                 : OS == OutputSpeed::medium ? 0b01U
                                 : OS == OutputSpeed::high   ? 0b10U
                                                             : 0b11U>;

        // PUPDR (7.4.4): 00 none, 01 pull-up, 10 pull-down.
        template<int Port, int Pin, PullConfiguration PC>
        using Pupdr = PinField<typename Regs<Port>::PUPDR,
                               Pin,
                               2,
                               PC == PullConfiguration::pullUp     ? 0b01U
                               : PC == PullConfiguration::pullDown ? 0b10U
                                                                   : 0b00U>;

        // AFRL (pins 0-7) / AFRH (8-15), 4 bits a pin (7.4.9, 7.4.10).
        template<int Port, int Pin, unsigned Af>
        using Afr = PinField<
          std::conditional_t<(Pin < 8), typename Regs<Port>::AFRL, typename Regs<Port>::AFRH>,
          Pin % 8,
          4,
          Af>;

        // The output level a pin starts with, set before it becomes an output (no glitch).
        template<int Port, int Pin, OutputInit OI>
        using InitLevel = Bsrr<Port, OI == OutputInit::high ? Pin : Pin + 16>;

        // MODER's offset in a port's block (7.4.1), from the generated registers: the same in every port
        inline constexpr unsigned moder = Regs<0>::MODER::Addr::value - Regs<0>::baseAddr;

        template<int... Ports>
        consteval bool moderOffsetMatches(std::integer_sequence<int,
                                                                Ports...>) {
            return ((Regs<Ports>::MODER::Addr::value == Regs<Ports>::baseAddr + moder) && ...);
        }

        static_assert(moderOffsetMatches(std::make_integer_sequence<int,
                                                                    Stm32::Gpio::portCount>{}),
                      "MODER sits at another offset in some port's block");
    }   // namespace Detail

    template<Io::PullConfiguration PC, int Port, int Pin>
    struct MakeAction<Action::Input<PC>, PinLocation<Port, Pin>>
      : decltype(MPL::list(Detail::Pupdr<Port, Pin, PC>{},
                           Detail::Moder<Port, Pin, Detail::input>{})) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<Io::OutputType OT, Io::OutputSpeed OS, Io::OutputInit OI, int Port, int Pin>
    struct MakeAction<Action::Output<OT, OS, OI>, PinLocation<Port, Pin>>
      : decltype(MPL::list(Detail::InitLevel<Port, Pin, OI>{},
                           Detail::Otyper<Port, Pin, OT>{},
                           Detail::Ospeedr<Port, Pin, OS>{},
                           Detail::Pupdr<Port, Pin, PullConfiguration::pullNone>{},
                           Detail::Moder<Port, Pin, Detail::output>{})) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    // The pin as an ADC/DAC input: analog mode, no pull (7.3.12: "the pull-up and pull-down
    // resistors are disabled", the Schmitt trigger is off).
    template<int Port, int Pin>
    struct MakeAction<Action::Analog, PinLocation<Port, Pin>>
      : decltype(MPL::list(Detail::Pupdr<Port, Pin, PullConfiguration::pullNone>{},
                           Detail::Moder<Port, Pin, Detail::analog>{})) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Clear, PinLocation<Port, Pin>> : Detail::Bsrr<Port, Pin + 16> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Set, PinLocation<Port, Pin>> : Detail::Bsrr<Port, Pin> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    // No toggle register on this GPIO: ODR is read, the toggled pins are set or reset through BSRR
    // (OdrToggle.hpp, RM0430 7.3.5): another pin's concurrent write is kept, the same pin's is not.
    // Still an XorLiteralAction on ODR, so toggles of one port merge into one read and one write.
    template<int Port, int Pin>
    struct MakeAction<Action::Toggle, PinLocation<Port, Pin>>
      : decltype(Register::toggle(
          Register::fieldAt<static_cast<unsigned>(Pin)>(typename Detail::Regs<Port>::ODR{}))) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    // IDR (7.4.5) holds the pad level in every mode but analog - also of a pin in
    // alternate-function mode (an SPI MISO read as "conversion ready").
    template<int Port, int Pin>
    struct MakeAction<Action::Read, PinLocation<Port, Pin>>
      : decltype(Register::read(Register::bit<0, bool>(
          Register::fieldAt<static_cast<unsigned>(Pin)>(typename Detail::Regs<Port>::IDR{})))) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    /// Function is the alternate function number AF0..AF15 of DS11581 Rev 7 Table 12
    /// "STM32F413xG/H alternate functions" - the board header cites the row it took it from.
    template<Io::OutputType    OT,
             Io::OutputSpeed   OS,
             Io::OutputInit    OI,
             PullConfiguration PC,
             int               Port,
             int               Pin,
             int               Function>
    struct MakeAction<Action::PinFunction<Function, OT, OS, OI, PC>, PinLocation<Port, Pin>>
      : decltype(MPL::list(Detail::InitLevel<Port, Pin, OI>{},
                           Detail::Afr<Port, Pin, static_cast<unsigned>(Function)>{},
                           Detail::Otyper<Port, Pin, OT>{},
                           Detail::Ospeedr<Port, Pin, OS>{},
                           Detail::Pupdr<Port, Pin, PC>{},
                           Detail::Moder<Port, Pin, Detail::alternate>{})) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
        static_assert(Function >= 0 && Function <= 15, "the alternate function is AF0 to AF15");
    };

    /// The pin list of a TX/RX pair on alternate function `Af` (USART, CAN): TX push-pull at high speed with its
    /// output latch high, RX at low speed with its latch low, both pulled up (an idle line is high).
    template<int Af,
             typename TxPin,
             typename RxPin>
    constexpr auto txRxPins() {
        return MPL::list(Io::action(Action::PinFunction<Af,
                                                        OutputType::pushPull,
                                                        OutputSpeed::high,
                                                        OutputInit::high,
                                                        PullConfiguration::pullUp>{},
                                    TxPin{}),
                         Io::action(Action::PinFunction<Af,
                                                        OutputType::pushPull,
                                                        OutputSpeed::low,
                                                        OutputInit::low,
                                                        PullConfiguration::pullUp>{},
                                    RxPin{}));
    }
}}   // namespace Kvasir::Io

// The pins a peripheral configures, derived from what it writes: every pin action above ends in
// the pin's MODER field (RM0430 Rev 8 7.4.1, one 2-bit field per pin at bits 2n+1:2n of the port's
// register at offset 0x00), so a MODER write is the peripheral taking that pin. Startup collects
// these as provided resources: two entries that configure one pin are a build error naming
// "Io pin <port>.<pin>" (kvasir/StartUp/Resources.hpp).
namespace Kvasir { namespace Io { namespace Detail {
    // The port whose MODER is at `address`, else -1.
    consteval int moderPort(unsigned address) {
        if(address < Traits::baseAddress) { return -1; }
        unsigned const offset = address - Traits::baseAddress;
        if(offset % Traits::portOffset != moder) { return -1; }
        auto const port = static_cast<int>(offset / Traits::portOffset);
        return port >= Traits::portBegin && port < Traits::portEnd ? port : -1;
    }

    // The pin whose MODER field `mask` is (0b11 << 2n), else -1.
    consteval int moderPin(unsigned mask) {
        for(int pin = Traits::pinBegin; pin < Traits::pinEnd; ++pin) {
            if(mask == 0b11U << (2 * static_cast<unsigned>(pin))) { return pin; }
        }
        return -1;
    }

    static_assert(moderPin(0b11U << 8) == 4);
    static_assert(moderPin(0b1U << 8) == -1);
    static_assert(moderPin(0b11U << 30) == 15);
    static_assert(moderPort(0x40020400) == 1);
    static_assert(moderPort(0x40021C00) == 7);
    static_assert(moderPort(0x40020404) == -1);   // OTYPER of port B
    static_assert(moderPort(0x40022000) == -1);   // past GPIOH
    static_assert(moderPort(0x40013000) == -1);   // SPI1
}}}   // namespace Kvasir::Io::Detail

namespace Kvasir { namespace Startup {
    template<unsigned Addr,
             unsigned Z,
             unsigned O,
             typename RegType,
             typename Mode,
             unsigned Mask,
             typename Access,
             typename FieldType,
             typename TAction>
        requires(Io::Detail::moderPort(Addr) >= 0 && Io::Detail::moderPin(Mask) >= 0)
    struct ResourceOfAction<Register::Action<
      Register::
        FieldLocation<Register::Address<Addr, Z, O, RegType, Mode>, Mask, Access, FieldType>,
      TAction>> {
        using type
          = brigand::list<Io::PinResource<Io::Detail::moderPort(Addr), Io::Detail::moderPin(Mask)>>;
    };
}}   // namespace Kvasir::Startup
