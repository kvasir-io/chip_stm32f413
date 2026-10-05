#pragma once

#include "kvasir/Io/Io.hpp"
#include "kvasir/Mpl/Utility.hpp"
#include "kvasir/Register/Register.hpp"

#include <array>

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
        /// GPIOA; port n at + n * 0x400 (RM0430 2.2.2 Table 1 "STM32F413/423 register boundary
        /// addresses": GPIOA 0x4002 0000 ... GPIOH 0x4002 1C00).
        static constexpr unsigned baseAddress = 0x40020000;
        static constexpr unsigned portOffset  = 0x400;
        static constexpr int      portBegin   = 0;
        /// Ports A (0) to H (7).
        static constexpr int portEnd          = 8;
        static constexpr int pinBegin         = 0;
        static constexpr int pinEnd           = 16;
        static constexpr int ListEndIndicator = 255;
        /// What the LQFP64 package does not bond (DS11581 Rev 7 Figure "STM32F413xG/H LQFP64
        /// pinout", PDF page 44: PA0-15, PB0-10, PB12-15, PC0-15, PD2, PH0-1 - 50 I/Os). PB11
        /// is not there (its pad is VCAP_1), D has only PD2, E to G have none, H only PH0/PH1.
        static constexpr std::array<std::array<int, pinEnd - pinBegin + 1>, portEnd - portBegin>
          PinsDisabled{
            {{{ListEndIndicator}},
             {{11, ListEndIndicator}},
             {{ListEndIndicator}},
             {{0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, ListEndIndicator}},
             {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, ListEndIndicator}},
             {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, ListEndIndicator}},
             {{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, ListEndIndicator}},
             {{2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, ListEndIndicator}}}
        };
        /// The ports with at least one bonded pin, for the clock enable at boot.
        static constexpr std::array<int, 5> PortsBonded{0, 1, 2, 3, 7};
    };

    template<int Port,
             int Pin>
    constexpr bool isValidPinLocation() {
        using T = PinLocationTraits<void>;
        if(Port < T::portBegin || Port >= T::portEnd) { return false; }
        if(Pin < T::pinBegin || Pin >= T::pinEnd) { return false; }
        for(auto dp : T::PinsDisabled[static_cast<std::size_t>(Port)]) {
            if(dp == T::ListEndIndicator) { break; }
            if(dp == Pin) { return false; }
        }
        return true;
    }

    namespace Detail {
        using namespace Register;
        using Traits = PinLocationTraits<void>;

        template<int Port, unsigned Offset>
        constexpr unsigned portRegister
          = Traits::baseAddress + static_cast<unsigned>(Port) * Traits::portOffset + Offset;

        // RM0430 7.4.1-7.4.10: register offsets of a GPIO port.
        enum Offset : unsigned {
            moder   = 0x00,
            otyper  = 0x04,
            ospeedr = 0x08,
            pupdr   = 0x0C,
            idr     = 0x10,
            odr     = 0x14,
            bsrr    = 0x18,
            afrl    = 0x20,
            afrh    = 0x24,
        };

        // A field of Width bits per pin, written with a literal: read-modify-write of the port
        // register (the other pins' fields are kept).
        template<int Port, unsigned Reg, int Pin, unsigned Width, unsigned Value>
        using PinField = Register::Action<
          FieldLocation<Register::Address<portRegister<Port, Reg>>,
                        ((1U << Width) - 1U) << (static_cast<unsigned>(Pin) * Width),
                        ReadWriteAccess,
                        unsigned>,
          WriteLiteralAction<(Value << (static_cast<unsigned>(Pin) * Width))>>;

        // BSRR: write-only, a 0 bit does nothing - no read needed (7.4.7).
        template<int Port, int Bit>
        using Bsrr = Register::Action<
          WOBitLocT<Register::Address<portRegister<Port, bsrr>, maskFromRange(31, 0)>, Bit>,
          WriteLiteralAction<(1U << static_cast<unsigned>(Bit))>>;

        // MODER (7.4.1): 00 input, 01 general purpose output, 10 alternate function, 11 analog.
        enum Mode : unsigned { input = 0b00, output = 0b01, alternate = 0b10, analog = 0b11 };

        template<int Port, int Pin, Mode M>
        using Moder = PinField<Port, moder, Pin, 2, M>;

        // OTYPER (7.4.2): 0 push-pull, 1 open-drain.
        template<int Port, int Pin, OutputType OT>
        using Otyper = PinField<Port, otyper, Pin, 1, OT == OutputType::OpenDrain ? 1U : 0U>;

        // OSPEEDR (7.4.3): 00 low, 01 medium, 10 fast, 11 high.
        template<int Port, int Pin, OutputSpeed OS>
        using Ospeedr = PinField<Port,
                                 ospeedr,
                                 Pin,
                                 2,
                                 OS == OutputSpeed::Low      ? 0b00U
                                 : OS == OutputSpeed::Medium ? 0b01U
                                 : OS == OutputSpeed::High   ? 0b10U
                                                             : 0b11U>;

        // PUPDR (7.4.4): 00 none, 01 pull-up, 10 pull-down.
        template<int Port, int Pin, PullConfiguration PC>
        using Pupdr = PinField<Port,
                               pupdr,
                               Pin,
                               2,
                               PC == PullConfiguration::PullUp     ? 0b01U
                               : PC == PullConfiguration::PullDown ? 0b10U
                                                                   : 0b00U>;

        // AFRL (pins 0-7) / AFRH (8-15), 4 bits a pin (7.4.9, 7.4.10).
        template<int Port, int Pin, unsigned Af>
        using Afr = PinField < Port,
              Pin<8 ? afrl : afrh, Pin % 8, 4, Af>;

        // The output level a pin starts with, set before it becomes an output (no glitch).
        template<int Port, int Pin, OutputInit OI>
        using InitLevel = Bsrr<Port, OI == OutputInit::High ? Pin : Pin + 16>;
    }   // namespace Detail

    template<Io::PullConfiguration PC, int Port, int Pin>
    struct MakeAction<Action::Input<PC>, Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(Detail::Pupdr<Port, Pin, PC>{},
                           Detail::Moder<Port, Pin, Detail::input>{})) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<Io::OutputType OT, Io::OutputSpeed OS, Io::OutputInit OI, int Port, int Pin>
    struct MakeAction<Action::Output<OT, OS, OI>, Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(Detail::InitLevel<Port, Pin, OI>{},
                           Detail::Otyper<Port, Pin, OT>{},
                           Detail::Ospeedr<Port, Pin, OS>{},
                           Detail::Pupdr<Port, Pin, PullConfiguration::PullNone>{},
                           Detail::Moder<Port, Pin, Detail::output>{})) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    // The pin as an ADC/DAC input: analog mode, no pull (7.3.12: "the pull-up and pull-down
    // resistors are disabled", the Schmitt trigger is off).
    template<int Port, int Pin>
    struct MakeAction<Action::Analog, Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(Detail::Pupdr<Port, Pin, PullConfiguration::PullNone>{},
                           Detail::Moder<Port, Pin, Detail::analog>{})) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Clear, Register::PinLocation<Port, Pin>>
      : Detail::Bsrr<Port, Pin + 16> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    template<int Port, int Pin>
    struct MakeAction<Action::Set, Register::PinLocation<Port, Pin>> : Detail::Bsrr<Port, Pin> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    // No toggle register on this GPIO: ODR is read, xor-ed and written back. Not atomic against
    // an interrupt that writes the same port's ODR - use set/clear (BSRR) where that matters.
    template<int Port, int Pin>
    struct MakeAction<Action::Toggle, Register::PinLocation<Port, Pin>>
      : Register::Action<
          Register::RWBitLocT<Register::Address<Detail::portRegister<Port, Detail::odr>>, Pin>,
          Register::XorLiteralAction<(1U << static_cast<unsigned>(Pin))>> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
    };

    // IDR (7.4.5) holds the pad level in every mode but analog - also of a pin in
    // alternate-function mode (an SPI MISO read as "conversion ready").
    template<int Port, int Pin>
    struct MakeAction<Action::Read, Register::PinLocation<Port, Pin>>
      : Register::Action<
          Register::RWBitLocT<Register::Address<Detail::portRegister<Port, Detail::idr>>, Pin>,
          Register::ReadAction> {
        static_assert(isValidPinLocation<Port,
                                         Pin>(),
                      "invalid PinLocation");
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
    struct MakeAction<Action::PinFunction<Function, OT, OS, OI, PC>,
                      Register::PinLocation<Port, Pin>>
      : decltype(MPL::list(Detail::InitLevel<Port, Pin, OI>{},
                           Detail::Afr<Port, Pin, static_cast<unsigned>(Function)>{},
                           Detail::Otyper<Port, Pin, OT>{},
                           Detail::Ospeedr<Port, Pin, OS>{},
                           Detail::Pupdr<Port, Pin, PC>{},
                           Detail::Moder<Port, Pin, Detail::alternate>{})) {
        static_assert(isValidPinLocation<Port, Pin>(), "invalid PinLocation");
        static_assert(Function >= 0 && Function <= 15, "the alternate function is AF0 to AF15");
    };
}}   // namespace Kvasir::Io
