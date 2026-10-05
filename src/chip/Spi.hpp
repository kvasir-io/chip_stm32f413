#pragma once

#include "ClockMath.hpp"
#include "Io.hpp"
#include "kvasir/Io/Io.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/RCC.hpp"
#include "peripherals/SPI.hpp"

#include <cstdint>
#include <type_traits>

// SPI master on the STM32F413 (RM0430 Rev 8 chapter 29), polled, chip select by the caller (a GPIO).
// SCK = PCLK / 2^(BR+1) (29.7.1: BR 000 = /2 ... 111 = /256); SPI1/4/5 on APB2 reach 50 MHz,
// SPI2/3 on APB1 25 MHz (DS11581 Rev 7 Table 67 "SPI dynamic characteristics", 2.7-3.6 V: SPI1,4,5
// master 42 MHz from 2.7 V, 50 MHz from 3.0 V; SPI2/3 25 MHz).
//
//     using DacBus = Kvasir::Spi::Master<HW::ClockTree, 2, {25'000'000, 16}, SckPin, Kvasir::Io::NotUsed<>, MosiPin, 5>;
//     DacBus::send(code); DacBus::waitIdle(); apply(set(csPin));
//
// The bus is always full duplex (BIDIMODE = 0, RXONLY = 0): a receive-only master clocks for as
// long as SPE is set (29.3.9 "receive-only mode"), so reads send 0xFF on a MOSI nobody listens to.
// A write-only user never reads DR, so OVR sets and stays set: an overrun is a reception condition
// only (29.3.13 "Overrun flag"); transfer() drains a stale frame first, and the DR read followed
// by the SR read clears OVR.
namespace Kvasir { namespace Spi {
    enum class Bus { apb1, apb2 };

    namespace Detail { using RCC = Kvasir::Peripheral::RCC::Registers<>; }

    template<unsigned N>
    struct Instance;

    // RM0430 Table 1 (boundary addresses: SPI2/3 on APB1, SPI1/4/5 on APB2), 6.3.15/6.3.16
    // (enable bits); DS11581 Table 67 (maximum SCK, 2.7-3.6 V).
    template<>
    struct Instance<1> {
        static constexpr Bus           bus   = Bus::apb2;
        static constexpr std::uint32_t maxHz = 42'000'000;

        static constexpr auto enable() {
            return write(Detail::RCC::APB2ENR::spi1en, Register::value<std::uint32_t, 1>());
        }
    };

    template<>
    struct Instance<2> {
        static constexpr Bus           bus   = Bus::apb1;
        static constexpr std::uint32_t maxHz = 25'000'000;

        static constexpr auto enable() {
            return write(Detail::RCC::APB1ENR::spi2en, Register::value<std::uint32_t, 1>());
        }
    };

    template<>
    struct Instance<3> {
        static constexpr Bus           bus   = Bus::apb1;
        static constexpr std::uint32_t maxHz = 25'000'000;

        static constexpr auto enable() {
            return write(Detail::RCC::APB1ENR::spi3en, Register::value<std::uint32_t, 1>());
        }
    };

    template<>
    struct Instance<4> {
        static constexpr Bus           bus   = Bus::apb2;
        static constexpr std::uint32_t maxHz = 42'000'000;

        static constexpr auto enable() {
            return write(Detail::RCC::APB2ENR::spi4en, Register::value<std::uint32_t, 1>());
        }
    };

    template<>
    struct Instance<5> {
        static constexpr Bus           bus   = Bus::apb2;
        static constexpr std::uint32_t maxHz = 42'000'000;

        static constexpr auto enable() {
            return write(Detail::RCC::APB2ENR::spi5en, Register::value<std::uint32_t, 1>());
        }
    };

    struct Config {
        std::uint32_t maxHz;   // the fastest SCK the part allows; the bus takes the next lower
        unsigned      frameBits = 8;   // 8 or 16 (CR1.DFF)
        bool          cpol      = false;
        bool          cpha      = false;
    };

    template<typename Clocks,
             unsigned N,
             Config   C,
             typename SckPin,
             typename MisoPin,
             typename MosiPin,
             int Af>
    struct Master {
        using I    = Instance<N>;
        using Regs = Kvasir::Peripheral::SPI::Registers<N>;
        static_assert(C.frameBits == 8 || C.frameBits == 16,
                      "SPI frames are 8 or 16 bits (CR1.DFF)");

        static constexpr std::uint32_t pclkHz
          = I::bus == Bus::apb1 ? Clocks::pclk1Hz : Clocks::pclk2Hz;
        static constexpr unsigned br = baudBits(pclkHz, C.maxHz < I::maxHz ? C.maxHz : I::maxHz);
        static constexpr std::uint32_t sckHz = pclkHz >> (br + 1);
        static_assert(sckHz <= C.maxHz && sckHz <= I::maxHz,
                      "no BR gets SCK low enough");
        // Errata ES0372 Rev 8 2.14.4 "Corrupted last bit of data ... received in Master mode with
        // delayed SCK feedback", Table 5: a receiving master needs APB <= 84 MHz with the SCK pad at
        // high/very high speed (it is, below). A write-only bus does not receive.
        static_assert(
          std::is_same_v<MisoPin,
                         Io::NotUsed<>>
            || pclkHz <= 84'000'000,
          "SPI receive above 84 MHz APB loses the last bit (errata ES0372 2.14.4): hold "
          "APB2 lower (Rcc::ClockTree Apb2MaxHz)");

        using Frame = std::conditional_t<C.frameBits == 16, std::uint16_t, std::uint8_t>;

        template<bool Output>
        static constexpr auto pin(auto p) {
            if constexpr(std::is_same_v<decltype(p), Io::NotUsed<>>) {
                return brigand::list<>{};
            } else if constexpr(Output) {
                // Very high speed (errata 2.14.4); SCK idles low in mode 0/1 and a pull-down keeps
                // it there before the SPI drives it (an LTC2401 takes a high SCK at its power-on
                // reset or a CS fall for "internal SCK mode").
                return list(
                  action(Io::Action::PinFunction<Af,
                                                 Io::OutputType::PushPull,
                                                 Io::OutputSpeed::VeryHigh,
                                                 Io::OutputInit::Low,
                                                 C.cpol ? Io::PullConfiguration::PullNone
                                                        : Io::PullConfiguration::PullDown>{},
                         p));
            } else {
                return list(action(Io::Action::PinFunction<Af>{}, p));
            }
        }

        static constexpr auto powerClockEnable = list(I::enable());
        static constexpr auto initStepPinConfig
          = list(pin<true>(SckPin{}), pin<false>(MisoPin{}), pin<true>(MosiPin{}));
        // Master, NSS by software (SSM = SSI = 1: 29.3.5, else MODF drops it out of master mode).
        static constexpr auto initStepPeripheryConfig = list(
          write(Regs::CR1::mstr, Register::value<std::uint32_t, 1>()),
          write(Regs::CR1::ssm, Register::value<std::uint32_t, 1>()),
          write(Regs::CR1::ssi, Register::value<std::uint32_t, 1>()),
          write(Regs::CR1::br, Register::value<std::uint32_t, br>()),
          write(Regs::CR1::cpol, Register::value<std::uint32_t, C.cpol ? 1U : 0U>()),
          write(Regs::CR1::cpha, Register::value<std::uint32_t, C.cpha ? 1U : 0U>()),
          write(Regs::CR1::dff, Register::value<std::uint32_t, C.frameBits == 16 ? 1U : 0U>()));
        static constexpr auto initStepPeripheryEnable
          = list(write(Regs::CR1::spe, Register::value<std::uint32_t, 1>()));

        [[nodiscard]] static bool txEmpty() { return apply(read(Regs::SR::txe)) != 0; }

        [[nodiscard]] static bool rxNotEmpty() { return apply(read(Regs::SR::rxne)) != 0; }

        [[nodiscard]] static bool busy() { return apply(read(Regs::SR::bsy)) != 0; }

        /// One frame out, the frame clocked in at the same time back.
        static Frame transfer(Frame out) {
            while(!txEmpty()) {}
            apply(Register::write(Regs::DR::dr, static_cast<std::uint32_t>(out)));
            while(!rxNotEmpty()) {}
            return static_cast<Frame>(apply(read(Regs::DR::dr)));
        }

        /// One frame out without waiting for anything but room in the transmit buffer (the
        /// received frame is dropped). Follow with waitIdle() before the chip select goes up.
        static void send(Frame out) {
            while(!txEmpty()) {}
            apply(Register::write(Regs::DR::dr, static_cast<std::uint32_t>(out)));
        }

        /// Until the last frame has left the shift register: TXE = 1, then BSY = 0 (29.3.10, "the
        /// NSS signal is managed by an arbitrary GPIO toggle and the master has to provide proper
        /// end of NSS pulse for slave").
        static void waitIdle() {
            while(!txEmpty()) {}
            while(busy()) {}
        }
    };
}}   // namespace Kvasir::Spi
