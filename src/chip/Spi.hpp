#pragma once

#include "ClockMath.hpp"
#include "Clocks.hpp"
#include "Instances.hpp"
#include "Io.hpp"
#include "Rcc.hpp"
#include "WaitBounds.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Io/Io.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/RCC.hpp"
#include "peripherals/SPI.hpp"

#include <cstdint>
#include <type_traits>

// SPI master on the STM32F413 (RM0430 Rev 8 chapter 29), polled, chip select by the caller (a GPIO).
// SCK = PCLK / 2^(BR+1) (29.7.1: BR 000 = /2 ... 111 = /256), at most what DS11581 Rev 7 Table 67
// allows for the instance, the board's supply and whether the master receives (Instances.hpp
// `Chip::spiMaxSckHz`: SPI2/3 25 MHz; SPI1/4/5 42 MHz receiving from 2.7 V, 25 MHz below, 50 MHz
// transmit-only).
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
    // The bus (RM0430 Rev 8 Table 1, Instances.hpp) and the register set whose row in Blocks.hpp has the enable bit, of each SPI.
    template<unsigned N>
    struct Instance;

    // clang-format off
    template<> struct Instance<1> : Rcc::OnBusOf<Peripheral::SPI::Registers<1>> {};
    template<> struct Instance<2> : Rcc::OnBusOf<Peripheral::SPI::Registers<2>> {};
    template<> struct Instance<3> : Rcc::OnBusOf<Peripheral::SPI::Registers<3>> {};
    template<> struct Instance<4> : Rcc::OnBusOf<Peripheral::SPI::Registers<4>> {};
    template<> struct Instance<5> : Rcc::OnBusOf<Peripheral::SPI::Registers<5>> {};
    // clang-format on

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

        static constexpr std::uint32_t pclkHz = Clocks::template pclkHz<I::bus>;
        // DS11581 Table 67 for this instance, the board's supply and direction (Instances.hpp): a
        // master without MISO is a master transmitter.
        static constexpr std::uint32_t maxSckHz
          = Chip::spiMaxSckHz(N, Clocks::supply, !std::is_same_v<MisoPin, Io::NotUsed<>>);

        // Startup: this block, and the bus clock its dividers are computed from (Clocks.hpp).
        using Provides               = brigand::list<Hal::Owns<Regs>>;
        using Claims                 = Hal::Claim<Kvasir::Clocks::ClkApb<I::bus>, pclkHz>;
        static constexpr unsigned br = baudBits(pclkHz, C.maxHz < maxSckHz ? C.maxHz : maxSckHz);
        static constexpr std::uint32_t sckHz = pclkHz >> (br + 1);
        static_assert(sckHz <= C.maxHz && sckHz <= maxSckHz,
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
                // it there before the SPI drives it (an SPI ADC that picks its clock source from
                // the SCK level at its power-on reset or a CS fall takes a high one for its
                // internal-clock mode).
                return list(
                  action(Io::Action::PinFunction<Af,
                                                 Io::OutputType::pushPull,
                                                 Io::OutputSpeed::veryHigh,
                                                 Io::OutputInit::low,
                                                 C.cpol ? Io::PullConfiguration::pullNone
                                                        : Io::PullConfiguration::pullDown>{},
                         p));
            } else {
                return list(action(Io::Action::PinFunction<Af>{}, p));
            }
        }

        static constexpr auto powerClockEnable = I::powerClockEnable;
        static constexpr auto initStepPinConfig
          = list(pin<true>(SckPin{}), pin<false>(MisoPin{}), pin<true>(MosiPin{}));
        // Master, NSS by software (SSM = SSI = 1: 29.3.5, else MODF drops it out of master mode).
        static constexpr auto initStepPeripheryConfig = list(
          set(Regs::CR1::mstr),
          set(Regs::CR1::ssm),
          set(Regs::CR1::ssi),
          write(Regs::CR1::br, Register::value<std::uint32_t, br>()),
          write(Regs::CR1::cpol, Register::value<std::uint32_t, C.cpol ? 1U : 0U>()),
          write(Regs::CR1::cpha, Register::value<std::uint32_t, C.cpha ? 1U : 0U>()),
          write(Regs::CR1::dff, Register::value<std::uint32_t, C.frameBits == 16 ? 1U : 0U>()));
        static constexpr auto initStepPeripheryEnable = list(set(Regs::CR1::spe));

    private:
        // Two frames at this SCK (WaitBounds.hpp). Under the injected policy: no caller recovers
        // from a stuck SPI today, so an application that injects Panic gets a panic here (the
        // default, Unbounded, is the plain loop).
        // The conditions call the flag readers rather than name the field: under Unbounded every
        // wait is then the old `while(!txEmpty()) {}` byte for byte (code an ISR may run); a
        // bounded policy's panic detail is 0, not SR's address.
        using FrameBound = Kvasir::Chip::SpiFrameBound<sckHz, C.frameBits>;

        [[nodiscard]] static bool txEmpty() { return apply(read(Regs::SR::txe)) != 0; }

        [[nodiscard]] static bool rxNotEmpty() { return apply(read(Regs::SR::rxne)) != 0; }

    public:
        [[nodiscard]] static bool busy() { return apply(read(Regs::SR::bsy)) != 0; }

        /// One frame out, the frame clocked in at the same time back.
        static Frame transfer(Frame out) {
            Register::waitUntil<FrameBound>([] { return txEmpty(); });
            apply(Register::write(Regs::DR::dr, static_cast<std::uint32_t>(out)));
            Register::waitUntil<FrameBound>([] { return rxNotEmpty(); });
            return static_cast<Frame>(apply(read(Regs::DR::dr)));
        }

        /// One frame out without waiting for anything but room in the transmit buffer (the
        /// received frame is dropped). Follow with waitIdle() before the chip select goes up.
        static void send(Frame out) {
            Register::waitUntil<FrameBound>([] { return txEmpty(); });
            apply(Register::write(Regs::DR::dr, static_cast<std::uint32_t>(out)));
        }

        /// Until the last frame has left the shift register: TXE = 1, then BSY = 0 (29.3.10, "the
        /// NSS signal is managed by an arbitrary GPIO toggle and the master has to provide proper
        /// end of NSS pulse for slave").
        static void waitIdle() {
            Register::waitUntil<FrameBound>([] { return txEmpty(); });
            Register::waitUntil<FrameBound>([] { return !busy(); });
        }
    };
}}   // namespace Kvasir::Spi
