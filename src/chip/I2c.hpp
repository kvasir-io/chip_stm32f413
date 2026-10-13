#pragma once

#include "Io.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/I2C.hpp"
#include "peripherals/RCC.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

// I2C1/2/3 of the STM32F413 (RM0430 Rev 8 chapter 27) as a polled standard-mode master: a register
// write, and a register read with a repeated start. Every wait is bounded; a transfer that does not
// finish ends with a STOP (or, if the peripheral is stuck, a software reset - ES0372 2.11.3) and an
// error. Made for a sensor read once a second, from the main loop; the queued interrupt-driven
// policy for kvasir_devices' I2C engine is not written yet.
//
//     using Bus = Kvasir::I2c::Master<HW::ClockTree, 3, 50'000, SclPin, SdaPin, 4>;
//     std::array<std::byte, 1> t{};
//     if(Bus::readRegister(0x49, std::byte{0x00}, t) == Kvasir::I2c::Result::ok) { ... }
//
// Errata ES0372 Rev 8: 2.11.1 a spurious BERR in master mode is cleared and ignored; 2.11.4 the
// repeated-start setup time can be violated between 88 and 100 kHz in standard mode - the clock is
// held below 88 kHz.
namespace Kvasir { namespace I2c {
    enum class Result : std::uint8_t { ok, busy, nack, arbitrationLost, timeout };

    namespace Detail { using RCC = Kvasir::Peripheral::RCC::Registers<>; }

    template<unsigned N>
    struct Instance;

    template<>
    struct Instance<1> {
        static constexpr auto enable() {
            return write(Detail::RCC::APB1ENR::i2c1en, Register::value<std::uint32_t, 1>());
        }
    };

    template<>
    struct Instance<2> {
        static constexpr auto enable() {
            return write(Detail::RCC::APB1ENR::i2c2en, Register::value<std::uint32_t, 1>());
        }
    };

    template<>
    struct Instance<3> {
        static constexpr auto enable() {
            return write(Detail::RCC::APB1ENR::i2c3en, Register::value<std::uint32_t, 1>());
        }
    };

    template<typename Clocks,
             unsigned      N,
             std::uint32_t Hz,
             typename SclPin,
             typename SdaPin,
             int Af>
    struct Master {
        using I    = Instance<N>;
        using Regs = Kvasir::Peripheral::I2C::Registers<N>;

        static_assert(Hz < 88'000,
                      "standard mode below 88 kHz (errata ES0372 Rev 8 2.11.4)");
        static constexpr std::uint32_t pclkMHz = Clocks::pclk1Hz / 1'000'000;
        static_assert(Clocks::pclk1Hz % 1'000'000 == 0 && pclkMHz >= 2 && pclkMHz <= 50,
                      "CR2.FREQ is PCLK1 in whole MHz, 2 to 50 (27.6.2)");
        // Sm: Thigh = Tlow = CCR x TPCLK1 (27.6.8), so f = PCLK1 / (2 CCR); the next CCR up.
        static constexpr std::uint32_t ccr = (Clocks::pclk1Hz + 2 * Hz - 1) / (2 * Hz);
        static_assert(ccr >= 4 && ccr <= 0xFFF,
                      "CCR is 4 to 4095 in Sm mode (27.6.8)");
        // TRISE: the 1000 ns Sm maximum rise time in PCLK1 periods, plus one (27.6.9).
        static constexpr std::uint32_t trise = pclkMHz + 1;

        static constexpr auto powerClockEnable = list(I::enable());
        // Open drain, the board has the pull-ups.
        static constexpr auto initStepPinConfig
          = list(action(Io::Action::PinFunction<Af, Io::OutputType::OpenDrain>{}, SclPin{}),
                 action(Io::Action::PinFunction<Af, Io::OutputType::OpenDrain>{}, SdaPin{}));
        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::CR2::freq, Register::value<std::uint32_t, pclkMHz>()),
                 write(Regs::CCR::ccr, Register::value<std::uint32_t, ccr>()),
                 write(Regs::TRISE::trise, Register::value<std::uint32_t, trise>()));
        static constexpr auto initStepPeripheryEnable
          = list(write(Regs::CR1::pe, Register::value<std::uint32_t, 1>()));

        static inline std::uint32_t resets{};

        // BUSY is "set by hardware on detection of SDA or SCL low", cleared only by a STOP, and
        // "still updated when the interface is disabled (PE=0)" (RM0430 Rev 8 27.6.7): the pins
        // switching to their alternate function latch it, with both lines idle high -
        // every boot (SR2 0x0002 at the first transfer, PA8 and PC9 read 1), and the
        // first transfer then waited 40 ms for it and failed "busy". A software reset clears it
        // (CR1.SWRST, 27.6.1: "if the BUSY bit is set and remains locked due to a glitch on the
        // bus, the SWRST bit can be used to exit from this state"), once, after the pin setup.
        static void preEnableRuntimeInit() { softwareReset(); }

    private:
        static constexpr unsigned pollLimit
          = 200'000;   // several ms at 100 MHz: a byte at 50 kHz is 180 us

        static void stop() { apply(write(Regs::CR1::stop, Register::value<std::uint32_t, 1>())); }

        // Wait for the SR1 flag(s) `want` (27.6.6; a field's `.Mask`); nack/arbitration loss end it
        // early. BERR is cleared and ignored (ES0372 2.11.1). SR1 flags are rc_w0: writing 0 to one
        // leaves the others. One SR1 load a poll; one function for every flag.
        static Result waitFor(std::uint32_t want) {
            for(unsigned i = 0; i < pollLimit; ++i) {
                auto const s = Register::readWhole(Regs::SR1::FULLREGISTER);
                if(Register::extract(Regs::SR1::berr, s) != 0) {
                    apply(write(Regs::SR1::berr, Register::value<std::uint32_t, 0>()));
                }
                if(Register::extract(Regs::SR1::af, s) != 0) {
                    apply(write(Regs::SR1::af, Register::value<std::uint32_t, 0>()));
                    stop();
                    return Result::nack;
                }
                if(Register::extract(Regs::SR1::arlo, s) != 0) {
                    apply(write(Regs::SR1::arlo, Register::value<std::uint32_t, 0>()));
                    return Result::arbitrationLost;
                }
                if((s & want) != 0) { return Result::ok; }
            }
            return Result::timeout;
        }

        // ADDR is cleared by reading SR1 and then SR2 (27.6.6): the whole registers, read for
        // their side effect, in this order.
        static void clearAddr() {
            static_cast<void>(Register::readWhole(Regs::SR1::FULLREGISTER));
            static_cast<void>(Register::readWhole(Regs::SR2::FULLREGISTER));
        }

        static Result start(std::uint8_t address7,
                            bool         readDirection) {
            apply(write(Regs::CR1::start, Register::value<std::uint32_t, 1>()));
            if(auto r = waitFor(Regs::SR1::sb.Mask); r != Result::ok) { return r; }
            apply(write(Regs::DR::dr,
                        (static_cast<std::uint32_t>(address7) << 1U) | (readDirection ? 1U : 0U)));
            return waitFor(Regs::SR1::addr.Mask);
        }

        // SWRST puts the I2C "under reset state" (27.6.1): the configuration is written again
        // afterwards; PE is left to the caller (Startup's enable step, or recover()).
        static void softwareReset() {
            apply(write(Regs::CR1::swrst, Register::value<std::uint32_t, 1>()));
            apply(write(Regs::CR1::swrst, Register::value<std::uint32_t, 0>()));
            apply(initStepPeripheryConfig);
        }

        // A transfer that timed out: STOP, and if the peripheral still holds the bus, a software
        // reset with the configuration written again (ES0372 2.11.3 workaround).
        static Result recover(Result r) {
            stop();
            for(unsigned i = 0; i < pollLimit; ++i) {
                if(apply(read(Regs::SR2::busy)) == 0) { return r; }
            }
            ++resets;
            softwareReset();
            apply(initStepPeripheryEnable);
            return r;
        }

        static Result finish(Result r) { return r == Result::timeout ? recover(r) : r; }

        static Result waitNotBusy() {
            for(unsigned i = 0; i < pollLimit; ++i) {
                if(apply(read(Regs::SR2::busy)) == 0) { return Result::ok; }
            }
            return recover(Result::busy);
        }

    public:
        /// START, address+W, reg, then the bytes, STOP.
        static Result writeRegister(std::uint8_t               address7,
                                    std::byte                  reg,
                                    std::span<std::byte const> bytes) {
            if(auto r = waitNotBusy(); r != Result::ok) { return r; }
            if(auto r = start(address7, false); r != Result::ok) { return finish(r); }
            clearAddr();
            apply(write(Regs::DR::dr, static_cast<std::uint32_t>(reg)));
            for(auto b : bytes) {
                if(auto r = waitFor(Regs::SR1::txe.Mask); r != Result::ok) { return finish(r); }
                apply(write(Regs::DR::dr, static_cast<std::uint32_t>(b)));
            }
            if(auto r = waitFor(Regs::SR1::btf.Mask); r != Result::ok) { return finish(r); }
            stop();
            return Result::ok;
        }

        /// START, address+W, reg, repeated START, address+R, one or two bytes, STOP - the
        /// SMBus "read byte"/"read word". The one-byte case follows 27.3.3: ACK cleared during EV6
        /// (before ADDR is cleared), STOP right after; two bytes use POS (the "N=2" sequence).
        static Result readRegister(std::uint8_t         address7,
                                   std::byte            reg,
                                   std::span<std::byte> out) {
            if(out.empty() || out.size() > 2) { return Result::busy; }
            if(auto r = waitNotBusy(); r != Result::ok) { return r; }
            if(auto r = start(address7, false); r != Result::ok) { return finish(r); }
            clearAddr();
            apply(write(Regs::DR::dr, static_cast<std::uint32_t>(reg)));
            if(auto r = waitFor(Regs::SR1::btf.Mask); r != Result::ok) { return finish(r); }
            if(auto r = start(address7, true); r != Result::ok) { return finish(r); }
            if(out.size() == 1) {
                apply(write(Regs::CR1::ack, Register::value<std::uint32_t, 0>()));
                clearAddr();
                stop();
                if(auto r = waitFor(Regs::SR1::rxne.Mask); r != Result::ok) { return finish(r); }
                out[0] = static_cast<std::byte>(apply(read(Regs::DR::dr)) & 0xFFU);
            } else {
                apply(write(Regs::CR1::pos, Register::value<std::uint32_t, 1>()));
                apply(write(Regs::CR1::ack, Register::value<std::uint32_t, 0>()));
                clearAddr();
                if(auto r = waitFor(Regs::SR1::btf.Mask); r != Result::ok) { return finish(r); }
                stop();
                out[0] = static_cast<std::byte>(apply(read(Regs::DR::dr)) & 0xFFU);
                out[1] = static_cast<std::byte>(apply(read(Regs::DR::dr)) & 0xFFU);
                apply(write(Regs::CR1::pos, Register::value<std::uint32_t, 0>()));
            }
            return Result::ok;
        }
    };
}}   // namespace Kvasir::I2c
