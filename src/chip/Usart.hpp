#pragma once

#include "Interrupt.hpp"
#include "Io.hpp"
#include "kvasir/Atomic/Queue.hpp"
#include "kvasir/Common/Interrupt.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Prescaler.hpp"
#include "peripherals/RCC.hpp"
#include "peripherals/USART.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <type_traits>

// USART1/2/3/6 of the STM32F413 (RM0430 Rev 8 chapter 28) as an interrupt-driven byte stream, 8N1,
// 16x oversampling. Baud = fCK / (16 x USARTDIV) and BRR holds USARTDIV x 16 (28.4.4), so BRR =
// fCK / baud, rounded; the error is checked at compile time.
//
//     using Debug = Kvasir::Usart::Port<HW::ClockTree, 1, 115'200, TxPin, RxPin, 7>;
//     Debug::send(bytes);                        // queued, returns at once (drops what does not fit)
//     while(auto b = Debug::receive()) { ... }   // what arrived
//
// The interrupt moves bytes between DR and two lock-free queues (Kvasir::Atomic::Queue, one
// producer and one consumer each). A receive overrun (ORE: a byte arrived while the last one was
// still in DR, 28.6.1) is cleared - SR read, then DR read - and counted.
namespace Kvasir { namespace Usart {
    enum class Bus { apb1, apb2 };

    namespace Detail { using RCC = Kvasir::Peripheral::RCC::Registers<>; }

    template<unsigned N>
    struct Instance;

    // RM0430 Table 1 (USART1/6 on APB2, USART2/3 on APB1), 6.3.15/6.3.16, Table 40.
    template<>
    struct Instance<1> {
        static constexpr Bus bus = Bus::apb2;
        using Irq                = std::remove_cvref_t<decltype(Interrupt::usart1)>;

        static constexpr auto enable() {
            return write(Detail::RCC::APB2ENR::usart1en, Register::value<std::uint32_t, 1>());
        }
    };

    template<>
    struct Instance<2> {
        static constexpr Bus bus = Bus::apb1;
        using Irq                = std::remove_cvref_t<decltype(Interrupt::usart2)>;

        static constexpr auto enable() {
            return write(Detail::RCC::APB1ENR::usart2en, Register::value<std::uint32_t, 1>());
        }
    };

    template<>
    struct Instance<3> {
        static constexpr Bus bus = Bus::apb1;
        using Irq                = std::remove_cvref_t<decltype(Interrupt::usart3)>;

        static constexpr auto enable() {
            return write(Detail::RCC::APB1ENR::usart3en, Register::value<std::uint32_t, 1>());
        }
    };

    template<>
    struct Instance<6> {
        static constexpr Bus bus = Bus::apb2;
        using Irq                = std::remove_cvref_t<decltype(Interrupt::usart6)>;

        static constexpr auto enable() {
            return write(Detail::RCC::APB2ENR::usart6en, Register::value<std::uint32_t, 1>());
        }
    };

    template<typename Clocks,
             unsigned      N,
             std::uint64_t Baud,
             typename TxPin,
             typename RxPin,
             int                  Af,
             std::size_t          TxSize   = 256,
             std::size_t          RxSize   = 256,
             unsigned             Priority = 2,
             Prescaler::Tolerance Tol      = Prescaler::Tolerance{std::ratio<1, 100>{}}>
    struct Port {
        using I    = Instance<N>;
        using Regs = Kvasir::Peripheral::USART::Registers<N>;
        using Irq  = typename I::Irq;

        static constexpr std::uint32_t fckHz
          = I::bus == Bus::apb1 ? Clocks::pclk1Hz : Clocks::pclk2Hz;
        // BRR: mantissa 12 bits, fraction 4 bits; at least 1.0 (16).
        static constexpr auto divider
          = Prescaler::fromRange(fckHz, Baud, {.first = 16, .last = 0xFFFF});
        static constexpr bool Ok = [] {
            Prescaler::assertInTolerance<divider.achieved, Baud, Tol, "USART baud rate">();
            return true;
        }();
        static_assert(Ok);
        static constexpr std::uint32_t brr = static_cast<std::uint32_t>(divider.setting);

        static constexpr auto powerClockEnable = list(I::enable());
        static constexpr auto initStepPinConfig
          = list(action(Io::Action::PinFunction<Af,
                                                Io::OutputType::PushPull,
                                                Io::OutputSpeed::High,
                                                Io::OutputInit::High,
                                                Io::PullConfiguration::PullUp>{},
                        TxPin{}),
                 action(Io::Action::PinFunction<Af,
                                                Io::OutputType::PushPull,
                                                Io::OutputSpeed::Low,
                                                Io::OutputInit::Low,
                                                Io::PullConfiguration::PullUp>{},
                        RxPin{}));
        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::BRR::FULLREGISTER, Register::value<std::uint32_t, brr>()),
                 write(Regs::CR1::te, Register::value<std::uint32_t, 1>()),
                 write(Regs::CR1::re, Register::value<std::uint32_t, 1>()),
                 write(Regs::CR1::rxneie, Register::value<std::uint32_t, 1>()));
        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<Priority>(Irq{}), Nvic::makeClearPending(Irq{}));
        static constexpr auto initStepPeripheryEnable
          = list(write(Regs::CR1::ue, Register::value<std::uint32_t, 1>()),
                 Nvic::makeEnable(Irq{}));

        static inline Atomic::Queue<std::byte, TxSize, Atomic::OverFlowPolicyIgnore> txQueue{};
        static inline Atomic::Queue<std::byte, RxSize, Atomic::OverFlowPolicyIgnore> rxQueue{};
        static inline std::uint32_t                                                  overruns{};
        static inline std::uint32_t                                                  rxDropped{};

        /// Queue bytes for sending; returns how many fit.
        static std::size_t send(std::span<std::byte const> bytes) {
            std::size_t n = 0;
            for(auto b : bytes) {
                if(txQueue.size() >= TxSize) { break; }
                txQueue.push(b);
                ++n;
            }
            if(n != 0) { apply(write(Regs::CR1::txeie, Register::value<std::uint32_t, 1>())); }
            return n;
        }

        /// The next received byte, if any.
        static std::optional<std::byte> receive() {
            std::byte b{};
            if(rxQueue.pop_into(b)) { return b; }
            return std::nullopt;
        }

        [[nodiscard]] static bool txIdle() {
            return txQueue.empty() && apply(read(Regs::SR::tc)) != 0;
        }

        static void onIsr() {
            // One SR load (28.6.1), then the DR read: that pair clears ORE.
            auto const sr   = apply(read(Regs::SR::rxne, Regs::SR::ore, Regs::SR::txe));
            bool const rxne = get<0>(sr) != 0;
            bool const ore  = get<1>(sr) != 0;
            bool const txe  = get<2>(sr) != 0;
            if(rxne || ore) {
                auto const b = static_cast<std::byte>(apply(read(Regs::DR::dr)) & 0xFFU);
                if(ore) { ++overruns; }   // SR read, then DR read: cleared
                if(rxQueue.size() < RxSize) {
                    rxQueue.push(b);
                } else {
                    ++rxDropped;
                }
            }
            if(txe && apply(read(Regs::CR1::txeie)) != 0) {
                std::byte b{};
                if(txQueue.pop_into(b)) {
                    apply(write(Regs::DR::dr, static_cast<std::uint32_t>(b)));
                } else {
                    apply(write(Regs::CR1::txeie, Register::value<std::uint32_t, 0>()));
                }
            }
        }

        static constexpr Nvic::Isr<std::addressof(onIsr), Irq> isr{};
    };
}}   // namespace Kvasir::Usart
