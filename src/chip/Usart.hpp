#pragma once

#include "Clocks.hpp"
#include "Instances.hpp"
#include "Interrupt.hpp"
#include "Io.hpp"
#include "Rcc.hpp"
#include "UsartQueue.hpp"
#include "kvasir/Common/Interrupt.hpp"
#include "kvasir/Hal/Owns.hpp"
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
//     std::optional<std::byte> b;
//     while(Debug::receive(b)) { ... }           // what arrived
//
// The interrupt moves bytes between DR and two lock-free queues (UsartQueue.hpp: Atomic::Fifo, one
// producer and one consumer each) that hold TxSize / RxSize bytes. A receive overrun (ORE: a byte arrived while the last one was
// still in DR, 28.6.1) is cleared - SR read, then DR read - and counted.
namespace Kvasir { namespace Usart {
    // The bus (RM0430 Rev 8 Table 1, Instances.hpp), the register set (its enable bit: Blocks.hpp) and the vector
    // (Table 40) of each USART.
    template<unsigned N>
    struct Instance;

    // clang-format off
    template<> struct Instance<1> : Rcc::OnBusOf<Peripheral::USART::Registers<1>> {
        using Irq = std::remove_cvref_t<decltype(Interrupt::usart1)>;
    };
    template<> struct Instance<2> : Rcc::OnBusOf<Peripheral::USART::Registers<2>> {
        using Irq = std::remove_cvref_t<decltype(Interrupt::usart2)>;
    };
    template<> struct Instance<3> : Rcc::OnBusOf<Peripheral::USART::Registers<3>> {
        using Irq = std::remove_cvref_t<decltype(Interrupt::usart3)>;
    };
    template<> struct Instance<6> : Rcc::OnBusOf<Peripheral::USART::Registers<6>> {
        using Irq = std::remove_cvref_t<decltype(Interrupt::usart6)>;
    };
    // clang-format on

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

        static constexpr std::uint32_t fckHz = Clocks::template pclkHz<I::bus>;

        // Startup: this block, and the bus clock its dividers are computed from (Clocks.hpp).
        using Provides = brigand::list<Hal::Owns<Regs>>;
        using Claims   = Hal::Claim<Kvasir::Clocks::ClkApb<I::bus>, fckHz>;
        // BRR: mantissa 12 bits, fraction 4 bits; at least 1.0 (16).
        static constexpr auto divider
          = Prescaler::fromRange(fckHz, Baud, {.first = 16, .last = 0xFFFF});
        static_assert(Chip::checkedInTolerance<divider.achieved,
                                               Baud,
                                               Tol,
                                               "USART baud rate">);
        static constexpr std::uint32_t brr = static_cast<std::uint32_t>(divider.setting);

        static constexpr auto powerClockEnable  = I::powerClockEnable;
        static constexpr auto initStepPinConfig = Io::txRxPins<Af, TxPin, RxPin>();
        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::BRR::FULLREGISTER, Register::value<std::uint32_t, brr>()),
                 set(Regs::CR1::te),
                 set(Regs::CR1::re),
                 set(Regs::CR1::rxneie));
        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<Priority>(Irq{}), Nvic::makeClearPending(Irq{}));
        static constexpr auto initStepPeripheryEnable
          = list(set(Regs::CR1::ue), Nvic::makeEnable(Irq{}));

        static inline detail::ByteQueue<TxSize> txQueue{};   // holds TxSize bytes
        static inline detail::ByteQueue<RxSize> rxQueue{};   // holds RxSize bytes
        static inline std::uint32_t             overruns{};
        static inline std::uint32_t             rxDropped{};

        /// Queue bytes for sending; returns how many fit. One range push of what fits: the ISR only
        /// takes from the queue meanwhile, so the room it had stays.
        static std::size_t send(std::span<std::byte const> bytes) {
            std::size_t const n = detail::pushWhatFits(txQueue, bytes);
            if(n == 0) { return 0; }
            apply(set(Regs::CR1::txeie));
            return n;
        }

        /// The next received byte, as Hal::Uart::ByteSource (kvasir/Hal/Uart.hpp): true and the byte
        /// in `out` when one was waiting. No nullopt mark here: an overrun is counted (`overruns`),
        /// not put in the stream.
        static bool receive(std::optional<std::byte>& out) {
            std::byte b{};
            if(!rxQueue.popInto(b)) { return false; }
            out = b;
            return true;
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
                (void)detail::pushOrCount(rxQueue, b, rxDropped);
            }
            if(txe && apply(read(Regs::CR1::txeie)) != 0) {
                std::byte b{};
                if(txQueue.popInto(b)) {
                    apply(write(Regs::DR::dr, static_cast<std::uint32_t>(b)));
                } else {
                    apply(clear(Regs::CR1::txeie));
                }
            }
        }

        using Isr = Nvic::Isr<std::addressof(onIsr), Irq>;
    };
}}   // namespace Kvasir::Usart
