#pragma once

#include "ClockMath.hpp"
#include "Interrupt.hpp"
#include "Io.hpp"
#include "kvasir/Common/Interrupt.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Prescaler.hpp"
#include "peripherals/RCC.hpp"
#include "peripherals/TIM10.hpp"
#include "peripherals/TIM11.hpp"
#include "peripherals/TIM12.hpp"
#include "peripherals/TIM13_14.hpp"
#include "peripherals/TIM2.hpp"
#include "peripherals/TIM3_4.hpp"
#include "peripherals/TIM5.hpp"
#include "peripherals/TIM6_7.hpp"
#include "peripherals/TIM9.hpp"

#include <cstdint>
#include <memory>
#include <type_traits>

// The general-purpose and basic timers of the STM32F413 (RM0430 Rev 8 chapters 18-20): TIM2-5
// (16 bit, TIM2 and TIM5 32 bit, four channels), TIM9/12 (two channels), TIM10/11/13/14 (one
// channel), TIM6/7 (no channel). The counter runs from the timer kernel clock (Rcc::ClockTree
// timer1Hz on APB1, timer2Hz on APB2) through PSC+1, and wraps at ARR: update rate
// clk / ((PSC + 1) * (ARR + 1)) (18.3.1, 18.4.11, 18.4.12).
//
// Three uses, each a Startup entry:
//   Tick<Clocks, 13, 50'000, &f, 0>      f() at 50 kHz from the update interrupt
//   Pwm<Clocks, 3, 2, 25'000, Pin, 2>    edge-aligned PWM mode 1 on TIM3 channel 2, duty set at run time
//   Counter<Clocks, 2, 1'000'000>        a free-running 32-bit microsecond counter (TIM2/TIM5)
// TIM1/TIM8 (advanced) are not covered: their update and capture interrupts are separate vectors
// shared with TIM9-14, and their outputs need BDTR.MOE.
namespace Kvasir { namespace Timer {
    enum class Bus { apb1, apb2 };

    namespace Detail { using RCC = Kvasir::Peripheral::RCC::Registers<>; }

    template<unsigned N>
    struct Instance;

    // Per timer: its generated registers, its bus (RM0430 Table 1 boundary addresses / 6.3.15,
    // 6.3.16 enable bits), counter width, channels and update vector (Table 40).
#define KVASIR_STM32_TIMER(N, REGS, BUS, ENABLE, BITS, CHANNELS, IRQ)            \
    template<>                                                                   \
    struct Instance<N> {                                                         \
        using Regs                         = REGS;                               \
        static constexpr Bus      bus      = Bus::BUS;                           \
        static constexpr unsigned bits     = BITS;                               \
        static constexpr unsigned channels = CHANNELS;                           \
        using Irq                          = std::remove_cvref_t<decltype(IRQ)>; \
        static constexpr auto enable() {                                         \
            return write(ENABLE, Register::value<std::uint32_t, 1>());           \
        }                                                                        \
    }

    KVASIR_STM32_TIMER(2,
                       Peripheral::TIM2::Registers<>,
                       apb1,
                       Detail::RCC::APB1ENR::tim2en,
                       32,
                       4,
                       Interrupt::tim2);
    KVASIR_STM32_TIMER(3,
                       Peripheral::TIM3_4::Registers<3>,
                       apb1,
                       Detail::RCC::APB1ENR::tim3en,
                       16,
                       4,
                       Interrupt::tim3);
    KVASIR_STM32_TIMER(4,
                       Peripheral::TIM3_4::Registers<4>,
                       apb1,
                       Detail::RCC::APB1ENR::tim4en,
                       16,
                       4,
                       Interrupt::tim4);
    KVASIR_STM32_TIMER(5,
                       Peripheral::TIM5::Registers<>,
                       apb1,
                       Detail::RCC::APB1ENR::tim5en,
                       32,
                       4,
                       Interrupt::tim5);
    KVASIR_STM32_TIMER(6,
                       Peripheral::TIM6_7::Registers<6>,
                       apb1,
                       Detail::RCC::APB1ENR::tim6en,
                       16,
                       0,
                       Interrupt::tim6_dac);
    KVASIR_STM32_TIMER(7,
                       Peripheral::TIM6_7::Registers<7>,
                       apb1,
                       Detail::RCC::APB1ENR::tim7en,
                       16,
                       0,
                       Interrupt::tim7);
    KVASIR_STM32_TIMER(9,
                       Peripheral::TIM9::Registers<>,
                       apb2,
                       Detail::RCC::APB2ENR::tim9en,
                       16,
                       2,
                       Interrupt::tim1_brk_tim9);
    KVASIR_STM32_TIMER(10,
                       Peripheral::TIM10::Registers<>,
                       apb2,
                       Detail::RCC::APB2ENR::tim10en,
                       16,
                       1,
                       Interrupt::tim1_up_tim10);
    KVASIR_STM32_TIMER(11,
                       Peripheral::TIM11::Registers<>,
                       apb2,
                       Detail::RCC::APB2ENR::tim11en,
                       16,
                       1,
                       Interrupt::tim1_trg_com_tim11);
    KVASIR_STM32_TIMER(12,
                       Peripheral::TIM12::Registers<>,
                       apb1,
                       Detail::RCC::APB1ENR::tim12en,
                       16,
                       2,
                       Interrupt::tim8_brk_tim12);
    KVASIR_STM32_TIMER(13,
                       Peripheral::TIM13_14::Registers<13>,
                       apb1,
                       Detail::RCC::APB1ENR::tim13en,
                       16,
                       1,
                       Interrupt::tim8_up_tim13);
    KVASIR_STM32_TIMER(14,
                       Peripheral::TIM13_14::Registers<14>,
                       apb1,
                       Detail::RCC::APB1ENR::tim14en,
                       16,
                       1,
                       Interrupt::tim8_trg_com_tim14);
#undef KVASIR_STM32_TIMER

    template<typename Clocks, unsigned N>
    inline constexpr std::uint32_t kernelHz
      = Instance<N>::bus == Bus::apb1 ? Clocks::timer1Hz : Clocks::timer2Hz;

    /// f() at RateHz from the update interrupt. The ISR clears UIF first (SR is rc_w0, the write
    /// of ~UIF leaves the other flags), then calls f.
    template<typename Clocks,
             unsigned      N,
             std::uint64_t RateHz,
             void (*F)(),
             unsigned             Priority = 0,
             Prescaler::Tolerance Tol      = Prescaler::Tolerance{std::ratio<1, 1000>{}}>
    struct Tick {
        using I    = Instance<N>;
        using Regs = typename I::Regs;
        using Irq  = typename I::Irq;

        static constexpr Division division = divisionFor(kernelHz<Clocks, N>, RateHz, I::bits);
        static constexpr bool     Ok       = [] {
            Prescaler::assertInTolerance<division.achieved, RateHz, Tol, "timer tick rate">();
            return true;
        }();
        static_assert(Ok);

        static constexpr auto powerClockEnable = list(I::enable());
        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::PSC::FULLREGISTER, Register::value<std::uint32_t, division.psc>()),
                 write(Regs::ARR::FULLREGISTER,
                       Register::value<std::uint32_t, static_cast<std::uint32_t>(division.arr)>()),
                 write(Regs::CR1::urs, Register::value<std::uint32_t, 1>()),
                 write(Regs::DIER::uie, Register::value<std::uint32_t, 1>()));

        // UG loads PSC (it is buffered: 18.4.11); URS keeps that UG from raising UIF - but only
        // once URS is written, and one list's writes go out highest address first: EGR (0x14)
        // before CR1 (0x00). UG in the list raised UIF, and the interrupt fired the moment Startup
        // enabled the NVIC, before the other peripherals were enabled (psu302, 2026-10-06: the
        // dither tick spun in Spi::waitIdle on a disabled SPI, main() never ran). Hence here, after
        // the configuration and before the NVIC enable.
        static void preEnableRuntimeInit() {
            apply(write(Regs::EGR::ug, Register::value<std::uint32_t, 1>()));
        }

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<Priority>(Irq{}), Nvic::makeClearPending(Irq{}));
        static constexpr auto initStepPeripheryEnable
          = list(Nvic::makeEnable(Irq{}),
                 write(Regs::CR1::cen, Register::value<std::uint32_t, 1>()));

        static void onIsr() {
            apply(write(Regs::SR::uif, Register::value<std::uint32_t, 0>()));
            F();
        }

        static constexpr Nvic::Isr<std::addressof(onIsr), Irq> isr{};
    };

    /// Edge-aligned PWM mode 1 on one channel (18.3.9: high while CNT < CCR), preloaded CCR and
    /// ARR so a new duty takes effect at the next period. The pin gets its alternate function.
    template<typename Clocks,
             unsigned      N,
             unsigned      Channel,
             std::uint64_t RateHz,
             typename Pin,
             int                  Af,
             Prescaler::Tolerance Tol = Prescaler::Tolerance{std::ratio<5, 1000>{}}>
    struct Pwm {
        using I    = Instance<N>;
        using Regs = typename I::Regs;
        static_assert(Channel >= 1 && Channel <= I::channels,
                      "this timer has no such channel");
        static_assert(Channel <= 2,
                      "channels 3 and 4 (CCMR2) are not wired up yet");

        static constexpr Division division = divisionFor(kernelHz<Clocks, N>, RateHz, I::bits);
        static constexpr bool     Ok       = [] {
            Prescaler::assertInTolerance<division.achieved, RateHz, Tol, "PWM rate">();
            return true;
        }();
        static_assert(Ok);

        /// Counts in one period: the duty is 0..period.
        static constexpr std::uint32_t period = static_cast<std::uint32_t>(division.arr + 1);

        static constexpr auto ccmr() {
            // OCxM = 110 (PWM mode 1), OCxPE = 1 (18.4.7 TIMx_CCMR1 output compare mode).
            if constexpr(Channel == 1) {
                return list(
                  write(Regs::CCMR1_OUTPUT::oc1m, Register::value<std::uint32_t, 0b110>()),
                  write(Regs::CCMR1_OUTPUT::oc1pe, Register::value<std::uint32_t, 1>()));
            } else {
                return list(
                  write(Regs::CCMR1_OUTPUT::oc2m, Register::value<std::uint32_t, 0b110>()),
                  write(Regs::CCMR1_OUTPUT::oc2pe, Register::value<std::uint32_t, 1>()));
            }
        }

        static constexpr auto ccer() {
            if constexpr(Channel == 1) {
                return write(Regs::CCER::cc1e, Register::value<std::uint32_t, 1>());
            } else {
                return write(Regs::CCER::cc2e, Register::value<std::uint32_t, 1>());
            }
        }

        static constexpr auto powerClockEnable = list(I::enable());
        static constexpr auto initStepPinConfig
          = list(action(Io::Action::PinFunction<Af>{}, Pin{}));
        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::PSC::FULLREGISTER, Register::value<std::uint32_t, division.psc>()),
                 write(Regs::ARR::FULLREGISTER, Register::value<std::uint32_t, period - 1>()),
                 ccmr(),
                 write(Regs::CR1::arpe, Register::value<std::uint32_t, 1>()),
                 ccer(),
                 write(Regs::EGR::ug, Register::value<std::uint32_t, 1>()));
        static constexpr auto initStepPeripheryEnable
          = list(write(Regs::CR1::cen, Register::value<std::uint32_t, 1>()));

        /// Duty in counts, clamped to the period (0 = always low, period = always high).
        static void setCounts(std::uint32_t counts) {
            if(counts > period) { counts = period; }
            if constexpr(Channel == 1) {
                apply(write(Regs::CCR1::FULLREGISTER, counts));
            } else {
                apply(write(Regs::CCR2::FULLREGISTER, counts));
            }
        }

        /// Duty in per mille, clamped to 0..1000.
        static void setPermille(std::uint32_t permille) {
            if(permille > 1000) { permille = 1000; }
            setCounts(static_cast<std::uint32_t>(std::uint64_t{period} * permille / 1000));
        }
    };

    /// A free-running up-counter at TickHz over the full width of a 32-bit timer (TIM2, TIM5),
    /// no interrupt: read() is the time base for edge timestamps.
    template<typename Clocks, unsigned N, std::uint64_t TickHz>
    struct Counter {
        using I    = Instance<N>;
        using Regs = typename I::Regs;
        static_assert(I::bits == 32,
                      "a free-running counter wants TIM2 or TIM5 (32 bit)");
        static_assert(kernelHz<Clocks,
                               N>
                          % TickHz
                        == 0,
                      "the tick must divide the timer clock");
        static constexpr std::uint32_t psc
          = static_cast<std::uint32_t>(kernelHz<Clocks, N> / TickHz - 1);
        static_assert(psc <= 0xFFFF,
                      "PSC is 16 bits (18.4.11)");

        static constexpr auto powerClockEnable = list(I::enable());
        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::PSC::FULLREGISTER, Register::value<std::uint32_t, psc>()),
                 write(Regs::ARR::FULLREGISTER, Register::value<std::uint32_t, 0xFFFF'FFFF>()),
                 write(Regs::EGR::ug, Register::value<std::uint32_t, 1>()));
        static constexpr auto initStepPeripheryEnable
          = list(write(Regs::CR1::cen, Register::value<std::uint32_t, 1>()));

        [[nodiscard]] static std::uint32_t read() {
            return get<0>(apply(Register::read(Regs::CNT::FULLREGISTER)));
        }
    };
}}   // namespace Kvasir::Timer
