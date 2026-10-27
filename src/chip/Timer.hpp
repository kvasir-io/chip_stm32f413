#pragma once

#include "ClockMath.hpp"
#include "Clocks.hpp"
#include "Instances.hpp"
#include "Interrupt.hpp"
#include "Io.hpp"
#include "Rcc.hpp"
#include "kvasir/Common/Interrupt.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"
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

#include <algorithm>
#include <cstdint>
#include <memory>
#include <type_traits>

// The general-purpose and basic timers of the STM32F413 (RM0430 Rev 8 chapters 18-20): TIM2-5
// (16 bit, TIM2 and TIM5 32 bit, four channels), TIM9/12 (two channels), TIM10/11/13/14 (one
// channel), TIM6/7 (no channel). The counter runs from the timer kernel clock of its bus
// (Rcc::ClockTree::timerHz<Bus>) through PSC+1, and wraps at ARR: update rate
// clk / ((PSC + 1) * (ARR + 1)) (18.3.1, 18.4.11, 18.4.12).
//
// Three uses, each a Startup entry:
//   Tick<Clocks, 13, 50'000, &f, 0>      f() at 50 kHz from the update interrupt
//   Pwm<Clocks, 3, 2, 25'000, Pin, 2>    edge-aligned PWM mode 1 on TIM3 channel 2, duty set at run time
//   Counter<Clocks, 2, 1'000'000>        a free-running 32-bit microsecond counter (TIM2/TIM5)
// TIM1/TIM8 (advanced) are not covered: their update and capture interrupts are separate vectors
// shared with TIM9-14, and their outputs need BDTR.MOE.
namespace Kvasir { namespace Timer {
    /// A timer's registers, counter width, channels and update vector (RM0430 Rev 8 chapters 18-20,
    /// Table 40).
    template<typename R, unsigned Bits, unsigned Channels, auto const& UpdateIrq>
    struct Shape : Rcc::OnBusOf<R> {   // its bus (by the base address) and enable bit (Blocks.hpp)
        using Regs                         = R;
        static constexpr unsigned bits     = Bits;
        static constexpr unsigned channels = Channels;
        using Irq                          = std::remove_cvref_t<decltype(UpdateIrq)>;
    };

    template<unsigned N>
    struct Instance;

    // Per timer: its register set (the bus by its base, RM0430 Table 1, Instances.hpp; its enable bit: Blocks.hpp) and
    // its shape.
    // clang-format off
    template<> struct Instance<2> : Shape<Peripheral::TIM2::Registers<>, 32, 4, Interrupt::tim2> {};
    template<> struct Instance<3> : Shape<Peripheral::TIM3_4::Registers<3>, 16, 4, Interrupt::tim3> {};
    template<> struct Instance<4> : Shape<Peripheral::TIM3_4::Registers<4>, 16, 4, Interrupt::tim4> {};
    template<> struct Instance<5> : Shape<Peripheral::TIM5::Registers<>, 32, 4, Interrupt::tim5> {};
    template<> struct Instance<6> : Shape<Peripheral::TIM6_7::Registers<6>, 16, 0,
                                          Interrupt::tim6_glb_it_dac1_dac2> {};
    template<> struct Instance<7> : Shape<Peripheral::TIM6_7::Registers<7>, 16, 0, Interrupt::tim7> {};
    template<> struct Instance<9> : Shape<Peripheral::TIM9::Registers<>, 16, 2, Interrupt::tim1_brk_tim9> {};
    template<> struct Instance<10> : Shape<Peripheral::TIM10::Registers<>, 16, 1, Interrupt::tim1_up_tim10> {};
    template<> struct Instance<11> : Shape<Peripheral::TIM11::Registers<>, 16, 1, Interrupt::tim1_trg_com_tim11> {};
    template<> struct Instance<12> : Shape<Peripheral::TIM12::Registers<>, 16, 2, Interrupt::tim8_brk_tim12> {};
    template<> struct Instance<13> : Shape<Peripheral::TIM13_14::Registers<13>, 16, 1, Interrupt::tim8_up_tim13> {};
    template<> struct Instance<14> : Shape<Peripheral::TIM13_14::Registers<14>, 16, 1, Interrupt::tim8_trg_com_tim14> {};
    // clang-format on

    /// A timer's counter as a Startup resource, keyed by the timer, its PSC and ARR the payload: users
    /// that run one timer at the same period merge (a Tick and a Pwm, two Pwm channels), users that
    /// would write other PSC/ARR values collide - before, the last one's write won silently
    /// (18.4.11, 18.4.12).
    struct TimerTag {
        static constexpr char const* describeAs     = "timer (TIMx, PSC, ARR)";
        static constexpr bool        mergeIdentical = true;
    };

    /// One output channel of a timer, keyed by (timer, channel): exclusive.
    struct TimerChannelTag {
        static constexpr char const* describeAs = "timer channel (TIMx, channel)";
        static constexpr unsigned    keyArity   = 2;
    };

    template<typename Clocks, unsigned N>
    inline constexpr std::uint32_t kernelHz = Clocks::template timerHz<Instance<N>::bus>;

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
        // Startup: the timer kernel clock the division is computed from (Clocks.hpp); no owner -
        // drivers may share a timer by channel.
        using Claims = Hal::Claim<Kvasir::Clocks::ClkTim<Instance<N>::bus>, kernelHz<Clocks, N>>;
        using Irq    = typename I::Irq;

        static constexpr Division division = divisionFor(kernelHz<Clocks, N>, RateHz, I::bits);
        static_assert(Chip::checkedInTolerance<division.achieved,
                                               RateHz,
                                               Tol,
                                               "timer tick rate">);

        using Provides = brigand::list<Startup::Resource<TimerTag, N, division.psc, division.arr>>;

        static constexpr auto powerClockEnable = I::powerClockEnable;
        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::PSC::FULLREGISTER, Register::value<std::uint32_t, division.psc>()),
                 write(Regs::ARR::FULLREGISTER,
                       Register::value<std::uint32_t, static_cast<std::uint32_t>(division.arr)>()),
                 set(Regs::CR1::urs),
                 set(Regs::DIER::uie));

        // UG loads PSC (it is buffered: 18.4.11); URS keeps that UG from raising UIF - but only
        // once URS is written, and one list's writes go out highest address first: EGR (0x14)
        // before CR1 (0x00). UG in the list raised UIF, and the interrupt fired the moment Startup
        // enabled the NVIC, before the other peripherals were enabled (an ISR that waits on one of
        // them then spins, and main() never runs). Hence here, after
        // the configuration and before the NVIC enable.
        static void preEnableRuntimeInit() { apply(set(Regs::EGR::ug)); }

        static constexpr auto initStepInterruptConfig
          = list(Nvic::makeSetPriority<Priority>(Irq{}), Nvic::makeClearPending(Irq{}));
        static constexpr auto initStepPeripheryEnable
          = list(Nvic::makeEnable(Irq{}), set(Regs::CR1::cen));

        static void onIsr() {
            apply(clear(Regs::SR::uif));
            F();
        }

        using Isr = Nvic::Isr<std::addressof(onIsr), Irq>;
    };

    /// One output channel's fields: OCxM/OCxPE (18.4.7 TIMx_CCMR1 output compare mode), CCxE (18.4.9 TIMx_CCER)
    /// and its compare register (18.4.13, 18.4.14). Channels 3 and 4 (CCMR2, 18.4.8) would be two more rows.
    template<typename Regs, unsigned Channel>
    struct OutputChannel;

    template<typename Regs>
    struct OutputChannel<Regs, 1> {
        static constexpr auto ocm  = Regs::CCMR1_OUTPUT::oc1m;
        static constexpr auto ocpe = Regs::CCMR1_OUTPUT::oc1pe;
        static constexpr auto cce  = Regs::CCER::cc1e;
        using Ccr                  = typename Regs::CCR1;
    };

    template<typename Regs>
    struct OutputChannel<Regs, 2> {
        static constexpr auto ocm  = Regs::CCMR1_OUTPUT::oc2m;
        static constexpr auto ocpe = Regs::CCMR1_OUTPUT::oc2pe;
        static constexpr auto cce  = Regs::CCER::cc2e;
        using Ccr                  = typename Regs::CCR2;
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
        // Startup: the timer kernel clock the division is computed from (Clocks.hpp); no owner -
        // drivers may share a timer by channel.
        using Claims = Hal::Claim<Kvasir::Clocks::ClkTim<Instance<N>::bus>, kernelHz<Clocks, N>>;
        static_assert(Channel >= 1 && Channel <= I::channels,
                      "this timer has no such channel");
        static_assert(Channel <= 2,
                      "channels 3 and 4 (CCMR2) are not wired up yet");

        static constexpr Division division = divisionFor(kernelHz<Clocks, N>, RateHz, I::bits);
        static_assert(Chip::checkedInTolerance<division.achieved,
                                               RateHz,
                                               Tol,
                                               "PWM rate">);

        /// Counts in one period: the duty is 0..period.
        static constexpr std::uint32_t period = static_cast<std::uint32_t>(division.arr + 1);

        using Provides = brigand::list<Startup::Resource<TimerTag, N, division.psc, period - 1>,
                                       Startup::Resource<TimerChannelTag, N, Channel>>;

        using C = OutputChannel<Regs, Channel>;

        static constexpr auto powerClockEnable = I::powerClockEnable;
        static constexpr auto initStepPinConfig
          = list(action(Io::Action::PinFunction<Af>{}, Pin{}));
        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::PSC::FULLREGISTER, Register::value<std::uint32_t, division.psc>()),
                 write(Regs::ARR::FULLREGISTER, Register::value<std::uint32_t, period - 1>()),
                 // OCxM = 110 (PWM mode 1), OCxPE = 1
                 write(C::ocm, Register::value<std::uint32_t, 0b110>()),
                 set(C::ocpe),
                 set(Regs::CR1::arpe),
                 set(C::cce),
                 set(Regs::EGR::ug));
        static constexpr auto initStepPeripheryEnable = list(set(Regs::CR1::cen));

    private:
        /// Duty in counts, clamped to the period (0 = always low, period = always high).
        static void setCounts(std::uint32_t counts) {
            counts = std::min(counts, period);
            apply(write(C::Ccr::FULLREGISTER, counts));
        }

    public:
        /// Duty in per mille, clamped to 0..1000.
        static void setPermille(std::uint32_t permille) {
            permille = std::min(permille, std::uint32_t{1000});
            setCounts(static_cast<std::uint32_t>(std::uint64_t{period} * permille / 1000));
        }
    };

    /// A free-running up-counter at TickHz over the full width of a 32-bit timer (TIM2, TIM5),
    /// no interrupt: read() is the time base for edge timestamps.
    template<typename Clocks, unsigned N, std::uint64_t TickHz>
    struct Counter {
        using I    = Instance<N>;
        using Regs = typename I::Regs;
        // Startup: the timer kernel clock the division is computed from (Clocks.hpp); no owner -
        // drivers may share a timer by channel.
        using Claims = Hal::Claim<Kvasir::Clocks::ClkTim<Instance<N>::bus>, kernelHz<Clocks, N>>;
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

        using Provides = brigand::list<Startup::Resource<TimerTag, N, psc, 0xFFFF'FFFFU>>;

        static constexpr auto powerClockEnable = I::powerClockEnable;
        static constexpr auto initStepPeripheryConfig
          = list(write(Regs::PSC::FULLREGISTER, Register::value<std::uint32_t, psc>()),
                 write(Regs::ARR::FULLREGISTER, Register::value<std::uint32_t, 0xFFFF'FFFF>()),
                 set(Regs::EGR::ug));
        static constexpr auto initStepPeripheryEnable = list(set(Regs::CR1::cen));

        [[nodiscard]] static std::uint32_t read() {
            return get<0>(apply(Register::read(Regs::CNT::FULLREGISTER)));
        }
    };
}}   // namespace Kvasir::Timer
