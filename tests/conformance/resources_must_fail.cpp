// The Startup resource checks of this chip's drivers as must-fail cases (kvasir_conformance_must_fail, CMakeLists.txt):
// RESOURCES_FAIL=0 is a complete Startup list of drivers that claim hardware - an EXTI line, pins, a timer as PWM and
// as a tick at the same rate, a free-running counter, an ADC, a USART, the clock tree's bus clocks - and must compile;
// each of 1..9 adds one conflict, and must fail with its own diagnostic (the case list in CMakeLists.txt). Built with
// the conformance tree's flags, for the target, with both compilers.

#include "kvasir_config.hpp"
//
#include "chip/Adc.hpp"
#include "chip/Exti.hpp"
#include "chip/Rcc.hpp"
#include "chip/Timer.hpp"
#include "chip/Usart.hpp"
#include "chip/chip.hpp"

#include <atomic>
#include <cstdint>
#include <kvasir/Runtime/BootReport.hpp>
#include <kvasir/Runtime/FaultHandler.hpp>
#include <kvasir/Runtime/StackProtector.hpp>
#include <kvasir/StartUp/StartUp.hpp>

#ifndef RESOURCES_FAIL
    #error "RESOURCES_FAIL=<0..9> selects the case"
#endif

namespace {
using Kvasir::Io::makePinLocation;
using Kvasir::Io::pin1;
using Kvasir::Io::pin13;
using Kvasir::Io::pin4;
using Kvasir::Io::pin5;
using Kvasir::Io::pin6;
using Kvasir::Io::pin7;
using Kvasir::Io::portA;
using Kvasir::Io::portB;
using Kvasir::Io::portC;

// 8 MHz crystal, 100 MHz core, APB1 and APB2 at 50 MHz
using Tree = Kvasir::Rcc::
  ClockTree<8'000'000, 100'000'000, Kvasir::Rcc::Supply::from2V7, false, false, 50'000'000>;

struct SystickConfig {
    static constexpr auto clockBase      = Kvasir::Systick::useProcessorClock;
    static constexpr auto clockSpeed     = Tree::hclkHz;
    static constexpr auto minOverrunTime = std::chrono::years(15);
};

using Clock = Kvasir::Systick::SystickClockBase<SystickConfig>;

struct ClockSettings {
    using Provides = Tree::Provides;

    static void coreClockInit() { Tree::init(); }

    static void peripheryClockInit() {}
};

using EdgePin  = decltype(makePinLocation(portB, pin4));   // EXTI line 4
using OtherPin = decltype(makePinLocation(portA, pin4));   // EXTI line 4 too
using LedPin   = decltype(makePinLocation(portC, pin13));
using AdcIn    = decltype(makePinLocation(portA, pin1));
using PwmOut   = decltype(makePinLocation(portB, pin5));   // TIM3 channel 2, AF2
using UartTx   = decltype(makePinLocation(portB, pin6));   // USART1, AF7
using UartRx   = decltype(makePinLocation(portB, pin7));

std::atomic<std::uint32_t> ticks{0};
std::atomic<std::uint32_t> edges{0};

void onTick() { ticks.store(ticks.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed); }

void onEdge() { edges.store(edges.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed); }

struct LedPinConfig {
    static constexpr auto initStepPinConfig = list(makeOutput(LedPin{}));
};

using Pwm  = Kvasir::Timer::Pwm<Tree, 3, 2, 25'000, PwmOut, 2>;
using Tick = Kvasir::Timer::
  Tick<Tree, 3, 25'000, &onTick, 3>;   // TIM3 at the PWM's rate: one timer resource
using Counter = Kvasir::Timer::Counter<Tree, 2, 1'000'000>;
using Edge    = Kvasir::Exti::Line<EdgePin, Kvasir::Exti::Edge::rising, &onEdge>;
using Adc     = Kvasir::Adc::Single<Tree, 1, AdcIn>;
using Uart    = Kvasir::Usart::Port<Tree, 1, 115'200, UartTx, UartRx, 7>;
}   // namespace

using StackProtector = Kvasir::StackProtector<>;
using FaultHandler   = Kvasir::Fault::HandlerFor<>;
using ComBackend     = uc_log::ComBackend<uc_log::Tag::User>;

#if RESOURCES_FAIL == 1   // a second EXTI line 4
void onOther();
using Extra = Kvasir::Exti::Line<OtherPin, Kvasir::Exti::Edge::falling, &onOther>;
#elif RESOURCES_FAIL == 2   // the LED pin configured by a second entry
struct Extra {
    static constexpr auto initStepPinConfig = list(makeInput(LedPin{}));
};
#elif RESOURCES_FAIL == 3   // TIM3 channel 1 at another rate than channel 2
using Extra = Kvasir::Timer::Pwm<Tree, 3, 1, 1'000, decltype(makePinLocation(portC, pin6)), 2>;
#elif RESOURCES_FAIL == 4   // TIM3 channel 2 twice
using Extra = Kvasir::Timer::Pwm<Tree, 3, 2, 25'000, decltype(makePinLocation(portC, pin7)), 2>;
#elif RESOURCES_FAIL == 5   // a Tick on TIM2, which the Counter runs free
using Extra = Kvasir::Timer::Tick<Tree, 2, 1'000, &onTick>;
#elif RESOURCES_FAIL == 6   // a second Adc::Single on the one ADC block
using Extra = Kvasir::Adc::Single<Tree, 4, OtherPin>;
#elif RESOURCES_FAIL == 7   // a driver computed from another clock tree than ClockSettings programs
using Extra = Kvasir::Usart::Port<Kvasir::Rcc::ClockTree<8'000'000, 84'000'000>,
                                  6,
                                  115'200,
                                  decltype(makePinLocation(portC, pin6)),
                                  decltype(makePinLocation(portC, pin7)),
                                  8>;
#elif RESOURCES_FAIL \
  == 9   // an instance row for a block on no APB: RCC itself (AHB1, RM0430 Table 1) - the bus comes
         // from the base address, so a row cannot name a wrong one, only a block without one
struct Extra : Kvasir::Rcc::OnBusOf<Kvasir::Peripheral::RCC::Registers<>> {};

static_assert(Extra::bus == Kvasir::Rcc::Bus::apb1);
#endif

#if RESOURCES_FAIL == 8   // clock settings without `using Provides = ClockTree::Provides;`
struct Settings {
    static void coreClockInit() { Tree::init(); }

    static void peripheryClockInit() {}
};
#else
using Settings = ClockSettings;
#endif

using Startup = Kvasir::Startup::Startup<Settings,
                                         Clock,
                                         FaultHandler,
                                         ComBackend,
                                         Kvasir::Boot::Report<>,
                                         StackProtector,
                                         LedPinConfig,
                                         Counter,
                                         Pwm,
                                         Tick,
                                         Edge,
                                         Adc,
                                         Uart
#if RESOURCES_FAIL >= 1 && RESOURCES_FAIL != 8
                                         ,
                                         Extra
#endif
                                         >;

int main() {
    Pwm::setPermille(300);
    while(true) { Startup::run<Kvasir::Hook::MainLoop>(); }
}

template struct Kvasir::Startup::Start<Startup>;
