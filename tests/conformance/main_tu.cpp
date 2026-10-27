// The conformance tree's main TU (Kvasir_SDK tests/conformance/conformance.cmake MAIN_TU): an image's main TU in the
// order a main may choose - kvasir/StartUp/StartUp.hpp FIRST, the chip after it - with a Startup instantiated, so every
// late hook (FirstInitStep, the fault handler's interrupt traits, the NVIC clear-all's traits, the chip's entries) is
// compiled behind the SDK's header. Linked with a second TU (chip/chip.hpp + chip/drivers.hpp) into one object, and
// with a copy of itself, which must fail on the one-TU rule's guard. Never run.
// clang-format off: the SDK's Startup header first, on purpose
#include <kvasir/StartUp/StartUp.hpp>
// clang-format on

#include <chip/Rcc.hpp>
#include <chip/chip.hpp>
#include <chrono>
#include <kvasir/Runtime/BootReport.hpp>   // after the chip: its reset cause by name (until step 140)
#include <kvasir/Runtime/FaultHandler.hpp>

namespace {
// 8 MHz crystal, 100 MHz core, APB1 and APB2 at 50 MHz (instantiate.cpp's tree)
using Tree = Kvasir::Rcc::
  ClockTree<8'000'000, 100'000'000, Kvasir::Rcc::Supply::from2V7, false, false, 50'000'000>;

struct ClockSettings {
    using Provides = Tree::Provides;

    static void coreClockInit() { Tree::init(); }

    static void peripheryClockInit() {}
};

// SysTick on the tree's core clock (the fixture's Conformance::SystickClock assumes the 16 MHz HSI)
struct SystickConfig {
    static constexpr auto clockBase      = Kvasir::Systick::useProcessorClock;
    static constexpr auto clockSpeed     = Tree::hclkHz;
    static constexpr auto minOverrunTime = std::chrono::years(15);
};

using Clock = Kvasir::Systick::SystickClockBase<SystickConfig>;
}   // namespace

using Startup = Kvasir::Startup::Startup<ClockSettings,
                                         Clock,
                                         uc_log::ComBackend<uc_log::Tag::User>,
                                         Kvasir::Boot::Report<>,
                                         Kvasir::Fault::HandlerFor<>>;

int main() {
    while(true) { Startup::run<Kvasir::Hook::MainLoop>(); }
}

template struct Kvasir::Startup::Start<Startup>;
