// The IWDG's gate (chip/Watchdog.hpp, the SDK's Health/WatchdogFront.hpp) as must-fail cases
// (kvasir_conformance_must_fail, CMakeLists.txt): WATCHDOG_FAIL=0 drives an ungated watchdog the plain way and must
// compile; 1 feeds a gated one without the supervisor's key, 2 arms it without the key, 3 disarms the ungated one (the
// IWDG cannot be stopped).
#include "kvasir_config.hpp"
//
#include "chip/chip.hpp"

#include <chrono>

#ifndef WATCHDOG_FAIL
    #error "WATCHDOG_FAIL=<0..3> selects the case"
#endif

namespace {
struct GatedConfig {
    static constexpr auto overrunTime = std::chrono::seconds{1};
    static constexpr bool gatedFeed   = true;
};

struct UngatedConfig {
    static constexpr auto overrunTime = std::chrono::seconds{1};
};

using Gated   = Kvasir::Watchdog<GatedConfig>;
using Ungated = Kvasir::Watchdog<UngatedConfig>;

template<typename W>
concept PlainFeed = requires { W::feed(); };

template<typename W>
concept Disarms = requires { W::disarm(); };
}   // namespace

void drive();

void drive() {
    Ungated::arm();
    Ungated::feed();
    static_assert(!Disarms<Ungated>, "the IWDG has no disarm");
    static_assert(!PlainFeed<Gated>, "a gated feed takes the supervisor's key");
#if WATCHDOG_FAIL == 1
    Gated::feed();
#elif WATCHDOG_FAIL == 2
    Gated::arm();
#elif WATCHDOG_FAIL == 3
    Ungated::disarm();
#endif
}
