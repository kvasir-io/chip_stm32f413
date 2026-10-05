// The independent watchdog's gate (src/chip/Iwdg.hpp), compiled the way psu302 compiles its code: ungated it has
// start() and feed() as ever; gated only the keyed arm() and feed() are there, and a Kvasir::Health::Supervisor over
// it compiles - without holdOff(), since an IWDG cannot be stopped. Every check is a static_assert: compiling
// without a diagnostic is the test (firmware_check.py).
#include "HWConfig.hpp"

#include <chrono>
#include <cstdint>
#include <kvasir/Util/Health.hpp>
#include <string_view>

namespace {
template<typename W>
constexpr bool plainLife = requires {
    W::start();
    W::feed();
};
template<typename W>
constexpr bool anyPlain = requires { W::start(); } || requires { W::feed(); };
template<typename W>
constexpr bool keyedLife = requires(Kvasir::Health::FeedKey key) {
    W::arm(key);
    W::feed(key);
};

using Plain = Kvasir::Iwdg::Watchdog<500>;
using Gated = Kvasir::Iwdg::Watchdog<500, true>;

static_assert(plainLife<Plain> && !keyedLife<Plain> && !Plain::GatedFeed);
static_assert(!anyPlain<Gated> && keyedLife<Gated> && Gated::GatedFeed);
static_assert(Plain::shortestUs == Gated::shortestUs && Plain::longestUs == Gated::longestUs);

struct FakeClock {
    using rep        = std::int64_t;
    using period     = std::milli;
    using duration   = std::chrono::milliseconds;
    using time_point = std::chrono::time_point<FakeClock>;

    static time_point now() { return {}; }
};

struct LoopHealth {
    static constexpr std::string_view name        = "loop";
    static constexpr auto             maxInterval = std::chrono::milliseconds{100};
};

struct FakeStartup {
    template<typename Hook>
    using ExtendsOn = brigand::list<Kvasir::Health::Check<LoopHealth>>;

    template<typename Hook>
    static void run() {}
};

using Health = Kvasir::Health::Supervisor<FakeStartup, Gated, FakeClock>;

template<typename S>
constexpr bool hasHoldOff = requires { S::holdOff(); };
static_assert(!hasHoldOff<Health>,
              "an IWDG cannot be stopped: no holdOff() over it");

[[maybe_unused]] void turn() {
    Kvasir::Health::checkIn<LoopHealth>();
    Health::service();   // arms through the key at the first turn, feeds after
}
}   // namespace
