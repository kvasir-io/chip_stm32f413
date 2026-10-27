#pragma once

#include "ClockMath.hpp"
#include "kvasir/Health/WatchdogFront.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/StartUp/Resources.hpp"
#include "kvasir/Util/Config.hpp"
#include "peripherals/DBG.hpp"
#include "peripherals/IWDG.hpp"

#include <chrono>
#include <cstdint>

// The independent watchdog (RM0430 Rev 8 chapter 22) in the RP/SAM drivers' shape (chip_rp_common watchdog.hpp,
// chip_atsam_common Wdt.hpp): the same Config names, the SDK's one front for arm/feed
// (kvasir/Health/WatchdogFront.hpp). A 12-bit down-counter on the LSI through a prescaler of 4 to 256; KR 0xCCCC
// starts it (and the LSI), 0xAAAA reloads it, 0x5555 unlocks PR and RLR (22.3, md l.29060-29070; 22.4.1, md
// l.29127). "Once started it cannot be stopped except by a reset" (md l.4695): there is no disarm(), so a supervisor
// over it has no holdOff().
//
//   struct WatchdogConfig {
//       static constexpr auto overrunTime  = 500ms;
//       static constexpr bool gatedFeed    = true;   // optional, default false
//       static constexpr bool pauseOnDebug = true;   // optional, default true
//   };
//   using Watchdog = Kvasir::Watchdog<WatchdogConfig>;   // ungated: in the Startup list (or arm() it)
//   Watchdog::feed();
//
// The LSI is anywhere from 16.1 to 47 kHz (DS11581 Rev 7 Table 46 "LSI oscillator characteristics", md l.4265), so
// the setting is chosen for the SHORTEST timeout (LSI at 47 kHz) to be at least `overrunTime`: that is `Timeout`; the
// longest (16.1 kHz) is about 2.9 times that and is `longestUs`. No `clockSpeed`: the LSI is the only clock.
//
// pauseOnDebug: DBGMCU_APB1_FZ.DBG_IWDG_STOP (34.16.4, bit 12 md l.56643; 22.3.3 md l.29084) holds the counter while
// a debugger halts the core, so a breakpoint does not reset the board.
//
// Gated (`gatedFeed = true`): only Kvasir::Health::Supervisor arms and feeds it (Health/HealthKey.hpp) - the plain
// arm() and feed() do not compile, the FeedKey ones do; its first service() arms. An ungated one arms itself as a
// Startup entry (runtimeInit()).
namespace Kvasir {

// Startup resource: the one watchdog (kvasir/StartUp/Resources.hpp).
struct WatchdogTag {};

template<typename Config>
struct Watchdog : Health::WatchdogFront<Watchdog<Config>, Health::gatedFeedOf<Config>()> {
    using Provides = brigand::list<Startup::Resource<WatchdogTag, 0>>;

    static constexpr bool gatedFeed = Health::gatedFeedOf<Config>();
    static constexpr bool pauseOnDebug
      = static_cast<bool>(KVASIR_CONFIG_OR(Config, pauseOnDebug, true));

    static constexpr std::uint64_t timeoutUs = static_cast<std::uint64_t>(
      std::chrono::ceil<std::chrono::microseconds>(Config::overrunTime).count());
    static constexpr Iwdg::Setting setting    = Iwdg::settingFor(timeoutUs);
    static constexpr std::uint64_t shortestUs = Iwdg::periodUs(setting, Iwdg::lsiMaxHz);
    static constexpr std::uint64_t longestUs  = Iwdg::periodUs(setting, Iwdg::lsiMinHz);
    static_assert(shortestUs >= timeoutUs,
                  "the watchdog's longest timeout is 4096 x 256 / 47 kHz = 22 s");

    /// The timeout the health supervisor checks its period against: the shortest one, with the LSI at its fastest
    /// (Health/HealthKey.hpp: WatchdogShape).
    static constexpr std::chrono::microseconds timeout{shortestUs};

    using IWDG = Kvasir::Peripheral::IWDG::Registers<>;
    using DBG  = Kvasir::Peripheral::DBG::Registers<>;

    /// The ungated watchdog as a Startup entry: armed after the enables. A gated one is armed by the supervisor.
    static void runtimeInit()
        requires(!gatedFeed)
    {
        armNow();
    }

private:
    friend struct Health::WatchdogFront<Watchdog, gatedFeed>;

    // arm(): KR 0xCCCC first (it starts the LSI), then 0x5555 and the setting; PR/RLR take effect once SR.PVU/RVU
    // drop (22.4.4), which the first feed does not need to wait for.
    static void armNow() {
        if constexpr(pauseOnDebug) {
            apply(set(DBG::DBGMCU_APB1_FZ::iwdeg_stop));
        } else {
            apply(clear(DBG::DBGMCU_APB1_FZ::iwdeg_stop));
        }
        apply(write(IWDG::KR::key, Register::value<std::uint32_t, 0xCCCC>()));
        apply(write(IWDG::KR::key, Register::value<std::uint32_t, 0x5555>()));
        apply(write(IWDG::PR::pr, Register::value<std::uint32_t, setting.pr>()));
        apply(write(IWDG::RLR::rl, Register::value<std::uint32_t, setting.rlr>()));
        feedNow();
    }

    // feed(): KR 0xAAAA reloads the counter from RLR.
    static void feedNow() { apply(write(IWDG::KR::key, Register::value<std::uint32_t, 0xAAAA>())); }
};
}   // namespace Kvasir
