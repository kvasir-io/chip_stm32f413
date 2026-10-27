#pragma once
// Kvasir::Chip::reboot() on the STM32F413: a software reset, AIRCR = VECTKEY | SYSRESETREQ (DDI0403E B3.2.6). A
// system reset "sets all registers to their reset values except the reset flags in the clock controller CSR register
// and the registers in the Backup domain" (RM0430 Rev 8 6.1.1); the next boot reads `software` (RCC_CSR.SFTRSTF,
// PM::resetCause(), resetKind() softwareRequest).
//
// The log printer is told first (kvasir/Runtime/AnnouncedReset.hpp): with one armed it waits until the printer is off
// the chip, counting polls at the chip's CpuHzCeiling (WaitBounds.hpp). A BootGuard's plannedReset() stays the caller's
// (Runtime/BootGuard.hpp).
#include "WaitBounds.hpp"   // Register::CpuHzCeiling<Tag::User>: announce()'s polls
#include "core/core.hpp"
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Port/Hooks.hpp"
#include "kvasir/Runtime/AnnouncedReset.hpp"

namespace Kvasir::Chip {
[[noreturn]] inline void reboot() {
    (void)AnnouncedReset::announce();
    Kvasir::SystemControl::
      resetNow();   // dsb; AIRCR; dsb; loop (core_cortex_common SystemControl.hpp)
}
}   // namespace Kvasir::Chip

/// reboot() as the port's hook (kvasir/Port/Hooks.hpp), for what the SDK ends in: Port::LateT<Port::Reboot<Tag::User>,
/// Dep>{}().
template<typename... Ts>
struct Kvasir::Port::Reboot<Kvasir::Tag::User, Ts...> {
    [[noreturn]] void operator()() const { Kvasir::Chip::reboot(); }
};
