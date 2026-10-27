#pragma once

// Kvasir::Chip::debuggerPresent() on the STM32F413, and the functor for uc_log's RTT backends (their
// DebuggerPresentFunction). No includes: a firmware names the functor in its ComBackend before uc_log and the chip.
//
// Always true: the chip keeps no "a probe was seen" bit. Kvasir::Core::haltingDebug() (cortex_common/Debug.hpp) is the
// tri-state "is halting debug on" query, and DHCSR.C_DEBUGEN is no gate for logging: a printer's own reset or flash
// clears it until its next session connects, and every line written meanwhile would be lost. With
// rtt::BufferMode::skip a board without a probe drops lines instead of stalling.
namespace Kvasir::Chip {
[[nodiscard]] constexpr bool debuggerPresent() { return true; }

struct DebuggerPresent {
    [[nodiscard]] static constexpr bool operator()() noexcept { return debuggerPresent(); }
};
}   // namespace Kvasir::Chip
