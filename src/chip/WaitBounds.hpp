#pragma once
// The bounds of the STM32F413 package's register waits (Kvasir_SDK kvasir/Register/Wait.hpp), each with its source.
// Under the default wait policy (Unbounded) every site is the loop it was.
#include "kvasir/Register/Wait.hpp"

#include <cstdint>

namespace Kvasir::Chip {
// HCLK max 100 MHz (RM0430 Table 6, ClockMath.hpp): Bound::Microseconds converts at this
inline constexpr std::uint64_t CpuHzCeiling = 100'000'000;

template<std::uint64_t Us>
using MicrosecondsBound = Kvasir::Register::Bound::Microseconds<Us, CpuHzCeiling>;

// A read back of a register just written (FLASH.ACR.LATENCY): a bus round trip.
using ReadBackBound = Kvasir::Register::Bound::Polls<100>;
// FLASH.SR.BSY: a 128 KB sector erase at x32 parallelism takes 2 s max (DS11581 Table 51, md l.4493-4494); the
// driver programs x32 (Flash.hpp)
using FlashBusyBound = MicrosecondsBound<4'000'000>;
}   // namespace Kvasir::Chip
