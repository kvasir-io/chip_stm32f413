#pragma once

#include "ResetCause.hpp"
#include "kvasir/Port/Hooks.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/RCC.hpp"

#include <cstdint>
#include <utility>

// resetCause() is latched once per run: the first call reads RCC_CSR's flags in one access and writes RMVF (so the
// flags do not add up over resets, RM0430 6.3.24), every later call returns what the first one saw - the boot guard
// (a Startup entry) and Boot::logBoot (main) both read it. Call it after initMemory() only, never from
// coreClockInit(): the latch is in .bss, and the memory init would wipe it after the flags were already cleared.
namespace Kvasir { namespace PM {
    namespace detail {
        inline std::uint8_t latchedCause{};   // .bss: 0 unread, else ResetCause + 1
    }

    inline ResetCause resetCause() {
        if(detail::latchedCause == 0) {
            using CSR          = Kvasir::Peripheral::RCC::Registers<>::CSR;
            auto const       r = apply(read(CSR::lpwrrstf),
                                       read(CSR::wwdgrstf),
                                       read(CSR::wdgrstf),   // IWDGRSTF; the SVD names it WDGRSTF
                                       read(CSR::sftrstf),
                                       read(CSR::porrstf),
                                       read(CSR::borrstf),
                                       read(CSR::padrstf));   // PINRSTF; the SVD names it PADRSTF
            ResetCause const cause = decode({.lpwr = r[CSR::lpwrrstf] != 0,
                                             .wwdg = r[CSR::wwdgrstf] != 0,
                                             .iwdg = r[CSR::wdgrstf] != 0,
                                             .sft  = r[CSR::sftrstf] != 0,
                                             .por  = r[CSR::porrstf] != 0,
                                             .bor  = r[CSR::borrstf] != 0,
                                             .pin  = r[CSR::padrstf] != 0});
            apply(set(CSR::rmvf));
            detail::latchedCause = static_cast<std::uint8_t>(std::to_underlying(cause) + 1);
        }
        return static_cast<ResetCause>(detail::latchedCause - 1);
    }

    /// resetCause() mapped onto the SDK's ResetKind (ResetCause.hpp toResetKind), for the boot guard.
    inline ResetKind resetKind() { return toResetKind(resetCause()); }
}}   // namespace Kvasir::PM

/// resetCause() as the port's hook (kvasir/Port/Hooks.hpp), for the boot report:
/// Port::LateT<Port::ResetCause<Tag::User>, Dep>{}().
template<typename... Ts>
struct Kvasir::Port::ResetCause<Kvasir::Tag::User, Ts...> {
    Kvasir::PM::ResetCause operator()() const { return Kvasir::PM::resetCause(); }
};
