#pragma once

#include "kvasir/Port/ResetKind.hpp"

#include <cstdint>

// The reset cause from RCC_CSR (RM0430 Rev 8 6.3.24): the decode and its mapping onto the SDK's ResetKind, without
// registers (host-tested in tests/reset_cause_test.cpp); PM.hpp reads the flags.
//
// The flags are sticky: they add up over resets until software writes RMVF ("Cleared by writing to the RMVF bit",
// every flag of 6.3.24), so PM::resetCause() reads them once per run and clears them then.
//
// Every internal reset is driven out on the NRST pin ("The system reset signal provided to the
// device is output on the NRST pin", 6.1.2), so PINRSTF comes with all of them: pin only when no
// other flag is set. A POR sets BORRSTF too ("Set by hardware when a POR/PDR or BOR reset
// occurs"): por before bor.
namespace Kvasir { namespace PM {
    enum class ResetCause : std::uint8_t { lowPower, wwdg, iwdg, software, por, bor, pin, unknown };

    // RCC_CSR's reset flags, as read in one access (bits 31..25, RM0430 6.3.24)
    struct CsrFlags {
        bool lpwr{};   // LPWRRSTF
        bool wwdg{};   // WWDGRSTF
        bool iwdg{};   // IWDGRSTF
        bool sft{};    // SFTRSTF
        bool por{};    // PORRSTF
        bool bor{};    // BORRSTF
        bool pin{};    // PINRSTF
    };

    [[nodiscard]] constexpr ResetCause decode(CsrFlags const& f) {
        return f.lpwr ? ResetCause::lowPower
             : f.wwdg ? ResetCause::wwdg
             : f.iwdg ? ResetCause::iwdg
             : f.sft  ? ResetCause::software
             : f.por  ? ResetCause::por
             : f.bor  ? ResetCause::bor
             : f.pin  ? ResetCause::pin
                      : ResetCause::unknown;
    }

    // resetCause() as the SDK's boot guard sees it (kvasir/Port/ResetKind.hpp). A debugger's reset is a pin or a
    // software reset, depending on the probe's reset strategy: without a record the guard counts neither.
    [[nodiscard]] constexpr ResetKind toResetKind(ResetCause c) {
        switch(c) {
        // Standby or Stop entered with nRST_STDBY / nRST_STOP cleared in the option bytes (6.1.1)
        case ResetCause::lowPower: return ResetKind::lowPower;
        // a WWDG reset is a count that ran out or a refresh outside the window: a failed refresh
        case ResetCause::wwdg:
        case ResetCause::iwdg:     return ResetKind::watchdogTimeout;
        case ResetCause::software: return ResetKind::softwareRequest;   // AIRCR.SYSRESETREQ (6.1.1)
        case ResetCause::por:      return ResetKind::powerOn;
        case ResetCause::bor:      return ResetKind::brownOut;
        case ResetCause::pin:      return ResetKind::external;
        case ResetCause::unknown:  return ResetKind::unknown;
        }
        return ResetKind::unknown;
    }
}}   // namespace Kvasir::PM
