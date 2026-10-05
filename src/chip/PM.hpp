#pragma once

#include "kvasir/Register/Register.hpp"
#include "peripherals/RCC.hpp"

#include <cstdint>

// The reset cause from RCC_CSR (RM0430 Rev 8 6.3.24). The flags are sticky: they add up over
// resets until software writes RMVF, so reset_cause() reads them and clears them - call it once
// per boot (Boot::logBoot does).
//
// Every internal reset is driven out on the NRST pin ("The system reset signal provided to the
// device is output on the NRST pin", 6.1.2), so PINRSTF comes with all of them: pin only when no
// other flag is set. A POR sets BORRSTF too ("Set by hardware when a POR/PDR or BOR reset
// occurs"): por before bor.
namespace Kvasir { namespace PM {
    enum class ResetCause : std::uint8_t { lowPower, wwdg, iwdg, software, por, bor, pin, unknown };

    inline ResetCause reset_cause() {
        using CSR             = Kvasir::Peripheral::RCC::Registers<>::CSR;
        auto const       flag = [](auto field) { return apply(read(field)) != 0; };
        ResetCause const cause
          = flag(CSR::lpwrrstf) ? ResetCause::lowPower
          : flag(CSR::wwdgrstf) ? ResetCause::wwdg
          : flag(CSR::wdgrstf)  ? ResetCause::iwdg   // IWDGRSTF; the SVD names it WDGRSTF
          : flag(CSR::sftrstf)  ? ResetCause::software
          : flag(CSR::porrstf)  ? ResetCause::por
          : flag(CSR::borrstf)  ? ResetCause::bor
          : flag(CSR::padrstf)  ? ResetCause::pin   // PINRSTF; the SVD names it PADRSTF
                                : ResetCause::unknown;
        apply(write(CSR::rmvf, Register::value<std::uint32_t, 1>()));
        return cause;
    }
}}   // namespace Kvasir::PM
