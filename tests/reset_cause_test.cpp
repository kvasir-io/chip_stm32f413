// src/chip/ResetCause.hpp: the RCC_CSR decode (RM0430 Rev 8 6.3.24) and its mapping onto the SDK's ResetKind. All
// static_asserts: building is the test.
#include "ResetCause.hpp"

namespace {
using namespace Kvasir::PM;
using Kvasir::ResetKind;

constexpr CsrFlags none{};

// one flag alone
static_assert(decode({.lpwr = true}) == ResetCause::lowPower);
static_assert(decode({.wwdg = true}) == ResetCause::wwdg);
static_assert(decode({.iwdg = true}) == ResetCause::iwdg);
static_assert(decode({.sft = true}) == ResetCause::software);
static_assert(decode({.por = true}) == ResetCause::por);
static_assert(decode({.bor = true}) == ResetCause::bor);
static_assert(decode({.pin = true}) == ResetCause::pin);
static_assert(decode(none) == ResetCause::unknown);

// every internal reset drives NRST (6.1.2): PINRSTF next to it names the internal cause
static_assert(decode({.lpwr = true,
                      .pin  = true})
              == ResetCause::lowPower);
static_assert(decode({.wwdg = true,
                      .pin  = true})
              == ResetCause::wwdg);
static_assert(decode({.iwdg = true,
                      .pin  = true})
              == ResetCause::iwdg);
static_assert(decode({.sft = true,
                      .pin = true})
              == ResetCause::software);
static_assert(decode({.bor = true,
                      .pin = true})
              == ResetCause::bor);

// the reset value 0x0E00'0000 (a power-on): PORRSTF, PINRSTF and BORRSTF -> por
static_assert(decode({.por = true,
                      .bor = true,
                      .pin = true})
              == ResetCause::por);

// the priority between internal causes (sticky flags of several resets before one RMVF)
static_assert(decode({.lpwr = true,
                      .wwdg = true,
                      .iwdg = true,
                      .sft  = true})
              == ResetCause::lowPower);
static_assert(decode({.wwdg = true,
                      .iwdg = true})
              == ResetCause::wwdg);
static_assert(decode({.iwdg = true,
                      .sft  = true,
                      .por  = true})
              == ResetCause::iwdg);
static_assert(decode({.sft = true,
                      .por = true})
              == ResetCause::software);

// the mapping, every enumerator
static_assert(toResetKind(ResetCause::lowPower) == ResetKind::lowPower);
static_assert(toResetKind(ResetCause::wwdg) == ResetKind::watchdogTimeout);
static_assert(toResetKind(ResetCause::iwdg) == ResetKind::watchdogTimeout);
static_assert(toResetKind(ResetCause::software) == ResetKind::softwareRequest);
static_assert(toResetKind(ResetCause::por) == ResetKind::powerOn);
static_assert(toResetKind(ResetCause::bor) == ResetKind::brownOut);
static_assert(toResetKind(ResetCause::pin) == ResetKind::external);
static_assert(toResetKind(ResetCause::unknown) == ResetKind::unknown);
}   // namespace

int main() { return 0; }
