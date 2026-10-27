#pragma once
#include "core/CoreInterrupts.hpp"
#include "kvasir/Common/Interrupt.hpp"
#include "kvasir/Port/Interrupts.hpp"
#include "peripherals/Interrupts.hpp"

#include <array>
#include <type_traits>

namespace Kvasir {
// The core's exceptions (core/CoreInterrupts.hpp) and the device lines of this part's SVD (svd_converter:
// peripherals/Interrupts.hpp, the SVD's names lower-cased without IRQ).
// chip.svd carries the lines RM0430 Rev 8 Table 40 (10.2, md l.11181) has and ST's SVD lacked (WWDG, EXTI18_OTG_FS_WKUP,
// OTG_FS, QUADSPI); the names are the SVD's (i2c1_evt, usart4, tim6_glb_it_dac1_dac2, dfsdm2_filter1..4, ...).
struct Interrupt
  : CoreInterrupts
  , Peripheral::DeviceInterrupts {
    // The vector that serves EXTI line n (Table 40): lines 0 to 4 have one each, 5 to 9 share
    // exti9_5, 10 to 15 share exti15_10.
    template<unsigned Line>
    using ExtiVector
      = Type<(Line <= 4   ? std::remove_cvref_t<decltype(exti0)>::value + static_cast<int>(Line)
              : Line <= 9 ? std::remove_cvref_t<decltype(exti9_5)>::value
                          : std::remove_cvref_t<decltype(exti15_10)>::value)>;
};

namespace Nvic {
    // the core's reserved slots, the positions Table 40 leaves out, and 79 (AES, the SVD's CRYPTO):
    // "Only available for STM32F423xx" (Table 40 note 1)
    template<>
    struct InterruptOffsetTraits<Kvasir::Tag::User>
      : Core::ChipInterruptTraits<Core::InterruptRules,
                                  Peripheral::interruptEnd,
                                  Peripheral::interruptGaps,
                                  std::array{79}> {};
}   // namespace Nvic
}   // namespace Kvasir
