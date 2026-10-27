#pragma once

#include <string_view>

// Which STM32F413 this firmware is built for: cmake/chip.cmake passes KVASIR_CHIP_STM32F413RG (with
// KVASIR_CHIP_STM32F413 and KVASIR_CHIP_FAMILY_STM32) to every TU of the image (kvasir_chip DEFINITIONS). One
// part so far; chip.hpp includes this first. The silicon says which in DBGMCU_IDCODE.DEV_ID, 0x463
// for the STM32F413/423 (RM0430 34.6.1).
namespace Kvasir { namespace Chip {
    enum class Part { stm32f413rg };

#ifdef KVASIR_CHIP_STM32F413RG
    inline constexpr Part part = Part::stm32f413rg;
#else
    #error \
      "chip_stm32f413 without a KVASIR_CHIP_STM32F413<part> define: not compiled with the package's kvasir_chip DEFINITIONS"
#endif

    // which part this image is for, as text
    inline constexpr std::string_view partName = "STM32F413RG";
}}   // namespace Kvasir::Chip
