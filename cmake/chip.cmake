# STM32F413RG: Cortex-M4F, 1 MiB flash, 320 KiB SRAM, LQFP64. The memory sizes are DS11581 Rev 7 (Table 2,
# "STM32F413xG/H features and peripheral counts"; flash at 0x08000000) and RM0430 Rev 8 2.4 (SRAM1 0x2000 0000 - 0x2003
# FFFF and SRAM2 0x2004 0000 - 0x2004 FFFF: one contiguous 320 KiB block on the S-bus).

# The F4 implements 4 NVIC priority bits (PM0214 4.3.7, RM0430 10.1): core_cortex_m4's core_prio4.svd.
set(CORE_NVIC_PRIO_BITS 4)
include(${CMAKE_CURRENT_LIST_DIR}/../core/cmake/core.cmake)

# Kvasir's write semantics (svd_convert's defaults): a new unclassified write-only field stops the build
kvasir_chip_svd(peripherals ${CMAKE_CURRENT_LIST_DIR}/../chip.svd)

# the chip's part of the contract (Kvasir_SDK cmake/kvasir_chip.cmake)
kvasir_chip(
    # also the J-Link device name (`JLinkExe -device`)
    MPU
    STM32F413RG
    # the SDK writes a UF2 next to every image; github.com/microsoft/uf2 utils/uf2families.json: STM32F4 = 0x57755a57
    UF2_FAMILY
    0x57755a57
    LINKER_FILE
    ${CMAKE_CURRENT_LIST_DIR}/../linker/chip.ld
    INCLUDE_DIR
    ${CMAKE_CURRENT_LIST_DIR}/../src
    REGISTERS
    peripherals
    # what src/chip/Variant.hpp reads: the part, the series and the family, for every TU of the image; and the chip's
    # Register::ExecuteSeam (Kvasir_SDK Register/Exec.hpp includes it at its end): a GPIO ODR toggle as an ODR read and
    # one BSRR write, src/chip/OdrToggle.hpp
    DEFINITIONS
    KVASIR_CHIP_STM32F413RG=1
    KVASIR_CHIP_STM32F413=1
    KVASIR_CHIP_FAMILY_STM32=1
    [[KVASIR_CHIP_REGISTER_SEAM="chip/OdrToggle.hpp"]])
