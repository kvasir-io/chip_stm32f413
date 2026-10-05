# STM32F413RG: Cortex-M4F, 1 MiB flash, 320 KiB SRAM, LQFP64. The memory sizes are DS11581 Rev 7 (Table 2,
# "STM32F413xG/H features and peripheral counts"; flash at 0x08000000) and RM0430 Rev 8 2.4 (SRAM1 0x2000 0000 - 0x2003
# FFFF and SRAM2 0x2004 0000 - 0x2004 FFFF: one contiguous 320 KiB block on the S-bus).

# The F4 implements 4 NVIC priority bits (PM0214 4.3.7, RM0430 10.1): core_cortex_m4's core_prio4.svd.
set(CORE_NVIC_PRIO_BITS 4)
include(${CMAKE_CURRENT_LIST_DIR}/../core/cmake/core.cmake)

# Also the J-Link device name (`JLinkExe -device`).
set(TARGET_MPU STM32F413RG)
set(TARGET_FLASH_SIZE 1048576)
set(TARGET_RAM_SIZE 327680)
set(TARGET_EEPROM_SIZE 0)

# The SDK writes a UF2 next to every image; github.com/microsoft/uf2 utils/uf2families.json: STM32F4 = 0x57755a57.
set(TARGET_UF2_CODE 0x57755a57)

set(LINKER_FILE ${CMAKE_CURRENT_LIST_DIR}/../linker/chip.ld)

# The write-only guard: every write-only field of the SVD is classified (oneToSet, a key, or <!-- Kvasir: write-only
# accepted -->), so a new one stops the build; registers with no readable field are never read
svd_convert(
    peripherals
    SVD_FILE
    ${CMAKE_CURRENT_LIST_DIR}/../chip.svd
    OUTPUT_DIRECTORY
    peripherals
    WRITE_ONLY_GUARD
    error
    WRITE_ONLY_REGISTERS
    derived
    WRITE_ONLY_MASK
    ON)

# kvasir_devices: the I2C driver includes kvasir/Devices/I2C/LineRecovery.hpp, so every image needs it. Found like
# CHIP_ROOT (KVASIR_DEVICES_ROOT: variable, environment, else next to the SDK); the SDK adds it after project() unless
# the firmware has it already.
kvasir_resolve_root(KVASIR_DEVICES_ROOT kvasir_devices)
kvasir_add_package(${KVASIR_DEVICES_ROOT} kvasir_devices kvasir_devices)
target_link_libraries(peripherals INTERFACE kvasir::devices)
