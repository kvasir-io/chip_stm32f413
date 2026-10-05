/* Linker script for the STM32F413RG (Cortex-M4F): RM0430 Rev 8 2.4 / DS11581 Rev 7 Table 2 (features and peripheral counts).
   Main flash at 0x08000000 (the boot ROM maps it at 0 with BOOT0 low and fetches the vector table from there);
   SRAM1 + SRAM2 as one contiguous block. */

MEMORY{
    flash  (xr ) : ORIGIN = 0x08000000, LENGTH = 1024K
    ram    (xrw) : ORIGIN = 0x20000000, LENGTH = 320K
}

INCLUDE common_flash.ld
INCLUDE common_ram.ld

INCLUDE common.ld
