# chip_stm32f413

Kvasir chip package for the STMicroelectronics STM32F413RG (Cortex-M4F, 1 MiB flash, 320 KiB SRAM,
LQFP64). Like every chip package this is a CMake
include()-fragment: `cmake/chip.cmake` is pulled in by the Kvasir SDK through `CHIP_ROOT`, nothing
builds here (`just` lists the upkeep recipes).

## Layout

| path | what |
|---|---|
| `chip.svd` | ST's `STM32F413.svd` V2.0 (from `~/repos/sodaCat/svd/ST/stm32f4-svd.zip`, LF line endings), with hand edits marked `<!-- Kvasir: RM0430 Rev 8 x.y.z ... -->` (below) |
| `cmake/chip.cmake` | part, memory sizes, J-Link device `STM32F413RG`, UF2 family, the 4-priority-bit core SVD |
| `linker/chip.ld` | flash 0x0800 0000 (1024K), SRAM1+SRAM2 0x2000 0000 (320K) |
| `core` | submodule `core_cortex_m4` (linked to the top-level checkout by `just link`) |
| `src/chip/` | the drivers (below) |
| `tests/` | host tests of `ClockMath.hpp` (static_asserts) and of the core SVD |

## Drivers

Every hardware fact in them cites RM0430 Rev 8, DS11581 Rev 7 or ES0372 Rev 8 (all in
`~/datasheeds/mcu/`).

| header | what |
|---|---|
| `chip.hpp` | everything below plus the generated peripheral headers |
| `StartUp.hpp` | FPU on (core), clocks of every bonded GPIO port, DSB (errata 2.2.7) |
| `Interrupt.hpp` | the 102 vectors of RM0430 Table 40 (not the SVD's list: it misses WWDG, OTG_FS, QUADSPI) |
| `Io.hpp` | GPIO: input, output, analog, alternate function (AF number from DS11581 Table 12), set/clear via BSRR, toggle via ODR |
| `Rcc.hpp` | `ClockTree<crystal, sysclk>`: PLL solved at compile time, regulator scale, flash wait states + caches, APB prescalers; `init()` never hangs (HSE failure -> stays on HSI; `Rcc::initResult()` reads back how far it got) |
| `Can.hpp` | `Can::Bus<Clocks, 1, bitrate, Rx, Tx, Af>`: bxCAN1 polled - bit timing solved at compile time, filter bank 0 = everything to FIFO 0, `send`/`receive`/`status`, bus-off left by the hardware (ABOM) |
| `ClockMath.hpp` | the arithmetic of the above and of the dividers below, register-free for the host tests |
| `PM.hpp` | `PM::reset_cause()` from RCC_CSR (and clears the flags) |
| `Timer.hpp` | `Tick` (update interrupt at a rate), `Pwm` (channels 1-2), `Counter` (free-running 32 bit on TIM2/5) |
| `Exti.hpp` | `Exti::Line<Pin, Edge, &fn>`: a GPIO edge as a sub-interrupt of the shared EXTI vectors |
| `Spi.hpp` | polled master, 8/16-bit frames, CS by the caller; `send` + `waitIdle` for GPIO chip selects (RM0430 29.3.10) |
| `Usart.hpp` | USART1/2/3/6, interrupt-driven with lock-free RX/TX queues, baud checked at compile time |
| `I2c.hpp` | polled standard-mode master for I2C1-3: register write, register read of 1-2 bytes; bounded waits, SWRST recovery |
| `Adc.hpp` | ADC1, one channel, software-started single conversions |
| `Iwdg.hpp` | the independent watchdog, sized for the fastest LSI; frozen while the debugger halts the core |
| `Flash.hpp` | sector erase and word programming of the main flash (x32) |

## State

- Built warning-free in all four variants through a firmware's examples (blink ...
  watchdog); host tests pass with gcc and clang. **Nothing has run on hardware yet**.
- Not written yet: DMA, TIM1/TIM8 (advanced timers), PWM channels 3-4, ADC channels 10-18 and
  sequences, UART4/5/7-10, FMPI2C, CAN (bxCAN), a queued
  interrupt-driven I2C/SPI policy for kvasir_devices' engines, RTC, USB, the RNG as the stack
  canary's entropy (`StackGuardEntropy`), a compile-time AF table (the AF is passed as a number today).
- `chip.svd` write semantics are marked only for the registers the drivers touch (FLASH_SR, EXTI_PR,
  ADC_SR, every TIMx_SR, I2C_SR1, USART_SR, SPI_SR, RCC_CIR/CSR, DMA LIFCR/HIFCR); DAC, RNG, WWDG,
  RTC, DFSDM, CAN, OTG are not done.

## Hand edits in chip.svd

Each is an XML comment `<!-- Kvasir: ... -->` next to the change, with its RM0430 section.

- `modifiedWriteValues`: rc_w0 flags `zeroToClear`, rc_w1 flags `oneToClear`, "a 1 clears another
  flag, a 0 does nothing" triggers `oneToSet` (157 fields). ST's SVD has none at all.
- RCC_CSR reset flags read-only (RM0430 6.3.24 says `r`; the SVD said read-write).
- ST splits multi-bit fields into one field per bit (`PLLM0`..`PLLM5`); merged back where the RM
  documents one field: RCC PLLCFGR PLLQ/PLLP/PLLN/PLLM, CFGR SW/SWS, BDCR RTCSEL, WWDG CFR WDGTB.
- Known and left: EXTI_PR has PR19/PR20 (reserved in the RM) and lacks PR23; DBGMCU_APB1_FZ calls
  IWDG_STOP `iwdeg_stop` and lacks the TIM6/7/12/13/14 stop bits; the 32-bit timers' CNT/ARR/CCRx
  are split into `_h`/`_l` halves (the drivers write `FULLREGISTER`).

## Depends on (uncommitted)

- `Kvasir_SDK/svd_converter`: reading ST's SVDs (instance numbers from each peripheral's own name).
- `core_cortex_m4`: `core_prio4.svd` and `CORE_NVIC_PRIO_BITS` in `cmake/core.cmake`. The submodule
  records a core commit without them: CI (tests.yml) checks out that commit and fails until the
  core change is committed and the submodule bumped.
- `Kvasir_SDK/cmake/util.cmake`: `LINKER_FILE` in `kvasir_executable_variants` (a firmware's own
  linker script).

The J-Link device is `STM32F413RG` (in `libjlinkarm.so`'s list). The UF2 family id 0x57755a57 is
the registry's STM32F4.
