# chip_stm32f413

Kvasir chip package for the STMicroelectronics STM32F413RG (Cortex-M4F, 1 MiB flash, 320 KiB SRAM,
LQFP64). Like every chip package this is a CMake
include()-fragment: `cmake/chip.cmake` is pulled in by the Kvasir SDK through `CHIP_ROOT`, nothing
builds here (`just` lists the upkeep recipes).

## Layout

| path | what |
|---|---|
| `chip.svd` | ST's `STM32F413.svd` V2.0 (from the sodaCat SVD collection's `svd/ST/stm32f4-svd.zip`, LF line endings), with hand edits marked `<!-- Kvasir: RM0430 Rev 8 x.y.z ... -->` (below) |
| `cmake/chip.cmake` | part, memory sizes, J-Link device `STM32F413RG`, UF2 family, the 4-priority-bit core SVD |
| `linker/chip.ld` | flash 0x0800 0000 (1024K), SRAM1+SRAM2 0x2000 0000 (320K) |
| `core` | submodule `core_cortex_m4` (linked to the top-level checkout by `just link`) |
| `src/chip/` | the drivers (below) |
| `tests/` | host tests of `ClockMath.hpp` (incl. the CAN bit timing, `can_timing`), `Instances.hpp` (+ `instances_must_fail`), `ResetCause.hpp`, `FlashLayout.hpp` (+ its must-fail cases), `OdrToggle.hpp` (on the SDK's register mock), `UsartQueue.hpp` (static_asserts and run-time checks) and the SVD's write-only guard; `tests/conformance/` the conformance tree |

## Drivers

Every hardware fact in them cites RM0430 Rev 8, DS11581 Rev 7 or ES0372 Rev 8.

| header | what |
|---|---|
| `chip.hpp` | the chip core: Variant, Rcc, PM, Io, Watchdog, Interrupt, Reboot, DebuggerPresent, StartUp and the peripherals they use - no driver |
| `drivers.hpp` | `chip.hpp` + every driver (Adc, Can, Exti, Flash, I2c, Spi, Timer, Usart); or include a driver by name |
| `peripherals.hpp` | every generated peripheral header of the part |
| `StartUp.hpp` | FPU on (core), clocks of every bonded GPIO port, DSB (errata 2.2.7) |
| `Interrupt.hpp` | the 102 vectors from chip.svd (svd_converter's `peripherals/Interrupts.hpp`; chip.svd adds WWDG, EXTI18_OTG_FS_WKUP, OTG_FS and QUADSPI from RM0430 Table 40), 79 (AES, F423 only) disabled |
| `Io.hpp` | GPIO: input, output, analog, alternate function (AF number from DS11581 Table 12), set/clear via BSRR, toggle via ODR |
| `Rcc.hpp` | `ClockTree<crystal, sysclk>`: PLL solved at compile time, regulator scale, flash wait states + caches, APB prescalers, `pclkHz<Bus>` / `timerHz<Bus>` / `supply` for the drivers; `init()` never hangs and writes registers only (every wait bounded, `OnTimeout::Report`; HSE failure -> stays on HSI; `Rcc::initResult()` reads back how far it got), `switchToPll()` its second half; `Rcc::OnBusOf<Regs>`, the instance rows' base: the bus from the block's base address (`Chip::busOfBase`), the enable bit its Blocks.hpp row (which must be in that bus's RCC_APBxENR) |
| `ClockRecovery.hpp` | `Rcc::Recovery<Tree, Clock>`, a `Hook::MainLoop` entry: while `initResult()` is not `ok`, restarts the HSE / switches to the PLL on a doubling pause (100 ms up to 10 s) - an HSE that did not start at boot is no latch; `recoveries()`, `attempts()` |
| `Can.hpp` | `Can::Bus<Clocks, 1, bitrate, Rx, Tx, Af, TimeClock>`: bxCAN1 polled - bit timing solved at compile time, filter bank 0 = everything to FIFO 0, `send`/`receive`/`status`, bus-off left by the hardware (ABOM); an initialization without INAK is retried by the bus's `Hook::MainLoop` entry `turn()` on a doubling pause (`Kvasir::Backoff` on `TimeClock`) |
| `ClockMath.hpp` | the arithmetic of the above and of the dividers below, the bxCAN bit timing solver (`Can::solveBitTiming`), register-free for the host tests |
| `Instances.hpp` | `Rcc::Bus`, the bus of a peripheral block by its base address (`Chip::busOfBase`, `onApb1`/`onApb2`/`onAhb1`; RM0430 Table 1) and the SPI limits per supply and direction (`Chip::spiMaxSckHz`, DS11581 Table 67), register-free for the host tests; `Rcc::OnBusOf<Regs>` (Rcc.hpp) is its register half |
| `PM.hpp` | `PM::resetCause()` from RCC_CSR, latched once per run (the first call clears the flags; call it after the memory init); `PM::resetKind()` for the SDK's boot guard |
| `ResetCause.hpp` | the RCC_CSR decode and its `ResetKind` mapping, register-free for the host tests |
| `Timer.hpp` | `Tick` (update interrupt at a rate), `Pwm` (channels 1-2), `Counter` (free-running 32 bit on TIM2/5) |
| `Exti.hpp` | `Exti::Line<Pin, Edge, &fn>`: a GPIO edge as a sub-interrupt of the shared EXTI vectors |
| `Spi.hpp` | polled master, 8/16-bit frames, CS by the caller; `send` + `waitIdle` for GPIO chip selects (RM0430 29.3.10) |
| `Usart.hpp` | USART1/2/3/6, interrupt-driven with lock-free RX/TX queues, baud checked at compile time |
| `I2c.hpp` | polled standard-mode master for I2C1-3: register write, register read of 1-2 bytes; bounded waits, SWRST recovery |
| `Adc.hpp` | ADC1, one channel, software-started single conversions |
| `Watchdog.hpp` | the independent watchdog as `Kvasir::Watchdog<Config>` (the SDK's gated front), sized for the fastest LSI; frozen while the debugger halts the core; no disarm |
| `Flash.hpp` | `Kvasir::Nvm`: sector erase and word programming of the main flash (x32); `Nvm::Region<Base, Sectors>`, sectors of one size as a `Kvasir::Hal::NorRegion` for CellStore (sector map and its checks: `FlashLayout.hpp`, host-tested) |

## Startup resources

Every check is a compile error naming the resource (the SDK's `kvasir/StartUp/Resources.hpp`), always on:

| resource | provided by | claimed by |
|---|---|---|
| a pin, `Io pin <port>.<pin>` | whoever writes its MODER field (`Io.hpp` `ResourceOfAction`; every pin action ends there) | - |
| an EXTI line (`Exti::ExtiLineTag`) | `Exti::Line` (one port per line, SYSCFG_EXTICRx) | - |
| a timer, `(TIMx, PSC, ARR)` (`Timer::TimerTag`) | `Tick`, `Pwm`, `Counter`; two users with equal PSC/ARR merge | - |
| a timer channel, `(TIMx, channel)` (`Timer::TimerChannelTag`) | `Pwm`, exclusive | - |
| a peripheral block (`Hal::Owns<Regs>`) | `Spi::Master`, `I2c::Master`, `Usart::Port`, `Can::Bus`, `Adc::Single` (ADC1) | - |
| the processor clock, PCLK1/PCLK2 (`Kvasir::Clocks::ClkApb1` / `ClkApb2`), the timer kernel clocks (`ClkTim1` / `ClkTim2`, `Clocks.hpp`) | `Rcc::ClockTree::Provides`, through the firmware's ClockSettings | every driver, at the rate it computed its dividers from (`ClockTree::pclkHz<Bus>` / `timerHz<Bus>`) |

A firmware's ClockSettings provides the tree its drivers use:

```cpp
struct ClockSettings {
    using Provides = ClockTree::Provides;
    static void coreClockInit() { ClockTree::init(); }
};
```

A ClockSettings without it, or a driver handed another tree, does not build (rule 3 of the SDK's Resources.hpp).
Not resources (shared by design): the SYSCFG and EXTI clock enables, the GPIO port clocks (`StartUp.hpp`).

## State

- Built warning-free in all four variants through a firmware's examples (blink ...
  watchdog); host tests pass with gcc and clang. Has run on hardware; not every driver has been checked there.
- Not written yet: DMA, TIM1/TIM8 (advanced timers), PWM channels 3-4, ADC channels 10-18 and
  sequences, UART4/5/7-10, FMPI2C, a queued
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
