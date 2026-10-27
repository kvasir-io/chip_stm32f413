# chip_stm32f413: notes for Claude

The lab notebook of this chip package (kvasir_work/CLAUDE.md has the bench-wide rules). Add what you
learn here, dated, in the same turn.

## Documents (all in ~/datasheeds/mcu/)

- `RM0430_stm32f413_423_reference_manual` (Rev 8, May 2018 - a mirror; ST's Rev 9 is newer),
  `DS11581_stm32f413xg_xh_datasheet` (Rev 7), `ES0372_stm32f413_423_errata` (Rev 8, April 2026 -
  the F413 errata is ES0372, ES0430 is the G4's), `PM0214_stm32_cortex_m4_programming_manual`.
- st.com and analog.com do not answer curl from this machine (2026-10-02); the Chrome extension can
  fetch them, but Chrome allows one automatic download per site and then blocks - dominic downloaded
  the rest by hand.
- The converted RM tables are often mangled (merged columns, empty leading cells lost by a naive
  `strip('|')`). Read a register's access row on the PDF page: `pdftotext -layout -f N -l N`. The
  datasheet's alternate-function table is Table 12 (PDF pages 66-72) and only readable that way.

## SVD (2026-10-02)

- `chip.svd` came from `~/repos/sodaCat/svd/ST/stm32f4-svd.zip` (README.md names the collection only since
  2026-10-21: no local paths in library files).
- ST's SVD has no write semantics, splits multi-bit fields into bits, and derives peripherals across
  name stems (USART3 is the base of UART4..10, GPIOF of GPIOC..H). README.md "Hand edits" lists
  what was changed; svd_converter handles the derivation since 2026-10-02.
- Generated groups: `USART` (1,2,3,6), `UART` (4,5,7..10), `GPIO` (C..H as 2..7; GPIOA/B are
  separate peripherals), `SPI` (1..5), `I2C` (1..3), `TIM1_8`, `TIM3_4`, `TIM6_7`, `TIM13_14`, the
  other timers by name, `ADC1` and `ADC_COMMON`.
- `apply(read(X::FULLREGISTER))` gives a FieldTuple: `get<0>(...)` for the number.
- `write(field, value<V>())` shifts V into the field itself; a hand-made FieldLocation over a
  generated `Addr` (EXTI per line, ADC SMPR per channel) inherits the register's write masks.

## Decisions

- GPIO port clocks: all bonded ports (A, B, C, D, H) are enabled in `FirstInitStep`, before any pin
  action - simpler than ordering an RCC enable before each pin's writes, and costs nothing visible.
- Clock init never spins forever: the bench's SAM boards taught that a hang in clock init leaves no
  log (kvasir_work/CLAUDE.md "the startup never got that far").
- Flash: x32 parallelism (2.7-3.6 V); an erase stalls every flash read, interrupts included, for up
  to 2 s (128 KiB sector).
- I2C: standard mode below 88 kHz (errata 2.11.4), BERR ignored (2.11.1), SWRST after a stuck bus
  (2.11.3).
- No kvasir_devices dependency (2026-10-18, plans/refactor_analysis 0.20): no header under `src/` includes
  `kvasir/Devices/...`, so `cmake/chip.cmake` no longer registers the package and the Dockerfile no longer clones it.
  A firmware that uses a driver registers it itself (`kvasir_resolve_root` + `kvasir_add_package` between
  `include(kvasir.cmake)` and `project()`; psu302 does) - the other five chips still bring it until 6.1.
- IWDG gate (2026-10-05; since 2026-10-24 `Kvasir::Watchdog<Config>` with `gatedFeed`, chip/Watchdog.hpp, on the SDK's
  `Health::WatchdogFront`; `Iwdg::Watchdog` and `start()` are gone - `arm()`): the gated IWDG is fed and started only by
  `Kvasir::Health::Supervisor` (`arm(FeedKey)`, `feed(FeedKey)`; the plain `arm()` and `feed()` do not
  compile: conformance ctest `watchdog_must_fail_{1,2,3}`, 3 = a `disarm()`). An IWDG cannot be stopped (RM0430 22.4.1), so the supervisor has no `holdOff()` over it and the
  watchdog only runs once the supervisor's first `service()` started it. Ungated (`Watchdog<ms>`, psu302) is
  unchanged: release images byte-identical. The firmware-tree check (`tests/firmware/iwdg_gate.cpp`,
  `fw_iwdg_gate`) was removed 2026-10-07 (dominic: no firmware names in library repos); only a psu302 build checks
  the gate now. Not run on a board.
- Field factories (refactor 4.11): a one-bit literal write is `set(f)` / `clear(f)`, the flag's semantics in the
  verb: `clear` for rc_w0 (TIMx_SR UIF, I2C_SR1 BERR/AF/ARLO; `zeroToClear` in chip.svd), `reset` for rc_w1 where
  one is ever written as a field (none today: EXTI PR goes through `Nvic::ClearFirst`, FLASH_SR is written whole),
  `set` for oneToSet (RCC_CSR RMVF, FLASH_ACR ICRST/DCRST) and inside `overrideDefaults` for CAN RF0R FOVR0/RFOM0
  (unmarked in chip.svd: plain fields). `write(f, value<...>())` stays for multi-bit fields, conditional values
  (`C ? 1U : 0U`) and a 0 into a oneToSet field (`clear` refuses those). Images byte-identical.
- One instance table (refactor 4.11): `Instances.hpp` (bus by base address, RM0430 Table 1, since step 148; SPI limits)
  + `Rcc::OnBusOf<Regs>` rows in each driver (the Timer's `Shape<Regs, ...>` is one); `ClockTree::pclkHz<Bus>` / `timerHz<Bus>` / `supply`. SPI limit = DS11581 Table 67 as far as
  `Supply` says it (Q-4.11-A): SPI1/4/5 receiving 42 MHz at `from2V7`, 25 MHz below, transmit-only 50 MHz;
  SPI2/3 25 MHz. Startup resources: README.md "Startup resources". CAN `Bus::turn()` (its `Hook::MainLoop` entry, dominic 2026-10-19) retries a failed
  initialization (Q-4.11-E, "every error recovers by itself"); not run on a board yet.
