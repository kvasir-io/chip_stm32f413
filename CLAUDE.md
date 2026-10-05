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
