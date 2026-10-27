#pragma once
// The chip itself: variant, clocks (Rcc), power, interrupts, Io, start-up, reset cause, reboot and the watchdog.
// No driver: include a driver by name (chip/I2c.hpp, ...) or all of them through chip/drivers.hpp, and a generated
// peripheral by name (peripherals/TIM2.hpp) or all of them through chip/peripherals.hpp. Every header includes the
// peripherals it uses.
// Any number of an image's TUs may include it, and every other chip header: the Startup itself (vector table, reset
// entry, the runtime-library stand-ins) is kvasir/StartUp/StartUp.hpp, which the main TU alone includes.
#include "Variant.hpp"
//
#include "peripherals/DBG.hpp"
#include "peripherals/FLASH.hpp"
#include "peripherals/GPIO.hpp"
#include "peripherals/GPIOA.hpp"
#include "peripherals/GPIOB.hpp"
#include "peripherals/IWDG.hpp"
#include "peripherals/PWR.hpp"
#include "peripherals/RCC.hpp"
//
#include "DebuggerPresent.hpp"
#include "Interrupt.hpp"
#include "Io.hpp"
#include "PM.hpp"
#include "Rcc.hpp"
#include "Reboot.hpp"
#include "Watchdog.hpp"
//
#include "StartUp.hpp"
#include "core/core.hpp"
