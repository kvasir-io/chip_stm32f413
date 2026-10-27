#pragma once
// Every driver of the STM32F413, opt-in: chip/chip.hpp is the chip without them. A firmware includes the drivers it
// uses by name (chip/<Driver>.hpp) or all of them here.
#include "chip.hpp"
//
#include "Adc.hpp"
#include "Can.hpp"
#include "ClockRecovery.hpp"
#include "Exti.hpp"
#include "Flash.hpp"
#include "I2c.hpp"
#include "Spi.hpp"
#include "Timer.hpp"
#include "Usart.hpp"
