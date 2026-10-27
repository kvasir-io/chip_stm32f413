// The drivers' register writes, instantiated once each (one representative instance and configuration per driver
// template): an explicit instantiation compiles every member function, so every dependent apply() of the driver is
// checked against its register's write-only rules (Register::Semantics: keys, reads with side effects) as the
// conformance tree converts the SVD (guard error, registers derived, mask on). Nothing here is called; what needs a
// board's configuration to be instantiated at all is in not_instantiated.txt.

// the config header first: the chip below needs its log backend and log clock (uc_log.hpp would read it too, but the
// chip headers before it need the same)
#include "kvasir_config.hpp"

// then the chip and the drivers
#include "chip/Adc.hpp"
#include "chip/Can.hpp"
#include "chip/ClockRecovery.hpp"
#include "chip/Exti.hpp"
#include "chip/Flash.hpp"
#include "chip/I2c.hpp"
#include "chip/Rcc.hpp"
#include "chip/Spi.hpp"
#include "chip/Timer.hpp"
#include "chip/Usart.hpp"
#include "chip/Watchdog.hpp"
#include "chip/chip.hpp"

#include <chrono>
#include <cstdint>

namespace Conformance {
using Kvasir::Io::pin1;
using Kvasir::Io::pin11;
using Kvasir::Io::pin12;
using Kvasir::Io::pin5;
using Kvasir::Io::pin6;
using Kvasir::Io::pin7;
using Kvasir::Io::pin8;
using Kvasir::Io::pin9;
using Kvasir::Io::portA;
using Kvasir::Io::portB;
using Kvasir::Io::portC;

// 8 MHz crystal, 100 MHz core, APB2 at 50 MHz (a receiving SPI wants APB <= 84 MHz, errata ES0372 2.14.4)
using ClockTree = Kvasir::Rcc::
  ClockTree<8'000'000, 100'000'000, Kvasir::Rcc::Supply::from2V7, false, false, 50'000'000>;

using SpiSck  = decltype(makePinLocation(portA, pin5));
using SpiMiso = decltype(makePinLocation(portA, pin6));
using I2cScl  = decltype(makePinLocation(portA, pin8));
using I2cSda  = decltype(makePinLocation(portC, pin9));
using UartTx  = decltype(makePinLocation(portB, pin6));
using UartRx  = decltype(makePinLocation(portB, pin7));
using AdcIn   = decltype(makePinLocation(portA, pin1));
using PwmOut  = decltype(makePinLocation(portB, pin5));
using CanRx   = decltype(makePinLocation(portA, pin11));
using CanTx   = decltype(makePinLocation(portA, pin12));

inline void onEdge() {}

inline void onTick() {}

// the watchdog, gated (the health supervisor's) and ungated (a Startup entry)
struct WatchdogConfig {
    static constexpr auto overrunTime = std::chrono::seconds{1};
    static constexpr bool gatedFeed   = true;
};

struct UngatedWatchdogConfig {
    static constexpr auto overrunTime = std::chrono::seconds{3};
};
}   // namespace Conformance

// pins and alternate functions: SPI1 AF5 on PA5/PA6, I2C3 AF4 on PA8/PC9, USART1 AF7 on PB6/PB7, TIM3 AF2 on PB5,
// CAN1 AF9 on PA11/PA12
// (STM32F413 data sheet DS11581, Table 12 "STM32F413xG/H alternate functions")
template struct Kvasir::Spi::Master<Conformance::ClockTree,
                                    1,
                                    Kvasir::Spi::Config{.maxHz = 2'000'000},
                                    Conformance::SpiSck,
                                    Conformance::SpiMiso,
                                    Kvasir::Io::NotUsed<>,
                                    5>;
template struct Kvasir::I2c::
  Master<Conformance::ClockTree, 3, 50'000, Conformance::I2cScl, Conformance::I2cSda, 4>;
template struct Kvasir::Usart::
  Port<Conformance::ClockTree, 1, 115'200, Conformance::UartTx, Conformance::UartRx, 7>;
template struct Kvasir::Adc::Single<Conformance::ClockTree, 1, Conformance::AdcIn>;
template struct Kvasir::Timer::Pwm<Conformance::ClockTree, 3, 2, 25'000, Conformance::PwmOut, 2>;
template struct Kvasir::Timer::Counter<Conformance::ClockTree, 2, 1'000'000>;
template struct Kvasir::Timer::Tick<Conformance::ClockTree, 13, 50'000, &Conformance::onTick, 0>;
template struct Kvasir::Exti::
  Line<Conformance::PwmOut, Kvasir::Exti::Edge::rising, &Conformance::onEdge>;
template struct Kvasir::Watchdog<Conformance::UngatedWatchdogConfig>;
template struct Kvasir::Watchdog<Conformance::WatchdogConfig>;
template struct Kvasir::Rcc::Recovery<Conformance::ClockTree, Conformance::SystickClock>;
template struct Kvasir::Can::Bus<Conformance::ClockTree,
                                 1,
                                 500'000,
                                 Conformance::CanRx,
                                 Conformance::CanTx,
                                 9,
                                 Conformance::SystickClock>;
// the timing tests/can_timing_test.cpp reproduces on the host
static_assert(Kvasir::Can::Bus<Conformance::ClockTree,
                               1,
                               500'000,
                               Conformance::CanRx,
                               Conformance::CanTx,
                               9,
                               Conformance::SystickClock>::pclkHz
              == 50'000'000);
static_assert(Kvasir::Can::Bus<Conformance::ClockTree,
                               1,
                               500'000,
                               Conformance::CanRx,
                               Conformance::CanTx,
                               9,
                               Conformance::SystickClock>::timing.brp
              == 5);
// the flash CellStore region (Flash.hpp, sectors 9-10, RM0430 Table 5), its dependent writes for the target
template struct Kvasir::Nvm::Region<0x080A'0000, 2>;
