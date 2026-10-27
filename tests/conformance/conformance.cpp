// The chip contract of chip_stm32f413 as it is today (Kvasir_SDK tests/conformance/ChipContract.hpp): every row the chip
// has is REQUIRE, every row it lacks is GAP - a GAP that the chip fills fails here and moves to REQUIRE.

// the config header first: the chip below needs its log backend and log clock (uc_log.hpp would read it too, but the
// chip headers before it need the same)
#include "kvasir_config.hpp"

// then the chip and the drivers
#include "chip/Flash.hpp"
#include "chip/SerialNumber.hpp"
#include "chip/WaitBounds.hpp"
#include "chip/Watchdog.hpp"
#include "chip/chip.hpp"
// the core's debug query and feature macros
#include "core/Features.hpp"
#include "cortex_common/Debug.hpp"

// last: the contract rows see only what is declared before them
#include "ChipContract.hpp"

#include <chrono>

namespace Conformance {
// the watchdog, gated (the health supervisor's) and ungated (a Startup entry)
struct WatchdogConfig {
    static constexpr auto overrunTime = std::chrono::seconds{1};
    static constexpr bool gatedFeed   = true;
};

struct UngatedWatchdogConfig {
    static constexpr auto overrunTime = std::chrono::seconds{3};
};
}   // namespace Conformance

// Startup hooks the SDK's Startup calls
KVASIR_CONFORMANCE_REQUIRE(
  "Startup::FirstInitStep<Tag::User>",
  requires { Kvasir::Startup::FirstInitStep<Kvasir::Tag::User>{}(); });

// the interrupt table and the pins
KVASIR_CONFORMANCE_REQUIRE("Nvic::InterruptOffsetTraits<Tag::User>::end",
                           Kvasir::Nvic::InterruptOffsetTraits<Kvasir::Tag::User>::end > 0);
KVASIR_CONFORMANCE_REQUIRE("Io::PinLocationTraits<void>",
                           Kvasir::Io::PinLocationTraits<void>::pinEnd > 0);

// reset cause, wait bounds
KVASIR_CONFORMANCE_REQUIRE("PM::resetCause()",
                           Kvasir::Conformance::Row::resetCause);
KVASIR_CONFORMANCE_REQUIRE("PM::resetKind()",
                           Kvasir::Conformance::Row::resetKind);
KVASIR_CONFORMANCE_REQUIRE("Chip::part / partName, the identity constants",
                           Kvasir::Conformance::Row::chipPart);
KVASIR_CONFORMANCE_REQUIRE("Register::CpuHzCeiling<Tag::User>",
                           Kvasir::Conformance::Row::cpuHzCeiling);
// the health supervisor's watchdog (chip/Watchdog.hpp: the IWDG, no disarm)
KVASIR_CONFORMANCE_REQUIRE("Kvasir::Watchdog",
                           Kvasir::Conformance::Row::kvasirWatchdog);
// the serial number, as bytes and as hex text (every chip's two functions)
KVASIR_CONFORMANCE_REQUIRE("Kvasir::serialNumber() / serialNumberString()",
                           Kvasir::Conformance::Row::serialNumber);
KVASIR_CONFORMANCE_REQUIRE(
  "health watchdog (gated feed and arm, Timeout)",
  Kvasir::Conformance::HealthWatchdog<Kvasir::Watchdog<Conformance::WatchdogConfig>>);
KVASIR_CONFORMANCE_REQUIRE(
  "health watchdog, ungated (feed, Timeout)",
  Kvasir::Health::WatchdogShape<Kvasir::Watchdog<Conformance::UngatedWatchdogConfig>>);
// the CellStore's flash: sectors 9-10 (RM0430 Table 5), Flash.hpp's own example
KVASIR_CONFORMANCE_REQUIRE("Hal::NorRegion (Nvm::Region)",
                           Kvasir::Hal::NorRegion<Kvasir::Nvm::Region<0x080A'0000,
                                                                      2>>);
// not asserted, no type to name yet: an ImageCheck backend
// the core of this chip (core_cortex_m4)
KVASIR_CORE_CONFORMANCE_REQUIRE("Core::Debug::isDebuggerConnected()",
                                Kvasir::Conformance::Row::coreDebugIsDebuggerConnected);
// a reboot every chip spells the same, "is a debugger there" (always true here: DebuggerPresent.hpp)
KVASIR_CONFORMANCE_REQUIRE("Chip::reboot()",
                           Kvasir::Conformance::Row::chipReboot);
KVASIR_CONFORMANCE_REQUIRE(
  "Port::Reboot<Tag::User>, the reboot hook",
  requires { Kvasir::Port::Reboot<Kvasir::Tag::User>{}(); });
KVASIR_CONFORMANCE_REQUIRE("Chip::debuggerPresent()",
                           Kvasir::Conformance::Row::chipDebuggerPresent);
// "is halting debug on": off / on / unknown on every core (cortex_common/Debug.hpp)
KVASIR_CORE_CONFORMANCE_REQUIRE("Core::haltingDebug()",
                                Kvasir::Conformance::Row::coreHaltingDebug);

// chip/Blocks.hpp names each block's base as a number from RM0430 Table 1: each row against the base of the generated
// register set it stands for (in table order)
#include "peripherals/ADC1.hpp"
#include "peripherals/CAN.hpp"
#include "peripherals/EXTI.hpp"
#include "peripherals/GPIO.hpp"
#include "peripherals/GPIOA.hpp"
#include "peripherals/GPIOB.hpp"
#include "peripherals/I2C.hpp"
#include "peripherals/PWR.hpp"
#include "peripherals/SPI.hpp"
#include "peripherals/SYSCFG.hpp"
#include "peripherals/TIM10.hpp"
#include "peripherals/TIM11.hpp"
#include "peripherals/TIM12.hpp"
#include "peripherals/TIM13_14.hpp"
#include "peripherals/TIM2.hpp"
#include "peripherals/TIM3_4.hpp"
#include "peripherals/TIM5.hpp"
#include "peripherals/TIM6_7.hpp"
#include "peripherals/TIM9.hpp"
#include "peripherals/USART.hpp"

namespace Conformance::BlockBases {
namespace P        = Kvasir::Peripheral;
using RegisterSets = brigand::list<P::SPI::Registers<1>,
                                   P::SPI::Registers<2>,
                                   P::SPI::Registers<3>,
                                   P::SPI::Registers<4>,
                                   P::SPI::Registers<5>,
                                   P::I2C::Registers<1>,
                                   P::I2C::Registers<2>,
                                   P::I2C::Registers<3>,
                                   P::USART::Registers<1>,
                                   P::USART::Registers<2>,
                                   P::USART::Registers<3>,
                                   P::USART::Registers<6>,
                                   P::CAN::Registers<1>,
                                   P::TIM2::Registers<>,
                                   P::TIM3_4::Registers<3>,
                                   P::TIM3_4::Registers<4>,
                                   P::TIM5::Registers<>,
                                   P::TIM6_7::Registers<6>,
                                   P::TIM6_7::Registers<7>,
                                   P::TIM9::Registers<>,
                                   P::TIM10::Registers<>,
                                   P::TIM11::Registers<>,
                                   P::TIM12::Registers<>,
                                   P::TIM13_14::Registers<13>,
                                   P::TIM13_14::Registers<14>,
                                   P::ADC1::Registers<>,
                                   P::SYSCFG::Registers<>,
                                   P::EXTI::Registers<>,
                                   P::GPIOA::Registers<>,
                                   P::GPIOB::Registers<>,
                                   P::GPIO::Registers<2>,
                                   P::GPIO::Registers<3>,
                                   P::GPIO::Registers<7>,
                                   P::PWR::Registers<>>;

template<typename Row,
         typename Regs>
consteval bool sameBase() {
    static_assert(Row::base == Regs::baseAddr,
                  "chip/Blocks.hpp: a row's base is not its register set's baseAddr");
    return true;
}

template<typename... Rows,
         typename... Regs>
consteval bool everyRow(Kvasir::Hal::BlockTable<Rows...>*,
                        brigand::list<Regs...>*) {
    static_assert(sizeof...(Rows) == sizeof...(Regs),
                  "one register set per row of chip/Blocks.hpp");
    return (sameBase<Rows, Regs>() && ...);
}

static_assert(everyRow(static_cast<Kvasir::Chip::Blocks::Table*>(nullptr),
                       static_cast<RegisterSets*>(nullptr)));
}   // namespace Conformance::BlockBases
