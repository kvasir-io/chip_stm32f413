// src/chip/Instances.hpp: the bus of every block a driver here has, by its base address, against RM0430 Rev 8 Table 1,
// and the SPI limits against DS11581 Rev 7 Table 67. Building is the test.
#include "Instances.hpp"

#include <cstdio>

namespace {
using Kvasir::Rcc::Bus;
using Kvasir::Rcc::Supply;
namespace Chip = Kvasir::Chip;

// Table 1 (md l.2676-2767): every block a driver here has, by its base address - SPI1/4/5 on APB2, SPI2/3 on APB1
static_assert(Chip::busOfBase(0x4001'3000) == Bus::apb2 && Chip::busOfBase(0x4001'3400) == Bus::apb2
              && Chip::busOfBase(0x4001'5000) == Bus::apb2);
static_assert(Chip::busOfBase(0x4000'3800) == Bus::apb1
              && Chip::busOfBase(0x4000'3C00) == Bus::apb1);
// USART1/6 on APB2, USART2/3 on APB1
static_assert(Chip::busOfBase(0x4001'1000) == Bus::apb2
              && Chip::busOfBase(0x4001'1400) == Bus::apb2);
static_assert(Chip::busOfBase(0x4000'4400) == Bus::apb1
              && Chip::busOfBase(0x4000'4800) == Bus::apb1);
// TIM1/8/9/10/11 on APB2, TIM2-7 and 12-14 on APB1
static_assert(Chip::busOfBase(0x4001'0000) == Bus::apb2
              && Chip::busOfBase(0x4001'0400) == Bus::apb2);
static_assert(Chip::busOfBase(0x4001'4000) == Bus::apb2 && Chip::busOfBase(0x4001'4400) == Bus::apb2
              && Chip::busOfBase(0x4001'4800) == Bus::apb2);
static_assert(Chip::busOfBase(0x4000'0000) == Bus::apb1 && Chip::busOfBase(0x4000'0C00) == Bus::apb1
              && Chip::busOfBase(0x4000'1000) == Bus::apb1);
static_assert(Chip::busOfBase(0x4000'1400) == Bus::apb1 && Chip::busOfBase(0x4000'1800) == Bus::apb1
              && Chip::busOfBase(0x4000'2000) == Bus::apb1);
// I2C1-3 and CAN1 on APB1 (0x4000 5400-5C00, 0x4000 6400), PWR too (0x4000 7000)
static_assert(Chip::busOfBase(0x4000'5400) == Bus::apb1 && Chip::busOfBase(0x4000'5C00) == Bus::apb1
              && Chip::busOfBase(0x4000'6400) == Bus::apb1
              && Chip::busOfBase(0x4000'7000) == Bus::apb1);
// ADC1, SYSCFG and EXTI on APB2 (0x4001 2000, 0x4001 3800, 0x4001 3C00)
static_assert(Chip::busOfBase(0x4001'2000) == Bus::apb2 && Chip::busOfBase(0x4001'3800) == Bus::apb2
              && Chip::busOfBase(0x4001'3C00) == Bus::apb2);
// the ends of the two APB ranges, and the GPIO ports on AHB1 (busOfBase refuses them: instances_must_fail)
static_assert(Chip::onApb1(0x4000'7FFF) && !Chip::onApb1(0x4000'8000)
              && !Chip::onApb2(0x4000'FFFF));
static_assert(Chip::onApb2(0x4001'FFFF) && !Chip::onApb2(0x4002'0000));
static_assert(Chip::onAhb1(0x4002'0000) && Chip::onAhb1(0x4002'1C00) && !Chip::onApb1(0x4002'0000)
              && !Chip::onApb2(0x4002'0000));

// Table 67: SPI2/3 25 MHz whatever the supply or direction
static_assert(Chip::spiMaxSckHz(2,
                                Supply::from2V7,
                                true)
              == 25'000'000);
static_assert(Chip::spiMaxSckHz(3,
                                Supply::from2V7,
                                false)
              == 25'000'000);
// SPI1/4/5 receiving: 42 MHz from 2.7 V (no 3.0 V range in Supply), 25 MHz from 1.7 V
static_assert(Chip::spiMaxSckHz(1,
                                Supply::from2V7,
                                true)
              == 42'000'000);
static_assert(Chip::spiMaxSckHz(5,
                                Supply::from2V4,
                                true)
              == 25'000'000);
static_assert(Chip::spiMaxSckHz(4,
                                Supply::from1V7,
                                true)
              == 25'000'000);
// master transmitter: 50 MHz from 1.71 V
static_assert(Chip::spiMaxSckHz(1,
                                Supply::from1V7,
                                false)
              == 50'000'000);
static_assert(Chip::spiMaxSckHz(5,
                                Supply::from2V7,
                                false)
              == 50'000'000);
}   // namespace

int main() {
    std::puts("instances: ok");
    return 0;
}
