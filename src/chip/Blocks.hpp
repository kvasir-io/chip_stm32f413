#pragma once

// The STM32F413's gated blocks (kvasir/Hal/Block.hpp): every block a driver here writes, with the RCC enable bit
// that clocks it (RM0430 Rev 8 6.3.11 RCC_AHB1ENR, 6.3.15 RCC_APB1ENR, 6.3.16 RCC_APB2ENR). A Startup-phase write
// into a block whose bit no listed powerClockEnable sets does not compile. A row names its block's base as a number,
// each block 1 KiB (RM0430 Rev 8 Table 1 "Register boundary addresses": AHB1 md l.2676-2710, APB2 l.2723, APB1
// l.2731-2767); `Rcc::OnBusOf<Regs>` finds a row by the generated `Regs::baseAddr`, so a wrong number is "no
// block" at the first driver, and tests/conformance asserts every row against its register set. The table includes
// only RCC: a driver includes its own peripheral's header. The drivers' `Instance<N>` rows take
// their enable action from here (Rcc::OnBusOf). The GPIO ports are clocked by FirstInitStep (StartUp.hpp) and PWR
// by the clock init (Rcc.hpp): open at boot, they claim nothing.
//
// The gate id is the enable bit's position counted from RCC's base: (register offset) * 8 + bit. Every row's enable bit
// sits in the enable register of its block's bus (Instances.hpp, by the base: RCC_APB1ENR 6.3.15, RCC_APB2ENR 6.3.16,
// RCC_AHB1ENR 6.3.11); a row that names another bus's bit does not compile.

#include "Instances.hpp"
#include "kvasir/Hal/Block.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/RCC.hpp"

#include <bit>
#include <cstdint>
#include <type_traits>

namespace Kvasir { namespace Chip { namespace Blocks {
    using R = Peripheral::RCC::Registers<>;

    // the RCC enable register of the bus the block at `base` is on (RM0430 Table 1, Instances.hpp), 0 for none
    constexpr unsigned enableRegisterOf(std::uint32_t base) {
        if(onApb1(base)) { return Register::RegisterTraits<R::APB1ENR::Addr>::address; }
        if(onApb2(base)) { return Register::RegisterTraits<R::APB2ENR::Addr>::address; }
        if(onAhb1(base)) { return Register::RegisterTraits<R::AHB1ENR::Addr>::address; }
        return 0;
    }

    template<std::uint32_t Base,
             auto const&   Enable>
    constexpr unsigned gateId() {
        using F = std::remove_cvref_t<decltype(Enable)>;
        static_assert(std::has_single_bit(F::mask), "an enable bit");
        static_assert(Register::RegisterTraits<F>::address == enableRegisterOf(Base),
                      "this block's enable bit is not in the RCC enable register of the bus its "
                      "base address is on "
                      "(RM0430 Table 1; 6.3.11 AHB1ENR, 6.3.15 APB1ENR, 6.3.16 APB2ENR)");
        return (Register::RegisterTraits<F>::address - R::baseAddr) * 8U
             + static_cast<unsigned>(std::countr_zero(F::mask));
    }

    template<std::uint32_t Base, auto const& Enable, bool OpenAtBoot = false>
    using Row = ::Kvasir::Hal::
      BlockRow<Base, 0x400, gateId<Base, Enable>(), decltype(set(Enable)), OpenAtBoot>;

    // clang-format off
    using Table = ::Kvasir::Hal::BlockTable<
        Row<0x40013000, R::APB2ENR::spi1en>,
        Row<0x40003800, R::APB1ENR::spi2en>,
        Row<0x40003C00, R::APB1ENR::spi3en>,
        Row<0x40013400, R::APB2ENR::spi4en>,
        Row<0x40015000, R::APB2ENR::spi5en>,
        Row<0x40005400, R::APB1ENR::i2c1en>,
        Row<0x40005800, R::APB1ENR::i2c2en>,
        Row<0x40005C00, R::APB1ENR::i2c3en>,
        Row<0x40011000, R::APB2ENR::usart1en>,
        Row<0x40004400, R::APB1ENR::usart2en>,
        Row<0x40004800, R::APB1ENR::usart3en>,
        Row<0x40011400, R::APB2ENR::usart6en>,
        Row<0x40006400, R::APB1ENR::can1en>,
        Row<0x40000000, R::APB1ENR::tim2en>,
        Row<0x40000400, R::APB1ENR::tim3en>,
        Row<0x40000800, R::APB1ENR::tim4en>,
        Row<0x40000C00, R::APB1ENR::tim5en>,
        Row<0x40001000, R::APB1ENR::tim6en>,
        Row<0x40001400, R::APB1ENR::tim7en>,
        Row<0x40014000, R::APB2ENR::tim9en>,
        Row<0x40014400, R::APB2ENR::tim10en>,
        Row<0x40014800, R::APB2ENR::tim11en>,
        Row<0x40001800, R::APB1ENR::tim12en>,
        Row<0x40001C00, R::APB1ENR::tim13en>,
        Row<0x40002000, R::APB1ENR::tim14en>,
        Row<0x40012000, R::APB2ENR::adc1en>,              // ADC_Common (0x40012300) is in this block
        Row<0x40013800, R::APB2ENR::syscfgen>,
        Row<0x40013C00, R::APB2ENR::extiten>,
        Row<0x40020000, R::AHB1ENR::gpioaen, true>,      // FirstInitStep, StartUp.hpp
        Row<0x40020400, R::AHB1ENR::gpioben, true>,
        Row<0x40020800, R::AHB1ENR::gpiocen, true>,
        Row<0x40020C00, R::AHB1ENR::gpioden, true>,
        Row<0x40021C00, R::AHB1ENR::gpiohen, true>,
        Row<0x40007000, R::APB1ENR::pwren, true>>;          // the clock init, Rcc.hpp
    // clang-format on
}}}   // namespace Kvasir::Chip::Blocks

template<>
struct Kvasir::Hal::BlockGates<Kvasir::Tag::User> {
    using type = Kvasir::Chip::Blocks::Table;
};
