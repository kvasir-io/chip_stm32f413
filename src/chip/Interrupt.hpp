#pragma once
#include "kvasir/Common/Interrupt.hpp"

#include <array>
#include <type_traits>

namespace Kvasir {
namespace Interrupt {
    template<int I>
    using Type = ::Kvasir::Nvic::Index<I>;

    static constexpr Type<-14> nonMaskableInt{};
    static constexpr Type<-13> hardFault{};
    static constexpr Type<-12> memoryManagement{};
    static constexpr Type<-11> busFault{};
    static constexpr Type<-10> usageFault{};
    static constexpr Type<-5>  sVCall{};
    static constexpr Type<-2>  pendSV{};
    static constexpr Type<-1>  systick{};

    // The peripheral lines, from RM0430 Rev 8 Table 40 "Vector table for STM32F413/423" (10.2, md
    // line 11181) - not from chip.svd, whose <interrupt> list misses WWDG, OTG_FS and QUADSPI. Shared
    // vectors are named after every peripheral on them (tim1_up_tim10: TIM1 update and TIM10).
    static constexpr Type<0>   wwdg{};
    static constexpr Type<1>   pvd{};
    static constexpr Type<2>   tamp_stamp{};
    static constexpr Type<3>   rtc_wkup{};
    static constexpr Type<4>   flash{};
    static constexpr Type<5>   rcc{};
    static constexpr Type<6>   exti0{};
    static constexpr Type<7>   exti1{};
    static constexpr Type<8>   exti2{};
    static constexpr Type<9>   exti3{};
    static constexpr Type<10>  exti4{};
    static constexpr Type<11>  dma1_stream0{};
    static constexpr Type<12>  dma1_stream1{};
    static constexpr Type<13>  dma1_stream2{};
    static constexpr Type<14>  dma1_stream3{};
    static constexpr Type<15>  dma1_stream4{};
    static constexpr Type<16>  dma1_stream5{};
    static constexpr Type<17>  dma1_stream6{};
    static constexpr Type<18>  adc{};
    static constexpr Type<19>  can1_tx{};
    static constexpr Type<20>  can1_rx0{};
    static constexpr Type<21>  can1_rx1{};
    static constexpr Type<22>  can1_sce{};
    static constexpr Type<23>  exti9_5{};
    static constexpr Type<24>  tim1_brk_tim9{};
    static constexpr Type<25>  tim1_up_tim10{};
    static constexpr Type<26>  tim1_trg_com_tim11{};
    static constexpr Type<27>  tim1_cc{};
    static constexpr Type<28>  tim2{};
    static constexpr Type<29>  tim3{};
    static constexpr Type<30>  tim4{};
    static constexpr Type<31>  i2c1_ev{};
    static constexpr Type<32>  i2c1_er{};
    static constexpr Type<33>  i2c2_ev{};
    static constexpr Type<34>  i2c2_er{};
    static constexpr Type<35>  spi1{};
    static constexpr Type<36>  spi2{};
    static constexpr Type<37>  usart1{};
    static constexpr Type<38>  usart2{};
    static constexpr Type<39>  usart3{};
    static constexpr Type<40>  exti15_10{};
    static constexpr Type<41>  exti17_rtc_alarm{};
    static constexpr Type<42>  exti18_otg_fs_wkup{};
    static constexpr Type<43>  tim8_brk_tim12{};
    static constexpr Type<44>  tim8_up_tim13{};
    static constexpr Type<45>  tim8_trg_com_tim14{};
    static constexpr Type<46>  tim8_cc{};
    static constexpr Type<47>  dma1_stream7{};
    static constexpr Type<48>  fsmc{};
    static constexpr Type<49>  sdio{};
    static constexpr Type<50>  tim5{};
    static constexpr Type<51>  spi3{};
    static constexpr Type<52>  uart4{};
    static constexpr Type<53>  uart5{};
    static constexpr Type<54>  tim6_dac{};
    static constexpr Type<55>  tim7{};
    static constexpr Type<56>  dma2_stream0{};
    static constexpr Type<57>  dma2_stream1{};
    static constexpr Type<58>  dma2_stream2{};
    static constexpr Type<59>  dma2_stream3{};
    static constexpr Type<60>  dma2_stream4{};
    static constexpr Type<61>  dfsdm1_flt0{};
    static constexpr Type<62>  dfsdm1_flt1{};
    static constexpr Type<63>  can2_tx{};
    static constexpr Type<64>  can2_rx0{};
    static constexpr Type<65>  can2_rx1{};
    static constexpr Type<66>  can2_sce{};
    static constexpr Type<67>  otg_fs{};
    static constexpr Type<68>  dma2_stream5{};
    static constexpr Type<69>  dma2_stream6{};
    static constexpr Type<70>  dma2_stream7{};
    static constexpr Type<71>  usart6{};
    static constexpr Type<72>  i2c3_ev{};
    static constexpr Type<73>  i2c3_er{};
    static constexpr Type<74>  can3_tx{};
    static constexpr Type<75>  can3_rx0{};
    static constexpr Type<76>  can3_rx1{};
    static constexpr Type<77>  can3_sce{};
    static constexpr Type<80>  rng{};
    static constexpr Type<81>  fpu{};
    static constexpr Type<82>  uart7{};
    static constexpr Type<83>  uart8{};
    static constexpr Type<84>  spi4{};
    static constexpr Type<85>  spi5{};
    static constexpr Type<87>  sai1{};
    static constexpr Type<88>  uart9{};
    static constexpr Type<89>  uart10{};
    static constexpr Type<92>  quadspi{};
    static constexpr Type<95>  fmpi2c1_ev{};
    static constexpr Type<96>  fmpi2c1_er{};
    static constexpr Type<97>  exti23_lptim1{};
    static constexpr Type<98>  dfsdm2_flt0{};
    static constexpr Type<99>  dfsdm2_flt1{};
    static constexpr Type<100> dfsdm2_flt2{};
    static constexpr Type<101> dfsdm2_flt3{};

    // The vector that serves EXTI line n (Table 40): lines 0 to 4 have one each, 5 to 9 share
    // exti9_5, 10 to 15 share exti15_10.
    template<unsigned Line>
    using ExtiVector
      = Type<(Line <= 4   ? std::remove_cvref_t<decltype(exti0)>::value + static_cast<int>(Line)
              : Line <= 9 ? std::remove_cvref_t<decltype(exti9_5)>::value
                          : std::remove_cvref_t<decltype(exti15_10)>::value)>;
}   // namespace Interrupt

namespace Nvic {
    using namespace Kvasir::Interrupt;

    template<>
    struct InterruptOffsetTraits<void> {
        static constexpr int begin = -14;
        /// Half open: the last vector is DFSDM2_FLT3 at 101.
        static constexpr int end = 102;
        /// The Cortex-M4 vectors that are reserved (-9 to -6, -3) or unused here (-4, the debug
        /// monitor), the positions Table 40 leaves out (78, 86, 90, 91, 93, 94), and 79 (AES):
        /// "Only available for STM32F423xx" (Table 40 note 1).
        static constexpr std::array disabled = {-9, -8, -7, -6, -4, -3, 78, 79, 86, 90, 91, 93, 94};
        static constexpr std::array noEnable
          = {nonMaskableInt.index(), sVCall.index(), pendSV.index()};
        static constexpr std::array noDisable
          = {nonMaskableInt.index(), sVCall.index(), pendSV.index()};
        static constexpr std::array noSetPending = {sVCall.index(), hardFault.index()};
        static constexpr std::array noClearPending
          = {nonMaskableInt.index(), sVCall.index(), hardFault.index()};
        static constexpr std::array noSetPriority = {nonMaskableInt.index(), hardFault.index()};

        /// The three configurable faults are enabled in SHCSR by Fault::Core::EarlyInitList
        /// rather than through the NVIC, so none of them needs an enable here.
        using FaultInterruptIndexs           = brigand::list<decltype(hardFault),
                                                             decltype(memoryManagement),
                                                             decltype(busFault),
                                                             decltype(usageFault)>;
        using FaultInterruptIndexsNeedEnable = brigand::list<>;
    };

}   // namespace Nvic
}   // namespace Kvasir
