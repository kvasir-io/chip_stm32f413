#pragma once
// The bounds of the STM32F413 package's register waits (Kvasir_SDK kvasir/Register/Wait.hpp), each with its source.
// Under the default wait policy (Unbounded) every site is the loop it was.
#include "kvasir/Register/Wait.hpp"

#include <cstdint>

// HCLK max 100 MHz (RM0430 Table 6, ClockMath.hpp): the SDK's MicrosecondsBound converts at this. Specialised before
// any bound below names it.
template<>
struct Kvasir::Register::CpuHzCeiling<Kvasir::Tag::User> {
    static constexpr std::uint64_t value = 100'000'000;
};

namespace Kvasir::Chip {
// A read back of a register just written (FLASH.ACR.LATENCY): a bus round trip.
using ReadBackBound = Kvasir::Register::Bound::Polls<100>;
// FLASH.SR.BSY: a 128 KB sector erase at x32 parallelism takes 2 s max (DS11581 Table 51, md l.4493-4494); the
// driver programs x32 (Flash.hpp)
using FlashBusyBound = Register::MicrosecondsBound<4'000'000>;

// RCC (Rcc.hpp ClockTree::init, in coreClockInit: Report only, the result read back by initResult()).
// "Unchecked" = the datasheet gives no time; the value is a choice.
// CR.HSERDY: tSU(HSE) typ 2 ms, no minimum or maximum, "can vary significantly with the crystal manufacturer"
// (DS11581 Rev 7 Table 43, md l.4141/4149); 20 ms = 10x the typical (unchecked as a maximum). The bypass mode
// (an external clock) gives no time (Table 41).
using HseReadyBound = Register::MicrosecondsBound<20'000>;
// CR.PLLRDY: tLOCK max 300 us at VCO 432 MHz, 200 us at 100 MHz (DS11581 Table 47 "Main PLL characteristics", md
// l.4312-4313); 1 ms.
using PllLockBound = Register::MicrosecondsBound<1'000>;
// PWR_CSR.VOSRDY: no time given (RM0430 Rev 8 5.4.2, md l.5046; the scale is "only taken into account once the PLL
// is activated", md l.4287) - unchecked: 1 ms.
using VosReadyBound = Register::MicrosecondsBound<1'000>;
// CFGR.SWS after SW = PLL: no time given; the switch "occurs only if the target clock source is ready" (RM0430 6.2,
// md l.5446), and the PLL is locked by then - unchecked: 1 ms.
using SwsBound = Register::MicrosecondsBound<1'000>;

// bxCAN MSR.INAK after MCR.INRQ: "the CAN hardware waits until the current CAN activity (transmission or reception)
// is completed before entering the initialization mode" (RM0430 32.9.2 MCR.INRQ, md l.47521), no time given: one
// frame at the bus's bit rate. 160 bits for the longest frame with its stuff bits is CAN's (ISO 11898-1), not the
// manual's (unchecked); x2.
template<std::uint32_t Bitrate>
using CanInitBound = Register::MicrosecondsBound<2ULL * 160 * 1'000'000 / Bitrate + 1>;

// ADC SR.EOC after SWSTART: tCONV = the sample time + 12 ADC clocks at 12 bits, "9 to 492 (tS for sampling + n-bit
// resolution for successive approximation) 1/fADC" (DS11581 Table 75, md l.5555; RM0430 13.5, md l.15277: "Tconv =
// Sampling time + 12 cycles").
template<std::uint64_t AdcHz, std::uint64_t SampleCycles>
using AdcConversionBound = Register::MicrosecondsBound<(SampleCycles + 12) * 1'000'000 / AdcHz + 1>;

// I2C SR1 SB/ADDR/TXE/BTF/RXNE and SR2.BUSY after a STOP: no time given for any flag (RM0430 27.6.6, md l.37780-37824;
// BUSY "cleared by hardware on detection of a Stop condition", 27.6.7 md l.37909). A byte and its acknowledge are
// nine SCL periods; a target may stretch SCL for as long as it likes - 10 bytes' time (unchecked).
template<std::uint32_t SclHz>
using I2cByteBound = Register::MicrosecondsBound<10ULL * 9 * 1'000'000 / SclHz + 1>;

// SPI SR.TXE / RXNE / BSY: "Wait until TXE=1 and then wait until BSY=0" (RM0430 29.3.10, md l.40830-40833); BSY
// clears "when it finishes a data transmission and no new data is ready to be sent" (29.3.12, md l.40952). At most
// the frame in the shift register and the one in the TX buffer: two frames at the bus's SCK.
template<std::uint32_t SckHz, unsigned FrameBits>
using SpiFrameBound = Register::MicrosecondsBound<2ULL * FrameBits * 1'000'000 / SckHz + 1>;
}   // namespace Kvasir::Chip
