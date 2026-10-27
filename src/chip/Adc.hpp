#pragma once

#include "ClockMath.hpp"
#include "Clocks.hpp"
#include "Instances.hpp"
#include "Io.hpp"
#include "Rcc.hpp"
#include "WaitBounds.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/ADC1.hpp"
#include "peripherals/ADC_Common.hpp"

#include <array>
#include <cstdint>
#include <optional>

// ADC1 of the STM32F413 (RM0430 Rev 8 chapter 13), one regular channel, a single software-started
// conversion at a time, polled (13.3.4 "Single conversion mode"): SQ1 = the channel, SWSTART, EOC,
// DR. 12 bit, right aligned.
//
//     using Vin = Kvasir::Adc::Single<HW::ClockTree, 1, HW::Board::unregulatedSense>;
//     if(auto counts = Vin::read()) { ... }   // 0..4095 of VREF+
//
// The ADC clock is PCLK2 / ADCPRE (2, 4, 6, 8; 13.12.16), at most 36 MHz at VDDA 2.4-3.6 V
// (DS11581 Rev 7 Table 75 "ADC characteristics"). Errata ES0372 Rev 8 2.2.8 ("Internal noise
// impacting the ADC accuracy"): averaging is the caller's.
namespace Kvasir { namespace Adc {
    namespace Detail {
        using ADC    = Kvasir::Peripheral::ADC1::Registers<>;
        using COMMON = Kvasir::Peripheral::ADC_COMMON::Registers<>;
    }   // namespace Detail

    /// Sample time per channel, SMPR1/SMPR2 SMPx (13.12.4, 13.12.5): 3, 15, 28, 56, 84, 112, 144
    /// or 480 ADC clock cycles.
    enum class SampleTime : std::uint32_t {
        c3   = 0b000,
        c15  = 0b001,
        c28  = 0b010,
        c56  = 0b011,
        c84  = 0b100,
        c112 = 0b101,
        c144 = 0b110,
        c480 = 0b111
    };

    template<typename Clocks, unsigned Channel, typename Pin, SampleTime Sample = SampleTime::c480>
    struct Single {
        static_assert(Channel <= 9,
                      "channels 10-18 (SMPR1) are not wired up yet");
        using ADC = Detail::ADC;
        // its bus (Instances.hpp) and enable bit (Blocks.hpp)
        using I = Rcc::OnBusOf<ADC>;

        static constexpr std::uint32_t pclkHz = Clocks::template pclkHz<I::bus>;
        static constexpr unsigned      adcpre = Detail::prescalerBits(pclkHz);
        static constexpr std::uint32_t adcHz  = pclkHz / (2 * (adcpre + 1));
        // Startup: ADC1, and the bus clock the ADC prescaler divides (ADCCLK from PCLK2, 13.3.4;
        // Clocks.hpp). One owner: a second Single would rewrite the one regular sequence (SQR3.SQ1,
        // 13.12.11), and the first one's read() would convert the second one's channel.
        using Provides = brigand::list<Hal::Owns<ADC>>;
        using Claims   = Hal::Claim<Kvasir::Clocks::ClkApb<I::bus>, pclkHz>;
        static_assert(adcHz <= 36'000'000,
                      "ADC clock above 36 MHz (DS11581 Table 75)");
        // the SMPx code's ADC clock cycles (13.12.5)
        static constexpr std::uint64_t sampleCycles
          = std::array<std::uint64_t,
                       8>{3, 15, 28, 56, 84, 112, 144, 480}[static_cast<std::uint32_t>(Sample)];

        // SMPR2 (13.12.5): channel n's sample time at bits 3n+2:3n (the SVD has it as one field).
        static constexpr Register::FieldLocation<typename ADC::SMPR2::Addr,
                                                 (0b111U << (3 * Channel)),
                                                 Register::ReadWriteAccess,
                                                 std::uint32_t>
          smp{};

        static constexpr auto powerClockEnable  = I::powerClockEnable;
        static constexpr auto initStepPinConfig = list(action(Io::Action::Analog{}, Pin{}));
        static constexpr auto initStepPeripheryConfig
          = list(write(Detail::COMMON::CCR::adcpre, Register::value<std::uint32_t, adcpre>()),
                 write(smp, Register::value<std::uint32_t, static_cast<std::uint32_t>(Sample)>()),
                 write(ADC::SQR1::l, Register::value<std::uint32_t, 0>()),   // one conversion
                 write(ADC::SQR3::sq1, Register::value<std::uint32_t, Channel>()));
        // ADON; the ADC needs tSTAB (3 us max, Table 75) before the first conversion - the first
        // read() comes after the rest of the start-up.
        static constexpr auto initStepPeripheryEnable = list(set(ADC::CR2::adon));

        /// One conversion, 0..4095; nullopt if EOC never came.
        static std::optional<std::uint16_t> read() {
            apply(set(ADC::CR2::swstart));
            // bounded (WaitBounds.hpp), Report whatever policy the application injects
            if(!Register::waitUntil<Kvasir::Chip::AdcConversionBound<adcHz, sampleCycles>,
                                    Register::OnTimeout::Report>(Register::isSet(ADC::SR::eoc)))
            {
                return std::nullopt;
            }
            // Reading DR clears EOC (13.12.1).
            return static_cast<std::uint16_t>(apply(Register::read(ADC::DR::data)) & 0xFFFU);
        }
    };
}}   // namespace Kvasir::Adc
