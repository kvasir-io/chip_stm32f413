#pragma once

#include "ClockMath.hpp"
#include "Io.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/ADC1.hpp"
#include "peripherals/ADC_Common.hpp"
#include "peripherals/RCC.hpp"

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
        using RCC    = Kvasir::Peripheral::RCC::Registers<>;
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

        static constexpr unsigned      adcpre = Detail::prescalerBits(Clocks::pclk2Hz);
        static constexpr std::uint32_t adcHz  = Clocks::pclk2Hz / (2 * (adcpre + 1));
        static_assert(adcHz <= 36'000'000,
                      "ADC clock above 36 MHz (DS11581 Table 75)");

        // SMPR2 (13.12.5): channel n's sample time at bits 3n+2:3n (the SVD has it as one field).
        static constexpr Register::FieldLocation<typename ADC::SMPR2::Addr,
                                                 (0b111U << (3 * Channel)),
                                                 Register::ReadWriteAccess,
                                                 std::uint32_t>
          smp{};

        static constexpr auto powerClockEnable
          = list(write(Detail::RCC::APB2ENR::adc1en, Register::value<std::uint32_t, 1>()));
        static constexpr auto initStepPinConfig = list(action(Io::Action::Analog{}, Pin{}));
        static constexpr auto initStepPeripheryConfig
          = list(write(Detail::COMMON::CCR::adcpre, Register::value<std::uint32_t, adcpre>()),
                 write(smp, Register::value<std::uint32_t, static_cast<std::uint32_t>(Sample)>()),
                 write(ADC::SQR1::l, Register::value<std::uint32_t, 0>()),   // one conversion
                 write(ADC::SQR3::sq1, Register::value<std::uint32_t, Channel>()));
        // ADON; the ADC needs tSTAB (3 us max, Table 75) before the first conversion - the first
        // read() comes after the rest of the start-up.
        static constexpr auto initStepPeripheryEnable
          = list(write(ADC::CR2::adon, Register::value<std::uint32_t, 1>()));

        /// One conversion, 0..4095; nullopt if EOC never came.
        static std::optional<std::uint16_t> read() {
            apply(write(ADC::CR2::swstart, Register::value<std::uint32_t, 1>()));
            for(unsigned i = 0; i < 100'000; ++i) {
                if(apply(Register::read(ADC::SR::eoc)) != 0) {
                    // Reading DR clears EOC (13.12.1).
                    return static_cast<std::uint16_t>(apply(Register::read(ADC::DR::data))
                                                      & 0xFFFU);
                }
            }
            return std::nullopt;
        }
    };
}}   // namespace Kvasir::Adc
