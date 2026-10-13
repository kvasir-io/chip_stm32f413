#pragma once

#include "Io.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/CAN.hpp"
#include "peripherals/RCC.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

// bxCAN of the STM32F413 (RM0430 Rev 8 chapter 32), polled: send into a free transmit mailbox,
// receive from FIFO 0, every frame accepted (filter bank 0, mask 0). The bit timing is solved at
// compile time from PCLK1 (32.7.7: bit = (1 + TS1 + TS2) tq, tq = (BRP + 1) / PCLK1, TS1 1-16,
// TS2 1-8 quanta), exact, with the sample point nearest the wanted one.
//
//     using Can = Kvasir::Can::Bus<HW::ClockTree, 1, 500'000, RxPin, TxPin, 8>;
//     Can::send(frame);                          // Result::ok, or ::full (no mailbox free)
//     while(auto f = Can::receive()) { ... }
//
// Bus-off is left by the hardware (MCR.ABOM, 32.7.6): no error needs a person or a reset. Startup
// requests normal mode and does not wait for it - it is reached after 11 recessive bits (32.4.2),
// which a board without a bus never sees; until then frames wait in their mailboxes.
namespace Kvasir { namespace Can {
    struct Frame {
        std::uint32_t            id{};   // 11 bits, or 29 with extended
        bool                     extended{};
        bool                     remote{};
        std::uint8_t             dlc{};   // 0-8
        std::array<std::byte, 8> data{};
    };

    enum class Result : std::uint8_t { ok, full, badFrame };

    struct BitTiming {
        unsigned brp;   // the divider, BRP + 1
        unsigned ts1;   // quanta, TS1 + 1
        unsigned ts2;   // quanta, TS2 + 1
        unsigned samplePermille;
    };

    // Every exact divider and split, the sample point closest to the wanted one; brp 0 = none.
    constexpr BitTiming solveBitTiming(std::uint32_t pclkHz,
                                       std::uint32_t bitrate,
                                       unsigned      wantPermille) {
        BitTiming best{0, 0, 0, 0};
        unsigned  bestDiff = ~0U;
        for(unsigned brp = 1; brp <= 1024; ++brp) {
            if(pclkHz % brp != 0 || (pclkHz / brp) % bitrate != 0) { continue; }
            unsigned const quanta = pclkHz / brp / bitrate;
            for(unsigned ts2 = 1; ts2 <= 8; ++ts2) {
                if(quanta < 1 + 1 + ts2) { break; }
                unsigned const ts1 = quanta - 1 - ts2;
                if(ts1 > 16) { continue; }
                unsigned const sample = (1 + ts1) * 1000 / quanta;
                unsigned const diff
                  = sample > wantPermille ? sample - wantPermille : wantPermille - sample;
                if(diff < bestDiff) {
                    bestDiff = diff;
                    best     = BitTiming{brp, ts1, ts2, sample};
                }
            }
        }
        return best;
    }

    namespace Detail { using RCC = Kvasir::Peripheral::RCC::Registers<>; }

    template<unsigned N>
    struct Instance;

    // CAN1 on APB1 (RM0430 Table 1, 6.3.15 APB1ENR.CAN1EN). CAN2 shares CAN1's filters and needs
    // its clock (32.3.3): not supported here.
    template<>
    struct Instance<1> {
        static constexpr auto enable() {
            return write(Detail::RCC::APB1ENR::can1en, Register::value<std::uint32_t, 1>());
        }
    };

    template<typename Clocks,
             unsigned      N,
             std::uint32_t Bitrate,
             typename RxPin,
             typename TxPin,
             int      Af,
             unsigned SamplePermille = 875>
    struct Bus {
        using I    = Instance<N>;
        using Regs = Kvasir::Peripheral::CAN::Registers<N>;

        static constexpr BitTiming timing
          = solveBitTiming(Clocks::pclk1Hz, Bitrate, SamplePermille);
        static_assert(timing.brp != 0,
                      "no exact CAN bit timing for this bit rate from PCLK1 (RM0430 32.7.7)");
        static_assert(timing.samplePermille >= 750 && timing.samplePermille <= 900,
                      "CAN sample point outside 75-90 %");
        static constexpr auto powerClockEnable = list(I::enable());
        static constexpr auto initStepPinConfig
          = list(action(Io::Action::PinFunction<Af,
                                                Io::OutputType::PushPull,
                                                Io::OutputSpeed::High,
                                                Io::OutputInit::High,
                                                Io::PullConfiguration::PullUp>{},
                        TxPin{}),
                 action(Io::Action::PinFunction<Af,
                                                Io::OutputType::PushPull,
                                                Io::OutputSpeed::Low,
                                                Io::OutputInit::Low,
                                                Io::PullConfiguration::PullUp>{},
                        RxPin{}));

        static inline std::uint32_t rxOverruns{};
        static inline bool          initFailed{};   // INAK never came: the CAN clock is not running

    private:
        static constexpr unsigned pollLimit = 200'000;   // several ms at 100 MHz

        template<typename F>
        static bool waitFor(F f) {
            for(unsigned i = 0; i < pollLimit; ++i) {
                if(f()) { return true; }
            }
            return false;
        }

        static std::uint32_t lowWord(Frame const& f) {
            return std::to_integer<std::uint32_t>(f.data[0])
                 | (std::to_integer<std::uint32_t>(f.data[1]) << 8)
                 | (std::to_integer<std::uint32_t>(f.data[2]) << 16)
                 | (std::to_integer<std::uint32_t>(f.data[3]) << 24);
        }

        static std::uint32_t highWord(Frame const& f) {
            return std::to_integer<std::uint32_t>(f.data[4])
                 | (std::to_integer<std::uint32_t>(f.data[5]) << 8)
                 | (std::to_integer<std::uint32_t>(f.data[6]) << 16)
                 | (std::to_integer<std::uint32_t>(f.data[7]) << 24);
        }

        // CAN_TIxR's fields (32.9.3). An extended id is split into STID (its upper 11 bits,
        // EXID[28:18]) and EXID (its lower 18).
        struct Id {
            std::uint32_t stid;
            std::uint32_t exid;
            std::uint32_t ide;
            std::uint32_t rtr;
        };

        static Id idOf(Frame const& f) {
            return f.extended ? Id{(f.id >> 18) & 0x7FFU, f.id & 0x3'FFFFU, 1U, f.remote ? 1U : 0U}
                              : Id{f.id & 0x7FFU, 0U, 0U, f.remote ? 1U : 0U};
        }

        // Every field written: one store.
        template<typename TI,
                 typename TxRq>
        static void writeId(Id const& id,
                            TxRq      txrq) {
            apply(write(TI::stid, id.stid),
                  write(TI::exid, id.exid),
                  write(TI::ide, id.ide),
                  write(TI::rtr, id.rtr),
                  write(TI::txrq, txrq));
        }

        // TIxR, TDTxR, TDLxR, TDHxR of one mailbox (32.9.3), TXRQ last ("set up the identifier,
        // the data length code (DLC) and the data before requesting the transmission by setting
        // the corresponding TXRQ bit", 32.7.1): separate applies, so the stores keep this order.
        template<typename TI,
                 typename TDT,
                 typename TDL,
                 typename TDH>
        static void load(Frame const& f) {
            Id const id = idOf(f);
            writeId<TI>(id, Register::value<std::uint32_t, 0>());
            // TGT and TIME as 0, as at reset: the write covers the register, one store
            apply(write(TDT::time, Register::value<std::uint32_t, 0>()),
                  write(TDT::tgt, Register::value<std::uint32_t, 0>()),
                  write(TDT::dlc, std::uint32_t{f.dlc}));
            Register::writeWhole(TDL::FULLREGISTER, lowWord(f));
            Register::writeWhole(TDH::FULLREGISTER, highWord(f));
            writeId<TI>(id, Register::value<std::uint32_t, 1>());
        }

    public:
        /// Initialization mode (MCR.INRQ, MSR.INAK; 32.4.1), BTR, filter bank 0 = everything to
        /// FIFO 0 (32.7.4), then normal mode requested.
        static void runtimeInit() {
            // Leaves Sleep (reset state) for initialization: SLEEP 0, INRQ 1 (32.4.3). ABOM: bus-off
            // left by the hardware; TXFP: mailboxes go out in request order; DBF as at reset (the
            // CAN stops while the core is halted).
            apply(Regs::MCR::overrideDefaults(
              write(Regs::MCR::dbf, Register::value<std::uint32_t, 1>()),
              write(Regs::MCR::abom, Register::value<std::uint32_t, 1>()),
              write(Regs::MCR::txfp, Register::value<std::uint32_t, 1>()),
              write(Regs::MCR::sleep, Register::value<std::uint32_t, 0>()),
              write(Regs::MCR::inrq, Register::value<std::uint32_t, 1>())));
            if(!waitFor([] { return apply(read(Regs::MSR::inak)) != 0; })) {
                initFailed = true;
                return;
            }
            // BTR (32.9.2): BRP - 1, TS1 - 1, TS2 - 1, SJW 1 tq (0); loop back and silent off.
            apply(write(Regs::BTR::brp, Register::value<std::uint32_t, timing.brp - 1>()),
                  write(Regs::BTR::ts1, Register::value<std::uint32_t, timing.ts1 - 1>()),
                  write(Regs::BTR::ts2, Register::value<std::uint32_t, timing.ts2 - 1>()),
                  write(Regs::BTR::sjw, Register::value<std::uint32_t, 0>()),
                  write(Regs::BTR::lbkm, Register::value<std::uint32_t, 0>()),
                  write(Regs::BTR::silm, Register::value<std::uint32_t, 0>()));

            // FMR.FINIT (its reset value, CAN2SB 14, kept), bank 0 off, 32-bit mask mode, FIFO 0,
            // id and mask 0, on (32.9.4).
            apply(write(Regs::FMR::finit, Register::value<std::uint32_t, 1>()));
            apply(write(Regs::FA1R::fact0, Register::value<std::uint32_t, 0>()));
            apply(write(Regs::FM1R::fbm0, Register::value<std::uint32_t, 0>()),
                  write(Regs::FS1R::fsc0, Register::value<std::uint32_t, 1>()),
                  write(Regs::FFA1R::ffa0, Register::value<std::uint32_t, 0>()),
                  write(Regs::F0R1::FULLREGISTER, Register::value<std::uint32_t, 0>()),
                  write(Regs::F0R2::FULLREGISTER, Register::value<std::uint32_t, 0>()));
            apply(write(Regs::FA1R::fact0, Register::value<std::uint32_t, 1>()));
            apply(write(Regs::FMR::finit, Register::value<std::uint32_t, 0>()));

            apply(write(Regs::MCR::inrq, Register::value<std::uint32_t, 0>()));
        }

        /// Into the next free transmit mailbox (TSR.CODE, 32.9.2), or full.
        static Result send(Frame const& f) {
            if(f.dlc > 8) { return Result::badFrame; }
            // TME0-2 and CODE (the next free mailbox), one TSR load
            auto const tsr
              = apply(read(Regs::TSR::tme0, Regs::TSR::tme1, Regs::TSR::tme2, Regs::TSR::code));
            if(get<0>(tsr) == 0 && get<1>(tsr) == 0 && get<2>(tsr) == 0) { return Result::full; }
            switch(get<3>(tsr)) {
            case 0:
                load<typename Regs::TI0R,
                     typename Regs::TDT0R,
                     typename Regs::TDL0R,
                     typename Regs::TDH0R>(f);
                break;
            case 1:
                load<typename Regs::TI1R,
                     typename Regs::TDT1R,
                     typename Regs::TDL1R,
                     typename Regs::TDH1R>(f);
                break;
            default:
                load<typename Regs::TI2R,
                     typename Regs::TDT2R,
                     typename Regs::TDL2R,
                     typename Regs::TDH2R>(f);
                break;
            }
            return Result::ok;
        }

        /// The oldest frame in FIFO 0, released afterwards (RF0R.RFOM0); an overrun is counted.
        static std::optional<Frame> receive() {
            auto const rf0r = apply(read(Regs::RF0R::fovr0, Regs::RF0R::fmp0));   // one load
            if(get<0>(rf0r) != 0) {   // FOVR0, rc_w1: the whole register written, FOVR0 alone
                ++rxOverruns;
                apply(Regs::RF0R::overrideDefaults(
                  write(Regs::RF0R::fovr0, Register::value<std::uint32_t, 1>())));
            }
            if(get<1>(rf0r) == 0) { return std::nullopt; }   // FMP0
            // The mailbox, one load per register, all before the release below.
            auto const ri
              = apply(read(Regs::RI0R::stid, Regs::RI0R::exid, Regs::RI0R::ide, Regs::RI0R::rtr));
            auto const dlc = get<0>(apply(read(Regs::RDT0R::dlc)));
            auto const lo  = Register::readWhole(Regs::RDL0R::FULLREGISTER);
            auto const hi  = Register::readWhole(Regs::RDH0R::FULLREGISTER);
            // RFOM0 is "rs": the whole register written with it alone, so FULL0/FOVR0 (rc_w1) get 0.
            apply(Regs::RF0R::overrideDefaults(
              write(Regs::RF0R::rfom0, Register::value<std::uint32_t, 1>())));

            Frame f{};
            f.extended = get<2>(ri) != 0;
            f.remote   = get<3>(ri) != 0;
            f.id       = f.extended ? (get<0>(ri) << 18) | get<1>(ri) : get<0>(ri);
            f.dlc      = static_cast<std::uint8_t>(dlc);
            for(unsigned b = 0; b < 4; ++b) {
                f.data[b]     = static_cast<std::byte>((lo >> (8 * b)) & 0xFFU);
                f.data[b + 4] = static_cast<std::byte>((hi >> (8 * b)) & 0xFFU);
            }
            return f;
        }

        struct Status {
            std::uint8_t tec;   // transmit error counter (ESR, 32.9.2)
            std::uint8_t rec;
            std::uint8_t
              lec;   // last error code: 0 none, 1 stuff, 2 form, 3 ack, 4 bit rec, 5 bit dom, 6 crc
            bool busOff;
            bool passive;
            bool initMode;   // MSR.INAK: not (yet) in normal mode
        };

        static Status status() {
            auto const esr = apply(read(Regs::ESR::tec,
                                        Regs::ESR::rec,
                                        Regs::ESR::lec,
                                        Regs::ESR::boff,
                                        Regs::ESR::epvf));   // one load
            return Status{static_cast<std::uint8_t>(get<0>(esr)),
                          static_cast<std::uint8_t>(get<1>(esr)),
                          static_cast<std::uint8_t>(get<2>(esr)),
                          get<3>(esr) != 0,
                          get<4>(esr) != 0,
                          apply(read(Regs::MSR::inak)) != 0};
        }
    };
}}   // namespace Kvasir::Can
