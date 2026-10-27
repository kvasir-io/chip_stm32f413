#pragma once

#include "ClockMath.hpp"
#include "Clocks.hpp"
#include "Instances.hpp"
#include "Io.hpp"
#include "Rcc.hpp"
#include "WaitBounds.hpp"
#include "kvasir/Hal/Owns.hpp"
#include "kvasir/Port/Extends.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Backoff.hpp"
#include "peripherals/CAN.hpp"
#include "peripherals/RCC.hpp"

#include <array>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <optional>

// bxCAN of the STM32F413 (RM0430 Rev 8 chapter 32), polled: send into a free transmit mailbox,
// receive from FIFO 0, every frame accepted (filter bank 0, mask 0). The bit timing is solved at
// compile time from PCLK1 (32.7.7: bit = (1 + TS1 + TS2) tq, tq = (BRP + 1) / PCLK1, TS1 1-16,
// TS2 1-8 quanta), exact, with the sample point nearest the wanted one.
//
//     using Can = Kvasir::Can::Bus<HW::ClockTree, 1, 500'000, RxPin, TxPin, 8, Clock>;
//     Can::send(frame);                          // Result::ok, or ::full (no mailbox free)
//     while(auto f = Can::receive()) { ... }
//
// Bus-off is left by the hardware (MCR.ABOM, 32.7.6): no error needs a person or a reset. Startup
// requests normal mode and does not wait for it - it is reached after 11 recessive bits (32.4.2),
// which a board without a bus never sees; until then frames wait in their mailboxes. An
// initialization that did not get its INAK (initFailed) is retried by turn(), the bus's Hook::MainLoop
// entry, on a doubling pause (Kvasir::Backoff on TimeClock: 1 s up to 10 min) until it succeeds - a
// firmware that lists the bus and runs `Startup::run<Kvasir::Hook::MainLoop>()` has it.
namespace Kvasir { namespace Can {
    struct Frame {
        std::uint32_t            id{};   // 11 bits, or 29 with extended
        bool                     extended{};
        bool                     remote{};
        std::uint8_t             dlc{};   // 0-8
        std::array<std::byte, 8> data{};
    };

    enum class Result : std::uint8_t { ok, full, badFrame };

    // The bit timing solver (BitTiming, solveBitTiming): ClockMath.hpp, register-free and host-tested.

    template<unsigned N>
    struct Instance;

    // CAN1 on APB1 (RM0430 Table 1, Instances.hpp; 6.3.15 APB1ENR.CAN1EN). CAN2 shares CAN1's filters
    // and needs its clock (32.3.3): not supported here.
    template<>
    struct Instance<1> : Rcc::OnBusOf<Peripheral::CAN::Registers<1>> {};

    template<typename Clocks,
             unsigned      N,
             std::uint32_t Bitrate,
             typename RxPin,
             typename TxPin,
             int Af,
             typename TimeClock,   // Clock::now() for the init retry's pauses (turn())
             unsigned SamplePermille = 875>
    struct Bus {
        using I    = Instance<N>;
        using Regs = Kvasir::Peripheral::CAN::Registers<N>;

        static constexpr std::uint32_t pclkHz = Clocks::template pclkHz<I::bus>;

        // Startup: this block, and the bus clock its dividers are computed from (Clocks.hpp).
        using Provides = brigand::list<Hal::Owns<Regs>>;
        using Claims   = Hal::Claim<Kvasir::Clocks::ClkApb<I::bus>, pclkHz>;

        static constexpr BitTiming timing = solveBitTiming(pclkHz, Bitrate, SamplePermille);
        static_assert(timing.brp != 0,
                      "no exact CAN bit timing for this bit rate from PCLK1 (RM0430 32.7.7)");
        static_assert(timing.samplePermille >= 750 && timing.samplePermille <= 900,
                      "CAN sample point outside 75-90 %");
        static constexpr auto powerClockEnable  = I::powerClockEnable;
        static constexpr auto initStepPinConfig = Io::txRxPins<Af, TxPin, RxPin>();

        static inline std::uint32_t rxOverruns{};
        // INAK did not come: "the CAN hardware waits until the current CAN activity (transmission or
        // reception) is completed before entering the initialization mode" (32.9.2 CAN_MCR, INRQ):
        // a CANRX that never goes idle, for one - or the CAN has no clock.
        // turn() tries again; `reinits` counts the initializations it got through.
        static inline bool          initFailed{};
        static inline std::uint32_t reinits{};

    private:
        // The data bytes as the two mailbox words: DATA0 is bits 7:0 of TDLxR/RDLxR, DATA4 of TDHxR/RDHxR (32.9.3,
        // md l.48203, l.48243), and the core stores a word's lowest byte first (little endian, 2.2.1, md l.2645) -
        // so the words are the bytes in memory order.
        using DataWords = std::array<std::uint32_t, 2>;

        // TIxR, TDTxR, TDLxR, TDHxR of each transmit mailbox (32.9.3).
        using TxMailboxes = brigand::list<brigand::list<typename Regs::TI0R,
                                                        typename Regs::TDT0R,
                                                        typename Regs::TDL0R,
                                                        typename Regs::TDH0R>,
                                          brigand::list<typename Regs::TI1R,
                                                        typename Regs::TDT1R,
                                                        typename Regs::TDL1R,
                                                        typename Regs::TDH1R>,
                                          brigand::list<typename Regs::TI2R,
                                                        typename Regs::TDT2R,
                                                        typename Regs::TDL2R,
                                                        typename Regs::TDH2R>>;

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

        // Transmit mailbox K, TXRQ last ("set up the identifier, the data length code (DLC) and the
        // data before requesting the transmission by setting the corresponding TXRQ bit", 32.7.1):
        // separate applies, so the stores keep this order.
        template<unsigned K>
        static void load(Frame const& f) {
            using M     = brigand::at_c<TxMailboxes, K>;
            using TI    = brigand::at_c<M, 0>;
            using TDT   = brigand::at_c<M, 1>;
            using TDL   = brigand::at_c<M, 2>;
            using TDH   = brigand::at_c<M, 3>;
            Id const id = idOf(f);
            writeId<TI>(id, Register::value<std::uint32_t, 0>());
            // TGT and TIME as 0, as at reset: the write covers the register, one store
            apply(write(TDT::time, Register::value<std::uint32_t, 0>()),
                  clear(TDT::tgt),
                  write(TDT::dlc, std::uint32_t{f.dlc}));
            auto const words = std::bit_cast<DataWords>(f.data);
            Register::writeWhole(TDL::FULLREGISTER, words[0]);
            Register::writeWhole(TDH::FULLREGISTER, words[1]);
            writeId<TI>(id, Register::value<std::uint32_t, 1>());
        }

    public:
        /// Initialization mode (MCR.INRQ, MSR.INAK; 32.4.1), BTR, filter bank 0 = everything to
        /// FIFO 0 (32.7.4), then normal mode requested.
        static void runtimeInit() {
            // Leaves Sleep (reset state) for initialization: SLEEP 0, INRQ 1 (32.4.3). ABOM: bus-off
            // left by the hardware; TXFP: mailboxes go out in request order; DBF as at reset (the
            // CAN stops while the core is halted).
            apply(Regs::MCR::overrideDefaults(set(Regs::MCR::dbf),
                                              set(Regs::MCR::abom),
                                              set(Regs::MCR::txfp),
                                              clear(Regs::MCR::sleep),
                                              set(Regs::MCR::inrq)));
            // bounded (WaitBounds.hpp), Report whatever policy the application injects
            if(!Register::waitUntil<Kvasir::Chip::CanInitBound<Bitrate>,
                                    Register::OnTimeout::Report>(Register::isSet(Regs::MSR::inak)))
            {
                initFailed = true;
                return;
            }
            initFailed = false;
            // BTR (32.9.2): BRP - 1, TS1 - 1, TS2 - 1, SJW 1 tq (0); loop back and silent off.
            apply(write(Regs::BTR::brp, Register::value<std::uint32_t, timing.brp - 1>()),
                  write(Regs::BTR::ts1, Register::value<std::uint32_t, timing.ts1 - 1>()),
                  write(Regs::BTR::ts2, Register::value<std::uint32_t, timing.ts2 - 1>()),
                  write(Regs::BTR::sjw, Register::value<std::uint32_t, 0>()),
                  clear(Regs::BTR::lbkm),
                  clear(Regs::BTR::silm));

            // FMR.FINIT (its reset value, CAN2SB 14, kept), bank 0 off, 32-bit mask mode, FIFO 0,
            // id and mask 0, on (32.9.4).
            apply(set(Regs::FMR::finit));
            apply(clear(Regs::FA1R::fact0));
            apply(clear(Regs::FM1R::fbm0),
                  set(Regs::FS1R::fsc0),
                  clear(Regs::FFA1R::ffa0),
                  write(Regs::F0R1::FULLREGISTER, Register::value<std::uint32_t, 0>()),
                  write(Regs::F0R2::FULLREGISTER, Register::value<std::uint32_t, 0>()));
            apply(set(Regs::FA1R::fact0));
            apply(clear(Regs::FMR::finit));

            apply(clear(Regs::MCR::inrq));
        }

        /// The Hook::MainLoop entry: while the initialization has failed, runtimeInit() again once the
        /// pause is over (the first retry 1 s after the failure is seen, doubling up to 10 min); nothing
        /// once it succeeded.
        static void turn() {
            if(!initFailed) { return; }
            auto const now = TimeClock::now();
            // the failure of Startup's runtimeInit(), seen for the first time: the first pause starts
            if(retry_.failures == 0) {
                retry_.failed(now);
                return;
            }
            if(!retry_.due(now)) { return; }
            runtimeInit();
            if(initFailed) {
                retry_.failed(now);
            } else {
                retry_.succeeded();
                ++reinits;
            }
        }

        using Extends = Startup::Extend<Kvasir::Hook::MainLoop, &Bus::turn>;

    private:
        static inline Kvasir::Backoff<TimeClock> retry_{};

    public:
        /// Into the next free transmit mailbox (TSR.CODE, 32.9.2), or full.
        static Result send(Frame const& f) {
            if(f.dlc > 8) { return Result::badFrame; }
            // TME0-2 and CODE (the next free mailbox), one TSR load
            auto const tsr
              = apply(read(Regs::TSR::tme0, Regs::TSR::tme1, Regs::TSR::tme2, Regs::TSR::code));
            if(get<0>(tsr) == 0 && get<1>(tsr) == 0 && get<2>(tsr) == 0) { return Result::full; }
            switch(get<3>(tsr)) {
            case 0:  load<0>(f); break;
            case 1:  load<1>(f); break;
            default: load<2>(f); break;
            }
            return Result::ok;
        }

        /// The oldest frame in FIFO 0, released afterwards (RF0R.RFOM0); an overrun is counted.
        static std::optional<Frame> receive() {
            auto const rf0r = apply(read(Regs::RF0R::fovr0, Regs::RF0R::fmp0));   // one load
            if(get<0>(rf0r) != 0) {   // FOVR0, rc_w1: the whole register written, FOVR0 alone
                ++rxOverruns;
                apply(Regs::RF0R::overrideDefaults(set(Regs::RF0R::fovr0)));
            }
            if(get<1>(rf0r) == 0) { return std::nullopt; }   // FMP0
            // The mailbox, one load per register, all before the release below.
            auto const ri
              = apply(read(Regs::RI0R::stid, Regs::RI0R::exid, Regs::RI0R::ide, Regs::RI0R::rtr));
            auto const dlc = get<0>(apply(read(Regs::RDT0R::dlc)));
            auto const lo  = Register::readWhole(Regs::RDL0R::FULLREGISTER);
            auto const hi  = Register::readWhole(Regs::RDH0R::FULLREGISTER);
            // RFOM0 is "rs": the whole register written with it alone, so FULL0/FOVR0 (rc_w1) get 0.
            apply(Regs::RF0R::overrideDefaults(set(Regs::RF0R::rfom0)));

            Frame f{};
            f.extended = get<2>(ri) != 0;
            f.remote   = get<3>(ri) != 0;
            f.id       = f.extended ? (get<0>(ri) << 18) | get<1>(ri) : get<0>(ri);
            f.dlc      = static_cast<std::uint8_t>(dlc);
            f.data     = std::bit_cast<std::array<std::byte, 8>>(DataWords{lo, hi});
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
