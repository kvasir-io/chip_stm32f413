#pragma once
// The GPIO port layout, and a toggle of ODR bits as an ODR read plus one BSRR write.
//
// The GPIO has no toggle register. A toggle (Register::XorLiteralAction on a GPIOx_ODR address -
// Io's Action::Toggle, merged multi-pin toggles and an SVD `toggle(GPIOx::ODR::odrN)` alike) reads
// ODR and writes BSRR: BS for the toggled pins that read low, BR for those that read high
// (RM0430 Rev 8 7.3.5 "I/O data bitwise handling": BSRR(i) sets ODR(i), BSRR(i+16) resets it, a 0
// does nothing; 7.3: BSRR is there "to allow atomic read/modify accesses"). The write touches only
// the toggled pins, so a concurrent write of ANOTHER pin of the port (an ISR, a DMA) is kept. A
// writer of the SAME pin between the read and the write is still overwritten - Register::atomic's
// contract (it protects the other fields, two writers of one field race).
//
// The access is the Register::ExecuteSeam hook (Kvasir_SDK Register/Exec.hpp) for Kvasir::Tag::User: cmake/chip.cmake
// names this header in KVASIR_CHIP_REGISTER_SEAM, and Exec.hpp includes it at its end, so every TU of an image has it
// before its first apply(). Only the SDK's Tags, Exec and Mmio here, and the port layout as constants: it is read in the
// middle of the register layer (a generated GPIO header would include Register.hpp half-way).
#include "kvasir/Common/Tags.hpp"
#include "kvasir/Register/Exec.hpp"
#include "kvasir/Register/Mmio.hpp"

namespace Kvasir::Stm32::Gpio {
// GPIOA; port n at + n * 0x400 (RM0430 2.2.2 Table 1 "STM32F413/423 register boundary addresses":
// GPIOA 0x4002 0000 ... GPIOH 0x4002 1C00).
inline constexpr unsigned baseAddress = 0x40020000;
inline constexpr unsigned portOffset  = 0x400;
// Ports A (0) to H (7).
inline constexpr int portCount = 8;
// RM0430 7.4.6 GPIOx_ODR (ODR15:0 rw, 31:16 reserved), 7.4.7 GPIOx_BSRR (BS15:0 in 15:0, BR15:0
// in 31:16, write-only).
inline constexpr unsigned odrOffset  = 0x14;
inline constexpr unsigned bsrrOffset = 0x18;

constexpr bool isOdr(unsigned address) {
    return address >= baseAddress
        && address < baseAddress + static_cast<unsigned>(portCount) * portOffset
        && (address - baseAddress) % portOffset == odrOffset;
}
}   // namespace Kvasir::Stm32::Gpio

namespace Kvasir::Register {
template<unsigned A,
         unsigned Z,
         unsigned O,
         typename T,
         typename M,
         unsigned Mask,
         typename Acc,
         typename FT,
         unsigned Data>
    requires(Stm32::Gpio::isOdr(A))
struct ExecuteSeam<
  ::Kvasir::Tag::User,
  Action<FieldLocation<Address<A, Z, O, T, M>, Mask, Acc, FT>, XorLiteralAction<Data>>> {
    static_assert((Data & ~0xFFFFU) == 0,
                  "GPIOx_ODR has 16 pins (RM0430 7.4.6)");
    static_assert((Data & ~Mask) == 0,
                  "bad mask");

    [[gnu::always_inline]] unsigned operator()(unsigned = 0) {
        unsigned const high
          = RegisterTraits<Address<A, Z, O, T, M>>::read() & Data;   // these go low
        // BS (bits 15:0) for the pins that are low, BR (bits 31:16) for those that are high
        Mmio<T>::store(A - Stm32::Gpio::odrOffset + Stm32::Gpio::bsrrOffset,
                       static_cast<T>((Data & ~high) | (high << 16U)));
        return 0;
    }
};
}   // namespace Kvasir::Register
