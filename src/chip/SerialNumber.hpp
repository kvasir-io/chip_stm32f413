#pragma once
#include "kvasir/Register/Memory.hpp"
#include "kvasir/Util/HexString.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace Kvasir {
namespace SerialNumber {
    /// The 96-bit unique device ID (RM0430 Rev 8 35.1 "Unique device ID register (96 bits)", md l.57008): base
    /// 0x1FFF 7A10 (l.57022), U_ID[31:0] at offset 0x00 (l.57034), U_ID[63:32] at 0x04 (l.57050), U_ID[95:64] at 0x08
    /// (l.57069); factory-programmed, read only.
    inline constexpr std::array<std::uintptr_t, 3> addresses{0x1FFF'7A10, 0x1FFF'7A14, 0x1FFF'7A18};
}   // namespace SerialNumber

/// The chip's 96-bit unique ID: its three words from U_ID[31:0] up, each copied in memory byte order (as the SAM
/// chips' serialNumber()). Memory::loadPlain: an aligned, non-volatile load (the words never change).
inline std::array<std::byte,
                  12>
serialNumber() {
    std::array<std::byte, 12> ret;   // every byte written below
    std::byte*                it = ret.data();
    for(auto const address : SerialNumber::addresses) {
        auto const word = Kvasir::Memory::loadPlain<std::uint32_t>(address);
        std::memcpy(it, &word, sizeof(word));
        it += sizeof(word);
    }
    return ret;
}

/// serialNumber() as 24 upper-case hex digits (Util/HexString.hpp, every chip's form).
inline auto serialNumberString() { return Kvasir::hexString(Kvasir::serialNumber()); }
}   // namespace Kvasir
