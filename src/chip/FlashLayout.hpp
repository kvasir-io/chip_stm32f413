#pragma once
// The STM32F413's main-flash sector map (RM0430 Rev 8 Table 5 "Flash module organization", 1 MiB parts): sectors 0-3
// 16 KiB from 0x0800 0000, 4 64 KiB, 5-11 128 KiB up to 0x080F FFFF; and the geometry of an Nvm::Region over sectors
// of it. No register here (Flash.hpp drives the controller), so the host tests include it.
#include "kvasir/Hal/Flash.hpp"

#include <array>
#include <cstdint>

namespace Kvasir::Nvm {
struct Sector {
    unsigned      number;
    std::uint32_t address;
    std::uint32_t size;
};

inline constexpr std::array<Sector, 12> sectors{
  {{0, 0x0800'0000, 16 * 1024},
   {1, 0x0800'4000, 16 * 1024},
   {2, 0x0800'8000, 16 * 1024},
   {3, 0x0800'C000, 16 * 1024},
   {4, 0x0801'0000, 64 * 1024},
   {5, 0x0802'0000, 128 * 1024},
   {6, 0x0804'0000, 128 * 1024},
   {7, 0x0806'0000, 128 * 1024},
   {8, 0x0808'0000, 128 * 1024},
   {9, 0x080A'0000, 128 * 1024},
   {10, 0x080C'0000, 128 * 1024},
   {11, 0x080E'0000, 128 * 1024}}
};

/// The sector that starts at `address`; {0xFF, 0, 0} when none does.
consteval Sector sectorAt(std::uint32_t address) {
    for(auto const& s : sectors) {
        if(s.address == address) { return s; }
    }
    return Sector{0xFF, 0, 0};
}

/// A region's program unit: 16 bytes, one 128-bit flash row, so no write crosses one ("It is not allowed to program
/// data to the Flash memory that would cross the 128-bit row boundary", PGAERR; RM0430 Rev 8 3.5.4).
inline constexpr std::uint32_t regionPageBytes = 16;

/// `Sectors` sectors from `Base` as a Kvasir::Hal::FlashGeometry: `Base` starts a sector, the sectors after it exist and
/// all have one size (a NorRegion has one sectorBytes).
template<std::uint32_t Base, std::uint32_t Sectors>
struct RegionGeometry {
    static constexpr Sector first = sectorAt(Base);
    static_assert(first.number != 0xFF,
                  "an Nvm::Region starts on a sector (RM0430 Table 5)");
    static_assert(Sectors >= 1 && first.number + Sectors <= sectors.size(),
                  "the region's sectors exist");

    static consteval bool oneSize() {
        for(std::uint32_t i = 0; i != Sectors; ++i) {
            if(sectors[first.number + i].size != first.size) { return false; }
        }
        return true;
    }

    static_assert(oneSize(),
                  "an Nvm::Region's sectors all have one size");

    static constexpr Kvasir::Hal::FlashGeometry value{Base, first.size, regionPageBytes, Sectors};
};
}   // namespace Kvasir::Nvm
