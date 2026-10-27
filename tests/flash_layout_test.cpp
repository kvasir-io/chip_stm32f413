// chip/FlashLayout.hpp: the sector map (RM0430 Rev 8 Table 5) and the geometry of an Nvm::Region. Building is the
// test; flash_layout_must_fail.cpp is the region the check refuses (sectors 3-4: 16 KiB + 64 KiB).
#include "FlashLayout.hpp"

namespace N = Kvasir::Nvm;

static_assert(N::sectorAt(0x0800'0000).number == 0 && N::sectorAt(0x0800'0000).size == 16 * 1024);
static_assert(N::sectorAt(0x0801'0000).number == 4 && N::sectorAt(0x0801'0000).size == 64 * 1024);
static_assert(N::sectorAt(0x080E'0000).number == 11 && N::sectorAt(0x080E'0000).size == 128 * 1024);
static_assert(N::sectorAt(0x080E'0004).number == 0xFF,
              "not a sector start");

// the map is contiguous and ends at 1 MiB
consteval bool contiguous() {
    for(std::size_t i = 1; i != N::sectors.size(); ++i) {
        if(N::sectors[i].address != N::sectors[i - 1].address + N::sectors[i - 1].size) {
            return false;
        }
        if(N::sectors[i].number != i) { return false; }
    }
    return N::sectors.back().address + N::sectors.back().size == 0x0810'0000;
}

static_assert(contiguous());

// sectors 9-10, 0-3 (the four 16 KiB ones), 11 alone
using G9 = N::RegionGeometry<0x080A'0000, 2>;
static_assert(G9::value.base == 0x080A'0000 && G9::value.sectorBytes == 128 * 1024
              && G9::value.pageBytes == 16 && G9::value.sectors == 2);
static_assert(N::RegionGeometry<0x0800'0000,
                                4>::value.sectorBytes
              == 16 * 1024);
static_assert(N::RegionGeometry<0x080E'0000,
                                1>::value.sectors
              == 1);

int main() { return 0; }
