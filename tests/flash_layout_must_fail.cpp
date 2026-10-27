// Must NOT compile: an Nvm::Region over sectors 3 (16 KiB) and 4 (64 KiB) - a NorRegion has one sectorBytes.
// ctest builds it and expects the static_assert's text (CMakeLists.txt: flash_layout_must_fail).
#include "FlashLayout.hpp"

#if FLASH_MUST_FAIL == 1
constexpr auto g = Kvasir::Nvm::RegionGeometry<0x0800'C000, 2>::value;
#elif FLASH_MUST_FAIL == 2   // not a sector start
constexpr auto g = Kvasir::Nvm::RegionGeometry<0x0800'C004, 1>::value;
#elif FLASH_MUST_FAIL == 3   // past sector 11
constexpr auto g = Kvasir::Nvm::RegionGeometry<0x080E'0000, 2>::value;
#endif

int main() { return static_cast<int>(g.sectors); }
