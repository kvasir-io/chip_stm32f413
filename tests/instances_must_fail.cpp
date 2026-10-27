// Must NOT compile: a block that is on no APB (GPIOA, AHB1, RM0430 Table 1 md l.2710) has no APB bus to take a clock
// enable from. ctest builds it and expects the refusal's name (CMakeLists.txt: instances_must_fail).
#include "Instances.hpp"

constexpr auto bus = Kvasir::Chip::busOfBase(0x4002'0000);

int main() { return static_cast<int>(bus); }
