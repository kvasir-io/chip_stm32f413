// A toggle of GPIO ODR bits is an ODR read and one BSRR write (src/chip/OdrToggle.hpp, RM0430 Rev 8
// 7.3.5): BS for the pins that read low, BR (bits 31:16) for those that read high, nothing written to
// ODR. Io's toggle(pin) and a hand-made XorLiteralAction on an ODR address take it; a toggle of any
// other register keeps the SDK's read-xor-write. Runs on the Kvasir SDK's register mock
// (kvasir::test_support) with KVASIR_REGISTER_MOCK.
#include "Io.hpp"
#include "kvasir/Io/PinFactories.hpp"
#include "test_registers.hpp"

using namespace Kvasir::Register;
using namespace Kvasir::Test;   // R, W: kvasir_test.hpp
using Kvasir::Io::PinLocation;

namespace {
constexpr unsigned odrA = 0x40020014, bsrrA = 0x40020018;
constexpr unsigned odrC = 0x40020814, bsrrC = 0x40020818;

using PC13 = PinLocation<2, 13>;
using PC14 = PinLocation<2, 14>;
using PA5  = PinLocation<0, 5>;

static_assert(Kvasir::Stm32::Gpio::isOdr(odrA) && Kvasir::Stm32::Gpio::isOdr(0x40021C14));
static_assert(!Kvasir::Stm32::Gpio::isOdr(0x40022014),
              "GPIOH is the last port");
static_assert(!Kvasir::Stm32::Gpio::isOdr(bsrrA) && !Kvasir::Stm32::Gpio::isOdr(0x40020010));

// an SVD-style ODR field and a register outside the GPIO
struct OdrLikeReg {
    using Addr = Address<odrC, 0x00000000, 0x00000000, std::uint32_t>;
    static constexpr FieldLocation<Addr, maskFromRange(13, 13), ReadWriteAccess, std::uint32_t>
      odr13{};
};

struct OtherReg {
    using Addr = Address<0x40000000, 0x00000000, 0x00000000, std::uint32_t>;
    static constexpr FieldLocation<Addr, maskFromRange(2, 2), ReadWriteAccess, std::uint32_t>
      bit2{};
};

void lowPinIsSet() {
    test("lowPinIsSet");
    recorder.setReadValue(odrC, 0);
    apply(toggle(PC13{}));
    checkActions({
      R{ odrC,        0},
      W{bsrrC, 1U << 13}
    });
}

void highPinIsReset() {
    test("highPinIsReset");
    recorder.setReadValue(odrC, 1U << 13);
    apply(toggle(PC13{}));
    checkActions({
      R{ odrC, 1U << 13},
      W{bsrrC, 1U << 29}
    });
}

void twoPinsOneAccess() {
    test("twoPinsOneAccess");
    recorder.setReadValue(odrC, 1U << 14);
    apply(toggle(PC13{}, PC14{}));
    checkActions({
      R{ odrC,                1U << 14},
      W{bsrrC, (1U << 13) | (1U << 30)}
    });
}

void otherPinsUntouched() {
    test("otherPinsUntouched");
    // every other pin of the port high: the write names pin 13 only
    recorder.setReadValue(odrC, 0xFFFFU & ~(1U << 13));
    apply(toggle(PC13{}));
    checkActions({
      R{ odrC, 0xFFFFU & ~(1U << 13)},
      W{bsrrC,              1U << 13}
    });
}

void twoPorts() {
    test("twoPorts");
    recorder.setReadValue(odrA, 1U << 5);
    recorder.setReadValue(odrC, 0);
    apply(toggle(PA5{}, PC13{}));
    // one read/write pair per port (in apply()'s register order)
    checkActions({
      R{ odrC,        0},
      W{bsrrC, 1U << 13},
      R{ odrA,  1U << 5},
      W{bsrrA, 1U << 21}
    });
}

void svdOdrField() {
    test("svdOdrField");
    recorder.setReadValue(odrC, 1U << 13);
    apply(toggle(OdrLikeReg::odr13));
    checkActions({
      R{ odrC, 1U << 13},
      W{bsrrC, 1U << 29}
    });
}

void otherRegisterKeepsXor() {
    test("otherRegisterKeepsXor");
    recorder.setReadValue(0x40000000, 0x13);
    apply(toggle(OtherReg::bit2));
    checkActions({
      R{0x40000000, 0x13},
      W{0x40000000, 0x17}
    });
}
}   // namespace

int main() {
    lowPinIsSet();
    highPinIsReset();
    twoPinsOneAccess();
    otherPinsUntouched();
    twoPorts();
    svdOdrField();
    otherRegisterKeepsXor();
    return report();
}
