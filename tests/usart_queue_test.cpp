// src/chip/UsartQueue.hpp: a Usart::Port queue holds exactly the N its TxSize / RxSize knob says, `send` takes what
// fits and says how many, and a received byte that finds the queue full is counted as dropped (a check of
// `size() >= TxSize` is never true for a Queue<T, TxSize>, which holds TxSize - 1: a full queue would drop the byte and
// report it sent).
#include "UsartQueue.hpp"

#include <array>
#include <cstdio>

namespace {
using Kvasir::Usart::detail::ByteQueue;
using Kvasir::Usart::detail::pushOrCount;
using Kvasir::Usart::detail::pushWhatFits;

static_assert(ByteQueue<16>::capacity() == 16);
static_assert(ByteQueue<256>::capacity() == 256,
              "the port's default: 256 bytes, one slot more inside");
static_assert(ByteQueue<1>::capacity() == 1);

int failures = 0;

void check(bool        ok,
           char const* what) {
    if(!ok) {
        ++failures;
        std::printf("FAIL: %s\n", what);
    }
}

template<std::size_t N>
void sendAtExactlyN() {
    ByteQueue<N>                 q{};
    std::array<std::byte, N + 4> bytes{};
    for(std::size_t i = 0; i < bytes.size(); ++i) { bytes[i] = static_cast<std::byte>(i); }

    check(pushWhatFits(q, std::span<std::byte const>{bytes}) == N, "send takes exactly N of N + 4");
    check(q.size() == N, "the queue holds N");
    check(pushWhatFits(q, std::span<std::byte const>{bytes}.first(1)) == 0,
          "a full queue takes nothing");
    std::byte b{};
    check(q.popInto(b) && b == std::byte{0}, "the oldest comes out first");
    check(pushWhatFits(q, std::span<std::byte const>{bytes}.first(3)) == 1,
          "one slot free: one byte taken");
    check(pushWhatFits(q, std::span<std::byte const>{}) == 0, "an empty send takes nothing");
}

template<std::size_t N>
void receiveAtExactlyN() {
    ByteQueue<N>  q{};
    std::uint32_t dropped = 0;
    bool          all     = true;
    for(std::size_t i = 0; i < N; ++i) {
        all = pushOrCount(q, static_cast<std::byte>(i), dropped) && all;
    }
    check(all && dropped == 0, "N received bytes all fit");
    check(!pushOrCount(q, std::byte{0xAA}, dropped) && dropped == 1,
          "byte N + 1 is dropped and counted");
    std::byte b{};
    check(q.popInto(b), "one taken out");
    check(pushOrCount(q, std::byte{0x55}, dropped) && dropped == 1, "then one fits again");
    std::size_t n = 0;
    while(q.popInto(b)) { ++n; }
    check(n == N, "and the queue held N");
}
}   // namespace

int main() {
    sendAtExactlyN<16>();
    sendAtExactlyN<256>();
    receiveAtExactlyN<1>();
    receiveAtExactlyN<16>();
    receiveAtExactlyN<256>();
    std::printf("usart_queue: %d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
