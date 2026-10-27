#pragma once

// The two byte queues of a Usart::Port (Usart.hpp) and what the port does with them, register-free so a host test
// counts them (tests/usart_queue_test.cpp): a queue holds exactly the N its knob says (Atomic::Fifo: one slot more
// inside, which keeps full apart from empty), `send` queues what fits and returns how many, and a received byte that
// finds its queue full is dropped and counted - never dropped and reported as taken.
#include "kvasir/Atomic/Queue.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

namespace Kvasir { namespace Usart { namespace detail {
    template<std::size_t N>
    using ByteQueue = Atomic::Fifo<std::byte, N, Atomic::OverflowPolicyIgnore>;

    /// Queues what fits of `bytes` in one range push and returns how many. The producer's side: the consumer (the
    /// interrupt) only takes from the queue meanwhile, so the room it had stays.
    template<typename Queue>
    std::size_t pushWhatFits(Queue&                     queue,
                             std::span<std::byte const> bytes) {
        std::size_t const n = std::min(bytes.size(), queue.capacity() - queue.size());
        if(n != 0) { queue.push(bytes.first(n)); }
        return n;
    }

    /// One received byte into `queue`; false, and `dropped` counts it, when the queue is full.
    template<typename Queue>
    bool pushOrCount(Queue&         queue,
                     std::byte      byte,
                     std::uint32_t& dropped) {
        if(queue.size() < queue.capacity()) {
            queue.push(byte);
            return true;
        }
        ++dropped;
        return false;
    }
}}}   // namespace Kvasir::Usart::detail
