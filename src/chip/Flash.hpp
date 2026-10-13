#pragma once

#include "WaitBounds.hpp"
#include "kvasir/Register/Register.hpp"
#include "peripherals/FLASH.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

// Erasing and programming the STM32F413's own main flash (RM0430 Rev 8 3.5), 32 bits at a time
// (PSIZE x32, Table 7: 2.7-3.6 V). Sector map of the 1 MiB parts (Table 5 "Flash module
// organization"): sectors 0-3 16 KiB from 0x0800 0000, 4 64 KiB, 5-11 128 KiB up to 0x080F FFFF.
//
// No read-while-write: while a sector erases every flash read - code, vectors, interrupts - waits
// (an AHB stall, 3.5.1 note), for up to 2 s for a 128 KiB sector at x32 (DS11581 Rev 7 Table 51
// "Flash memory programming": tERASE128KB max 2 s, a word 100 us). A watchdog must be set longer,
// and nothing time-critical may depend on interrupts meanwhile.
namespace Kvasir { namespace Flash {
    enum class Result : std::uint8_t { ok, busy, writeProtected, programmingError, notErased };

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

    consteval Sector sectorAt(std::uint32_t address) {
        for(auto const& s : sectors) {
            if(s.address == address) { return s; }
        }
        return Sector{0xFF, 0, 0};
    }

    namespace Detail {
        using FLASH = Kvasir::Peripheral::FLASH::Registers<>;

        // SR (3.8.4): EOP and the error flags are rc_w1 (chip.svd: oneToClear), BSY read-only.
        inline constexpr std::uint32_t errors = FLASH::SR::operr.Mask | FLASH::SR::wrperr.Mask
                                              | FLASH::SR::pgaerr.Mask | FLASH::SR::pgperr.Mask
                                              | FLASH::SR::pgserr.Mask;

        inline void waitIdle() {
            Kvasir::Register::waitUntil<Kvasir::Chip::FlashBusyBound>(
              Kvasir::Register::isClear(FLASH::SR::bsy));
        }

        inline Result check() {
            // A snapshot of the flags, written back whole: clears exactly the rc_w1 flags it saw set.
            auto const s = Register::readWhole(FLASH::SR::FULLREGISTER);
            Register::writeWhole(FLASH::SR::FULLREGISTER, s & (errors | FLASH::SR::eop.Mask));
            if(Register::extract(FLASH::SR::wrperr, s) != 0) { return Result::writeProtected; }
            if((s & errors) != 0) { return Result::programmingError; }
            return Result::ok;
        }

        // 3.5.1: KEY1, KEY2 into KEYR; a wrong sequence locks CR until the next reset.
        inline void unlock() {
            if(apply(read(FLASH::CR::lock)) != 0) {
                apply(write(FLASH::KEYR::key, Register::value<std::uint32_t, 0x45670123>()));
                apply(write(FLASH::KEYR::key, Register::value<std::uint32_t, 0xCDEF89AB>()));
            }
        }

        inline void lock() { apply(write(FLASH::CR::lock, Register::value<std::uint32_t, 1>())); }

        // "If an erase operation in Flash memory also concerns data in the data or instruction
        // cache ... it is recommended to flush the caches" (3.5.4): DCRST/ICRST work only while
        // the cache is off (3.8.1).
        inline void flushCaches() {
            apply(write(FLASH::ACR::dcen, Register::value<std::uint32_t, 0>()),
                  write(FLASH::ACR::icen, Register::value<std::uint32_t, 0>()));
            apply(write(FLASH::ACR::dcrst, Register::value<std::uint32_t, 1>()),
                  write(FLASH::ACR::icrst, Register::value<std::uint32_t, 1>()));
            apply(write(FLASH::ACR::dcrst, Register::value<std::uint32_t, 0>()),
                  write(FLASH::ACR::icrst, Register::value<std::uint32_t, 0>()));
            apply(write(FLASH::ACR::dcen, Register::value<std::uint32_t, 1>()),
                  write(FLASH::ACR::icen, Register::value<std::uint32_t, 1>()));
        }
    }   // namespace Detail

    /// Sector erase, 3.5.3. Blocks every flash read for the whole erase (see above).
    inline Result eraseSector(unsigned number) {
        using namespace Detail;
        waitIdle();
        unlock();
        static_cast<void>(check());
        apply(write(FLASH::CR::psize, Register::value<std::uint32_t, 0b10>()),   // x32
              write(FLASH::CR::ser, Register::value<std::uint32_t, 1>()),
              write(FLASH::CR::snb, number));
        apply(write(FLASH::CR::strt, Register::value<std::uint32_t, 1>()));
        waitIdle();
        apply(write(FLASH::CR::ser, Register::value<std::uint32_t, 0>()));
        auto const r = check();
        lock();
        flushCaches();
        return r;
    }

    /// Word programming, 3.5.4: the words must be erased (0xFFFFFFFF) first.
    inline Result program(std::uint32_t                  address,
                          std::span<std::uint32_t const> words) {
        using namespace Detail;
        if(address % 4 != 0) { return Result::programmingError; }
        auto* target = reinterpret_cast<std::uint32_t volatile*>(address);
        for(std::size_t i = 0; i < words.size(); ++i) {
            if(target[i] != 0xFFFF'FFFF) { return Result::notErased; }
        }
        waitIdle();
        unlock();
        static_cast<void>(check());
        apply(write(FLASH::CR::psize, Register::value<std::uint32_t, 0b10>()),
              write(FLASH::CR::pg, Register::value<std::uint32_t, 1>()));
        Result r = Result::ok;
        for(std::size_t i = 0; i < words.size() && r == Result::ok; ++i) {
            target[i] = words[i];
            waitIdle();
            r = check();
        }
        apply(write(FLASH::CR::pg, Register::value<std::uint32_t, 0>()));
        lock();
        flushCaches();
        return r;
    }
}}   // namespace Kvasir::Flash
