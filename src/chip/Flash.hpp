#pragma once

#include "FlashLayout.hpp"
#include "WaitBounds.hpp"
#include "kvasir/Hal/Flash.hpp"
#include "kvasir/Register/Memory.hpp"
#include "kvasir/Register/Register.hpp"
#include "kvasir/Util/Result.hpp"
#include "peripherals/FLASH.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <expected>
#include <span>

// Erasing and programming the STM32F413's own main flash (RM0430 Rev 8 3.5), 32 bits at a time
// (PSIZE x32, Table 7: 2.7-3.6 V); the sector map is FlashLayout.hpp's. Kvasir::Nvm, not Kvasir::Flash: that is the
// namespace of CellStore. Nvm::Region<Base, Sectors> is a Kvasir::Hal::NorRegion over sectors of one size, for
// CellStore.
//
// A call returns Nvm::Result<> = Kvasir::Result<void, Nvm::Error> (an std::expected), the same name and shape as on the
// SAM chips (chip_atsam_common NvmResult.hpp); the error values are this controller's.
//
// No read-while-write: while a sector erases every flash read - code, vectors, interrupts - waits
// (an AHB stall, 3.5.1 note), for up to 2 s for a 128 KiB sector at x32 (DS11581 Rev 7 Table 51
// "Flash memory programming": tERASE128KB max 2 s, a word 100 us). A watchdog must be set longer,
// and nothing time-critical may depend on interrupts meanwhile.
namespace Kvasir { namespace Nvm {
    enum class Error : std::uint8_t {
        writeProtected,   ///< WRPERR: the sector is write protected (3.8.4); nothing was done
        programmingError,   ///< OPERR, PGAERR, PGPERR or PGSERR (3.8.4), or an address that is not word-aligned
        notErased,   ///< a word to program is not 0xFFFFFFFF; nothing was written (checked before the operation)
    };

    template<typename T = void>
    using Result = Kvasir::Result<T, Error>;

    namespace Detail {
        using FLASH = Kvasir::Peripheral::FLASH::Registers<>;

        // SR (3.8.4): EOP and the error flags are rc_w1 (chip.svd: oneToClear), BSY read-only.
        inline constexpr std::uint32_t errors = FLASH::SR::operr.mask | FLASH::SR::wrperr.mask
                                              | FLASH::SR::pgaerr.mask | FLASH::SR::pgperr.mask
                                              | FLASH::SR::pgserr.mask;

        inline void waitIdle() {
            Kvasir::Register::waitUntil<Kvasir::Chip::FlashBusyBound>(
              Kvasir::Register::isClear(FLASH::SR::bsy));
        }

        inline Result<> check() {
            // A snapshot of the flags, written back whole: clears exactly the rc_w1 flags it saw set.
            auto const s = Register::readWhole(FLASH::SR::FULLREGISTER);
            Register::writeWhole(FLASH::SR::FULLREGISTER, s & (errors | FLASH::SR::eop.mask));
            if(Register::extract(FLASH::SR::wrperr, s) != 0) {
                return std::unexpected{Error::writeProtected};
            }
            if((s & errors) != 0) { return std::unexpected{Error::programmingError}; }
            return {};
        }

        // 3.5.1: KEY1, KEY2 into KEYR; a wrong sequence locks CR until the next reset.
        inline void unlock() {
            if(apply(read(FLASH::CR::lock)) != 0) {
                apply(write(FLASH::KEYR::key, Register::value<std::uint32_t, 0x45670123>()));
                apply(write(FLASH::KEYR::key, Register::value<std::uint32_t, 0xCDEF89AB>()));
            }
        }

        inline void lock() { apply(set(FLASH::CR::lock)); }

        // "If an erase operation in Flash memory also concerns data in the data or instruction
        // cache ... it is recommended to flush the caches" (3.5.4): DCRST/ICRST work only while
        // the cache is off (3.8.1).
        inline void flushCaches() {
            apply(clear(FLASH::ACR::dcen), clear(FLASH::ACR::icen));
            apply(set(FLASH::ACR::dcrst), set(FLASH::ACR::icrst));
            apply(write(FLASH::ACR::dcrst, Register::value<std::uint32_t, 0>()),
                  write(FLASH::ACR::icrst, Register::value<std::uint32_t, 0>()));
            apply(set(FLASH::ACR::dcen), set(FLASH::ACR::icen));
        }

        // The frame of one erase or program (3.5.3, 3.5.4): BSY clear, CR unlocked, the flags of an earlier
        // operation cleared, PSIZE x32 with the operation's CR bit (SER, PG) and its other CR fields in one write;
        // then the operation, which clears its bit itself (an erase reads its flags only after that CR write,
        // which stalls until BSY drops: 3.5.1 note, md l.3195); then CR locked and the caches flushed.
        template<typename Operation,
                 typename Bit,
                 typename... CrFields>
        inline Result<> operation(Operation&& op,
                                  Bit         bit,
                                  CrFields... crFields) {
            waitIdle();
            unlock();
            static_cast<void>(check());
            apply(write(FLASH::CR::psize, Register::value<std::uint32_t, 0b10>()),   // x32
                  set(bit),
                  crFields...);
            Result<> const r = op();
            lock();
            flushCaches();
            return r;
        }
    }   // namespace Detail

    /// Sector erase, 3.5.3. Blocks every flash read for the whole erase (see above).
    inline Result<> eraseSector(unsigned number) {
        using namespace Detail;
        return operation(
          [] {
              apply(set(FLASH::CR::strt));
              waitIdle();
              apply(clear(FLASH::CR::ser));
              return check();
          },
          FLASH::CR::ser,
          write(FLASH::CR::snb, number));
    }

    /// Word programming, 3.5.4: the words must be erased (0xFFFFFFFF) first.
    inline Result<> program(std::uint32_t                  address,
                            std::span<std::uint32_t const> words) {
        using namespace Detail;
        if(address % 4 != 0) { return std::unexpected{Error::programmingError}; }
        Kvasir::Memory::Words<std::uint32_t> const target{address};
        for(std::size_t i = 0; i < words.size(); ++i) {
            if(target.load(i) != 0xFFFF'FFFF) { return std::unexpected{Error::notErased}; }
        }
        return operation(
          [&] {
              Result<> r{};
              for(std::size_t i = 0; i < words.size() && r.has_value(); ++i) {
                  target.store(i, words[i]);
                  waitIdle();
                  r = check();
              }
              apply(clear(FLASH::CR::pg));
              return r;
          },
          FLASH::CR::pg);
    }

    /// `Sectors` sectors of one size from `Base` (a sector start, RM0430 Table 5; checked at compile time) as a
    /// Kvasir::Hal::NorRegion: a sector is the erase unit, a 16-byte flash row the program unit (a write across
    /// pages is split). Erased words of a page (0xFFFFFFFF) are left out: programming them would change no bit
    /// (3.5.4: "Successive write operations are possible without the need of an erase operation when changing bits
    /// from '1' to '0'"), and program() refuses a word that is not erased. The image check is FlashRegionBase's:
    /// the images start at 0x0800 0000, so a region must lie past the image's end.
    ///
    ///     using Region = Kvasir::Nvm::Region<0x080A'0000, 2>;    // sectors 9-10, 2 x 128 KiB
    template<std::uint32_t Base, std::uint32_t Sectors>
    struct Region
      : Kvasir::Hal::FlashRegionBase<Region<Base, Sectors>,
                                     RegionGeometry<Base, Sectors>::value,
                                     Kvasir::Hal::PageSplit::split> {
        static constexpr std::uint32_t pageBytes = regionPageBytes;
        static constexpr unsigned      first     = RegionGeometry<Base, Sectors>::first.number;

        static void readAt(std::uint32_t        addr,
                           std::span<std::byte> out) {
            Kvasir::Memory::loadBytes(Base + addr, out.data(), out.size());
        }

        static bool programPage(std::uint32_t        page,
                                std::span<std::byte const,
                                          pageBytes> bytes) {
            std::array<std::uint32_t, pageBytes / 4> words{};
            std::memcpy(words.data(), bytes.data(), pageBytes);
            std::size_t i = 0;
            while(i != words.size()) {
                if(words[i] == 0xFFFF'FFFFU) {
                    ++i;
                    continue;
                }
                std::size_t n = i;
                while(n != words.size() && words[n] != 0xFFFF'FFFFU) { ++n; }
                if(!Nvm::program(Base + page + static_cast<std::uint32_t>(i * 4),
                                 std::span<std::uint32_t const>{words}.subspan(i, n - i)))
                {
                    return false;
                }
                i = n;
            }
            return true;
        }

        static bool eraseSector(std::uint32_t sector) {
            return Nvm::eraseSector(first + sector).has_value();
        }
    };
}}   // namespace Kvasir::Nvm
