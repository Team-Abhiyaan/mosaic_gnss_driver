#ifndef MOSAIC_GNSS_DRIVER_HELPERS_H
#define MOSAIC_GNSS_DRIVER_HELPERS_H

#include <cinttypes> // For ints of specific width.
#include <endian.h>  // For little endian check
#include <memory>    // For unique_ptr / make_unique

/**
 * Contains:
 *  - SBF <=> c++ types
 *  - Structs for common sbf fields
 */

namespace sbf {
    /*
     * Here: we should not directly cast because that depends on the system we are running in.
     * C++ does not guarantee little endian / big endian, twos complement / ones complement.
     *
     * We have assumed a little endian and twos complement system for simplicity.
     */

#if __BYTE_ORDER != __LITTLE_ENDIAN
    static_assert(false, "Wrong endianess, requires little endian system");
#endif

    // TWOS_COMPLEMENT
    static_assert((uint32_t)(-1) == (uint32_t)0xFFFFFFFFuL,
                  "Wrong signed integer format. Require two's complement");

    static_assert(sizeof(float) == 4, "Bad float size, no 4 byte floating point type");
    static_assert(sizeof(double) == 8, "Bad double size, no 8 byte floating point type");

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedTypeAliasInspection" // Ignore unused sbf types
    using float32_t = float;
    using float64_t = double;

    // SBF Data types
    using c1 = char;
    using u1 = uint8_t;
    using u2 = uint16_t;
    using u4 = uint32_t;
    using f4 = float32_t;
    using f8 = float64_t;
#pragma clang diagnostic pop

#pragma pack(push, 1) // Packs the struct tightly, no gaps b/w objs
    struct Header
    { /// SBF Header
        char sync_chars[2];
        u2 CRC;
        u2 ID;
        u2 length;
    };
#pragma pack(pop)

    /// Gets the ID and Revision Number from the SBF ID field
    // TODO: Find the double definition and remove the `inline`
    inline std::pair<uint16_t, uint8_t> parse_id(const uint16_t raw_id)
    {
        return {raw_id & 0b0001111111111111u, (raw_id & 0b1110000000000000u) >> 13u};
    }

} // namespace sbf
#endif // MOSAIC_GNSS_DRIVER_HELPERS_H
