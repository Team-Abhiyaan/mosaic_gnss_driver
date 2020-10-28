#ifndef MOSAIC_GNSS_DRIVER_HELPERS_H
#define MOSAIC_GNSS_DRIVER_HELPERS_H

#include <cinttypes>

/**
 * Contains:
 *  - SBF <=> c++ types
 *  - Structs for common sbf fields
 */

#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedTypeAliasInspection" // Ignore unused sbf types
namespace sbf
{
    /*
     * Here: we cant directly cast b/c that depends on the system we are running in.
     * C++ does not guarantee little endian / big endian, twos complement /ones complement.
     * 
     * Using these functions makes it easy to change code to work on big endian systems if necessary 
     */

    // static_assert(LITTLE_ENDIAN, "Wrong endianness");
    // static_assert(TWOS_COMPLEMENT, "Wrong signed integer format");

    static_assert(sizeof(float) == 4, "Bad float size, no 4 byte floating point type");
    static_assert(sizeof(double) == 8, "Bad double size, no 8 byte floating point type");

    using float32_t = float;
    using float64_t = double;

    using c1 = char;
    using u1 = uint8_t;
    using u2 = uint16_t;
    using u4 = uint32_t;
    using f4 = float32_t;
    using f8 = float64_t;

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
    std::pair<uint16_t, uint8_t> parse_id(const uint16_t raw_id)
    {
        return {raw_id & 0b0001111111111111u, (raw_id & 0b1110000000000000u) >> 13u};
    }
} // namespace sbf
#pragma clang diagnostic pop
#endif //MOSAIC_GNSS_DRIVER_HELPERS_H

