#ifndef MOSAIC_GNSS_DRIVER_HELPERS_H
#define MOSAIC_GNSS_DRIVER_HELPERS_H

#include <cinttypes>


/*
 * Contains functions for:
 *  - converting SBF types to c++ types
 *  - Parsing common sbf fields
 */

namespace sbf {
    // Here: we cant directly cast b/c that depends on the system we are running in.
    // C++ does not guarantee little endian / big endian, twos complement /ones complement.

    // Using these functions makes it easy to change code to work on big endian systems if necessary

    // static_assert(LITTLE_ENDIAN, "Wrong endianness");
    // static_assert(TWOS_COMPLEMENT, "Wrong signed integer format");
    static_assert(sizeof(float) == 4, "Bad float size, no 4 byte floating point type");
    static_assert(sizeof(double) == 8, "Bad double size, no 8 byte floating point type");

    using float4_t = float;
    using float8_t = double;

    uint8_t u1(const uint8_t *buffer) {
        return *buffer;
    }

    uint16_t u2(const uint8_t *buffer) {
        return *reinterpret_cast<const uint16_t *>(buffer);
    }

    uint32_t u4(const uint8_t *buffer) {
        return *reinterpret_cast<const uint32_t *>(buffer);
    }

    int8_t i1(const uint8_t *buffer) {
        return *reinterpret_cast<const int8_t *>(buffer);
    }

    int16_t i2(const uint8_t *buffer) {
        return *reinterpret_cast<const int16_t *>(buffer);
    }

    int32_t i4(const uint8_t *buffer) {
        return *reinterpret_cast<const int32_t *>(buffer);
    }

    float4_t f4(const uint8_t *buffer) {
        return *reinterpret_cast<const float4_t *>(buffer);
    }

    float8_t f8(const uint8_t *buffer) {
        return *reinterpret_cast<const float8_t *>(buffer);
    }

    std::string c(const uint8_t *buffer, size_t size) {
        return std::string{reinterpret_cast<const char *>(buffer), size};
    }


    std::pair<uint16_t, uint8_t> parse_id(const uint16_t raw_id) {
        return {raw_id & 0b0001111111111111u, (raw_id & 0b1110000000000000u) >> 13u};
    }
}

#endif //MOSAIC_GNSS_DRIVER_HELPERS_H
