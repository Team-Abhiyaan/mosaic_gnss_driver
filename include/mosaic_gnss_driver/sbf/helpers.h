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

    // static_assert(LITTLE_ENDIAN, "Wrong endianness");
    // static_assert(TWOS_COMPLEMENT, "Wrong signed integer format");
    static_assert(sizeof(float) == 4, "Bad float size, no 4 byte floating point type");
    static_assert(sizeof(double) == 8, "Bad double size, no 8 byte floating point type");

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

    float f4(const uint8_t *buffer) {
        return *reinterpret_cast<const float *>(buffer);
    }

    double f8(const uint8_t *buffer) {
        return *reinterpret_cast<const double *>(buffer);
    }

    std::string c(const uint8_t *buffer, size_t size) {
        return std::string{reinterpret_cast<const char *>(buffer), size};
    }


    std::pair<uint16_t, uint8_t> parse_id(const uint16_t raw_id) {
        return {raw_id & 0b0001111111111111u, (raw_id & 0b1110000000000000u) >> 13u};
    }
}

// Below is normal attempt without relying on architecture and should run anywhere, but not complete
/*
#if __cplusplus < 201300 // i.e before c++ 14
#define unsign(number) (static_cast<unsigned>(number))
#else

template<typename T>
auto unsign(T number) { return static_cast<unsigned>(number); }

#endif

namespace sbf {
    // Here: we cant directly cast b/c that depends on the system we are running in.
    // C++ does not guarantee little endian / big endian, twos complement ones complement.
    // No idea
    uint8_t u1(const uint8_t *buffer) {
        return *buffer;
    }

    uint16_t u2(const uint8_t *buffer) {
        return unsign(buffer[0]) | (unsign(buffer[1]) << 8u);
    }

    uint16_t u4(const uint8_t *buffer) {
        return unsign(buffer[0]) | (unsign(buffer[1]) << 8u) | (unsign(buffer[2]) << 16u) | (unsign(buffer[3]) << 24u);
    }

    int8_t i1(const uint8_t *buffer) {
        return ~(buffer[0] & ~(1u << 7u)) - (buffer[0] & (1u << 7u));
    }

    int8_t i2(const uint8_t *buffer) {
        return ~(buffer[0] & ~(1u << 7u)) - (buffer[0] & (1u << 7u));
    }

}*/
#endif //MOSAIC_GNSS_DRIVER_HELPERS_H
