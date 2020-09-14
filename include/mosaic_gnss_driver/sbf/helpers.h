#ifndef MOSAIC_GNSS_DRIVER_HELPERS_H
#define MOSAIC_GNSS_DRIVER_HELPERS_H

#include <cinttypes>

/**
 * Contains functions for:
 *  - converting SBF types to c++ types
 *  - Parsing common sbf fields
 */

namespace sbf {
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

    using float4_t = float;
    using float8_t = double;

#pragma pack(push, 1) // Packs the struct tightly, no gaps b/w objs
    struct Header { /// SBF Header
        char sync_chars[2];
        uint16_t CRC;
        uint16_t ID;
        uint16_t length;
    };
    struct PVTGeodetic { /// SBF BlockNum 4007
        uint32_t TOW;
        uint16_t WNc;
        uint8_t Mode, Error;
        float8_t Latitude, Longitude, Height;
        float4_t undulation, vn, ve, vu, cog;
        float8_t rxclkbias;
        float4_t rxclkdrifk;
        uint8_t time_system, datum, num_satellites, wa_corr_info;
        uint16_t referenceID, mean_corr_age;
        uint32_t signal_info;
        uint8_t alert_flag, num_bases;
        uint16_t ppp_info, latency, haccuracy;
        uint8_t misc;
    };
#pragma pack(pop)

    /// Helper Functions for converting sbf types to c types
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

    /// Gets the ID and Revision Number from the SBF ID field
    std::pair<uint16_t, uint8_t> parse_id(const uint16_t raw_id) {
        return {raw_id & 0b0001111111111111u, (raw_id & 0b1110000000000000u) >> 13u};
    }
} // namespace sbf

#endif //MOSAIC_GNSS_DRIVER_HELPERS_H
