#ifndef MOSAIC_GNSS_DRIVER_HELPERS_H
#define MOSAIC_GNSS_DRIVER_HELPERS_H

#include <cinttypes>

/**
 * Contains functions for:
 *  - converting SBF types to c++ types
 *  - Parsing common sbf fields
 */

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
    struct PVTGeodetic
    { /// SBF BlockNum 4007
        u4 TOW; // DO_NOT_USE: 4294967295
        u2 WNc;
        u1 Mode, Error;
        f8 Latitude, Longitude, Height;
        f4 undulation, vn, ve, vu, cog;
        f8 rxclkbias;
        f4 rxclkdrifk;
        u1 time_system, datum, num_satellites, wa_corr_info;
        u2 referenceID, mean_corr_age;
        u4 signal_info;
        u1 alert_flag, num_bases;
        u2 ppp_info, latency, haccuracy;
        u1 misc;
    };

    struct PosCovGeodetic
    { /// SBF BlockNum 5906
        u4 TOW;
        u2 WNc;
        u1 Mode, Error;
        // lat - Latitude, lon - Longitude, hgt - Height, bias - Clock Bias
        f4 lat_lat, lon_lon, hgt_hgt, bias_bias, lat_lon, lat_hgt, lat_bias, lon_hgt, lon_bias, hgt_bias;
    };

    struct VelCovGeodetic
    { /// SBF BlockNum 5908
        u4 TOW;
        u2 WNc;
        u1 Mode, Error;
        // Covariances
        f4 vn_vn, ve_ve, vu_vu, dt_dt, vn_ve, vn_vu, vn_dt, ve_vu, ve_dt, vu_dt;
    };

#pragma pack(pop)

    /// Gets the ID and Revision Number from the SBF ID field
    std::pair<uint16_t, uint8_t> parse_id(const uint16_t raw_id)
    {
        return {raw_id & 0b0001111111111111u, (raw_id & 0b1110000000000000u) >> 13u};
    }
} // namespace sbf

#endif //MOSAIC_GNSS_DRIVER_HELPERS_H
