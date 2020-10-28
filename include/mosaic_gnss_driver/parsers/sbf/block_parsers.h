#ifndef MOSAIC_GNSS_DRIVER_BLOCK_PARSERS_H
#define MOSAIC_GNSS_DRIVER_BLOCK_PARSERS_H

#include "helpers.h"

namespace sbf::block_parsers
{
    class Geodetic
    {
        mosaic_gnss_driver::DataBuffers &db;

        decltype(db.nav_sat_fix)::ptr_t nav_sat_fix;

        // const static sbf::u4 max_dt = 100;
        const static sbf::u4 do_not_use_time = 4294967295; // TODO static member of block
        sbf::u4 pvt_last_time{0}, cov_last_time{0};

    public:
        Geodetic(mosaic_gnss_driver::DataBuffers &data_buf) : db(data_buf)
        {}

        void PVTGeodetic(const uint8_t *block_ptr, const sbf::u2 length)
        {
            if (length < sizeof(sbf::PVTGeodetic))
            {
                std::cout << "[WARN] Block is too small." << std::endl;
                return;
            }
            auto block = reinterpret_cast<const sbf::PVTGeodetic *>(block_ptr);
            if ((block->Mode & 0b1111u) == 0u) // PVT Error
            {
                std::cout << "[WARN] PVT Error" << std::endl;
                return;
            }

            // NavSatFix

            // Create ptr if required
            if (!nav_sat_fix)
            {
                nav_sat_fix = db.nav_sat_fix.get_new_ptr();
                cov_last_time = pvt_last_time = do_not_use_time;
            }

            pvt_last_time = block->TOW;
            // Check for mismatch times
            if ((cov_last_time != do_not_use_time) && pvt_last_time != cov_last_time) // TODO range?
            {
                std::cout << "[WARN] Timestamp mismatch" << std::endl;
                cov_last_time = do_not_use_time;
            }


            // Fill PVT
            nav_sat_fix->header.stamp = ros::Time::now(); // TODO: use GNSS time
            nav_sat_fix->header.frame_id = "geodetic";
            static auto rad2deg = [](auto val)
            { return val * 180 / 3.141592653; };
            nav_sat_fix->latitude = rad2deg(block->Latitude);
            nav_sat_fix->longitude = rad2deg(block->Longitude);
            nav_sat_fix->altitude = block->Height;

            // Publish if cov also filled
            if (cov_last_time != do_not_use_time)
                db.nav_sat_fix.set_ptr(std::move(nav_sat_fix));


        }

        void PosCovGeodetic(const uint8_t *block_ptr, const sbf::u2 length)
        {
            if (length < sizeof(sbf::PosCovGeodetic))
            {
                std::cout << "[WARN] Block is too small." << std::endl;
                return;
            }
            auto block = reinterpret_cast<const sbf::PosCovGeodetic *>(block_ptr);
            if ((block->Mode & 0b1111u) == 0u) // PVT Error
            {
                std::cout << "[WARN] PVT Error" << std::endl;
                return;
            }

            // Create ptr if required
            if (!nav_sat_fix)
            {
                nav_sat_fix = db.nav_sat_fix.get_new_ptr();
                cov_last_time = pvt_last_time = do_not_use_time;
            }

            cov_last_time = block->TOW;
            // Check for mismatch times
            if ((pvt_last_time != do_not_use_time) && pvt_last_time != cov_last_time) // TODO range?
            {
                std::cout << "[WARN] Timestamp mismatch" << std::endl;
                pvt_last_time = do_not_use_time;
            }

            // Fill Cov
            nav_sat_fix->position_covariance = {
                    block->lat_lat, block->lat_lon, block->lat_hgt,
                    block->lat_lon, block->lon_lon, block->lon_hgt,
                    block->lat_hgt, block->lon_hgt, block->hgt_hgt
            };
            nav_sat_fix->position_covariance_type = 3; // COVARIANCE_TYPE_KNOWN

            // Publish if pvt also filled
            if (pvt_last_time != do_not_use_time)
                db.nav_sat_fix.set_ptr(std::move(nav_sat_fix));
        }
    };
}
#endif //MOSAIC_GNSS_DRIVER_BLOCK_PARSERS_H
