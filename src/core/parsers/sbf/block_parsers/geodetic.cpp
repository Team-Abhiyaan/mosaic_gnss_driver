#include <cinttypes>
#include <mosaic_gnss_driver/parsers/sbf/block_parsers.h>

namespace sbf::block_parsers
{

#pragma pack(push, 1) // Packs the struct tightly, no gaps b/w objs
#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
    namespace structs
    {
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
    }
#pragma clang diagnostic pop
#pragma pack(pop)

    void Geodetic::PVTGeodetic(const uint8_t *block_ptr, const sbf::u2 length)
    {
        if (length < sizeof(structs::PVTGeodetic))
        {
            std::cout << "[WARN] Block is too small." << std::endl;
            return;
        }
        auto block = reinterpret_cast<const structs::PVTGeodetic *>(block_ptr);
        if ((block->Mode & 0b1111u) == 0u) // PVT Error
        {
            std::cout << "[WARN] PVT Error" << std::endl;
            return;
        }


        ///////////////
        // NavSatFix //
        ///////////////
        // Create ptr if required
        if (!nav_sat_fix)
        {
            nav_sat_fix = db.nav_sat_fix.get_new_ptr();
            pos_cov_last_time = pos_pvt_last_time = do_not_use_time;
        }

        pos_pvt_last_time = block->TOW;
        // Check for mismatch times
        if ((pos_cov_last_time != do_not_use_time) && pos_pvt_last_time != pos_cov_last_time) // TODO range?
        {
            std::cout << "[WARN] Timestamp mismatch" << std::endl;
            pos_cov_last_time = do_not_use_time;
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
        if (pos_cov_last_time != do_not_use_time)
            db.nav_sat_fix.set_ptr(std::move(nav_sat_fix));


        //////////////
        // Velocity //
        //////////////

        // Create ptr if required
        if (!velocity)
        {
            velocity = db.velocity.get_new_ptr();
            vel_cov_last_time = vel_pvt_last_time = do_not_use_time;
        }

        vel_pvt_last_time = block->TOW;
        // Check for mismatch times
        if ((vel_cov_last_time != do_not_use_time) && vel_pvt_last_time != vel_cov_last_time) // TODO range?
        {
            std::cout << "[WARN] Timestamp mismatch" << std::endl;
            vel_cov_last_time = do_not_use_time;
        }


        // Fill Vel
        velocity->header.stamp = ros::Time::now(); // TODO: use GNSS time
        velocity->header.frame_id = "geodetic";

        velocity->twist.twist.linear.x = block->vn;
        velocity->twist.twist.linear.y = block->ve;
        velocity->twist.twist.linear.z = block->vu;

        // Publish if cov also filled
        if (vel_cov_last_time != do_not_use_time)
            db.velocity.set_ptr(std::move(velocity));


    }

    void Geodetic::PosCovGeodetic(const uint8_t *block_ptr, const sbf::u2 length)
    {
        if (length < sizeof(structs::PosCovGeodetic))
        {
            std::cout << "[WARN] Block is too small." << std::endl;
            return;
        }
        auto block = reinterpret_cast<const structs::PosCovGeodetic *>(block_ptr);
        if ((block->Mode & 0b1111u) == 0u) // PVT Error
        {
            std::cout << "[WARN] PVT Error" << std::endl;
            return;
        }

        // Create ptr if required
        if (!nav_sat_fix)
        {
            nav_sat_fix = db.nav_sat_fix.get_new_ptr();
            pos_cov_last_time = pos_pvt_last_time = do_not_use_time;
        }

        pos_cov_last_time = block->TOW;
        // Check for mismatch times
        if ((pos_pvt_last_time != do_not_use_time) && pos_pvt_last_time != pos_cov_last_time) // TODO range?
        {
            std::cout << "[WARN] Timestamp mismatch" << std::endl;
            pos_pvt_last_time = do_not_use_time;
        }

        // Fill Cov
        nav_sat_fix->position_covariance = {
                block->lat_lat, block->lat_lon, block->lat_hgt,
                block->lat_lon, block->lon_lon, block->lon_hgt,
                block->lat_hgt, block->lon_hgt, block->hgt_hgt
        };
        nav_sat_fix->position_covariance_type = 3; // COVARIANCE_TYPE_KNOWN

        // Publish if pvt also filled
        if (pos_pvt_last_time != do_not_use_time)
            db.nav_sat_fix.set_ptr(std::move(nav_sat_fix));
    }

    void Geodetic::VelCovGeodetic(const uint8_t *block_ptr, const sbf::u2 length)
    {
        if (length < sizeof(structs::VelCovGeodetic))
        {
            std::cout << "[WARN] Block is too small." << std::endl;
            return;
        }
        auto block = reinterpret_cast<const structs::VelCovGeodetic *>(block_ptr);
        if ((block->Mode & 0b1111u) == 0u) // PVT Error
        {
            std::cout << "[WARN] PVT Error" << std::endl;
            return;
        }

        // Create ptr if required
        if (!velocity)
        {
            velocity = db.velocity.get_new_ptr();
            vel_cov_last_time = vel_pvt_last_time = do_not_use_time;
        }

        vel_cov_last_time = block->TOW;
        // Check for mismatch times
        if ((vel_pvt_last_time != do_not_use_time) && vel_pvt_last_time != vel_cov_last_time) // TODO range?
        {
            std::cout << "[WARN] Timestamp mismatch" << std::endl;
            vel_pvt_last_time = do_not_use_time;
        }

        // Fill Cov
        auto &c = velocity->twist.covariance;
        c[0] = block->vn_vn;
        c[1] = block->vn_ve;
        c[2] = block->vn_vu;

        c[0 + 6] = block->vn_ve;
        c[1 + 6] = block->ve_ve;
        c[2 + 6] = block->ve_vu;

        c[0 + 12] = block->vn_vu;
        c[1 + 12] = block->ve_vu;
        c[2 + 12] = block->vu_vu;

        // Publish if pvt also filled
        if (vel_pvt_last_time != do_not_use_time)
            db.velocity.set_ptr(std::move(velocity));
    }
}