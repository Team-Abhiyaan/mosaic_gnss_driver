#include <mosaic_gnss_driver/parsers/sbf/block_parsers.h>
#include <cinttypes>
#include <cmath>

namespace sbf::block_parsers
{

#pragma pack(push, 1) // Packs the struct tightly, no gaps b/w objs
#pragma clang diagnostic push
#pragma ide diagnostic ignored "OCUnusedGlobalDeclarationInspection"
    namespace structs
    {
        struct PVTCartesian
        { /// SBF BlockNum 4006
            u4 TOW; // DO_NOT_USE: 4294967295
            u2 WNc;
            u1 Mode, Error;
            f8 x, y, z;
            f4 undulation, vx, vy, vz, cog;
            f8 rxclkbias;
            f4 rxclkdrifk;
            u1 time_system, datum, num_satellites, wa_corr_info;
            u2 referenceID, mean_corr_age;
            u4 signal_info;
            u1 alert_flag, num_bases;
            u2 ppp_info, latency, haccuracy;
            u1 misc;
        };
        struct PosCovCartesian
        { /// SBF BlockNum 5905
            u4 TOW;
            u2 WNc;
            u1 Mode, Error;
            f4 xx, yy, zz, bb, xy, xz, xb, yz, yb, zb;
        };
        struct VelCovCartesian
        { /// SBF BlockNum 5907
            u4 TOW;
            u2 WNc;
            u1 Mode, Error;
            // Covariances
            f4 vx_vx, vy_vy, vz_vz, dt_dt, vx_vy, vx_vz, vx_dt, vy_vz, vy_dt, vz_dt;
        };
    }

#pragma clang diagnostic pop
#pragma pack(pop)

    void Cartesian::PVTCartesian(const uint8_t *block_ptr, const sbf::u2 length)
    {
        if (length < sizeof(structs::PVTCartesian))
        {
            std::cout << "[WARN] Block is too small." << std::endl;
            return;
        }
        auto block = reinterpret_cast<const structs::PVTCartesian *>(block_ptr);
        if ((block->Mode & 0b1111u) == 0u) // PVT Error
        {
            std::cout << "[WARN] PVT Error" << std::endl;
            return;
        }


        ///////////////
        //   Pose    //
        ///////////////
        // Create ptr if required
        if (!pose)
        {
            pose = db.pose.get_new_ptr();
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
        pose->header.stamp = ros::Time::now(); // TODO: use GNSS time
        pose->header.frame_id = "geodetic";
        static auto rad2deg = [](auto val)
        { return val * 180 / 3.141592653; };
        auto &position = pose->pose.pose.position;
        position.x = block->x;
        position.y = block->y;
        position.z = block->z;
        if (block->cog != -2e10)
        {
            pose->pose.pose.orientation.z = cos(M_PI * block->cog / 180.0);
            pose->pose.pose.orientation.w = sin(M_PI * block->cog / 180.0);
        } else
        {
            pose->pose.pose.orientation.z = 0;
            pose->pose.pose.orientation.w = 1;
        }
        // Publish if cov also filled
        if (pos_cov_last_time != do_not_use_time)
            db.pose.set_ptr(std::move(pose));

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
        velocity->header.frame_id = "gnss_cart";

        velocity->twist.twist.linear.x = block->vx;
        velocity->twist.twist.linear.y = block->vy;
        velocity->twist.twist.linear.z = block->vz;

        // Publish if cov also filled
        if (vel_cov_last_time != do_not_use_time)
            db.velocity.set_ptr(std::move(velocity));

    }

    void Cartesian::PosCovCartesian(const uint8_t *block_ptr, const sbf::u2 length)
    {
        if (length < sizeof(structs::PosCovCartesian))
        {
            std::cout << "[WARN] Block is too small." << std::endl;
            return;
        }
        auto block = reinterpret_cast<const structs::PosCovCartesian *>(block_ptr);
        if ((block->Mode & 0b1111u) == 0u) // PVT Error
        {
            std::cout << "[WARN] PVT Error" << std::endl;
            return;
        }

        // Create ptr if required
        if (!pose)
        {
            pose = db.pose.get_new_ptr();
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
        auto &c = pose->pose.covariance;
        c[0] = block->xx;
        c[1] = block->xy;
        c[2] = block->xz;

        c[0 + 6] = block->xy;
        c[1 + 6] = block->yy;
        c[2 + 6] = block->yz;

        c[0 + 12] = block->xz;
        c[1 + 12] = block->yz;
        c[2 + 12] = block->zz;

        // Publish if pvt also filled
        if (pos_pvt_last_time != do_not_use_time)
        {
            db.pose.set_ptr(std::move(pose));

        }
    }

    void Cartesian::VelCovCartesian(const uint8_t *block_ptr, const sbf::u2 length)
    {
        if (length < sizeof(structs::VelCovCartesian))
        {
            std::cout << "[WARN] Block is too small." << std::endl;
            return;
        }
        auto block = reinterpret_cast<const structs::VelCovCartesian *>(block_ptr);
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
        c[0] = block->vx_vx;
        c[1] = block->vx_vy;
        c[2] = block->vx_vz;

        c[0 + 6] = block->vx_vy;
        c[1 + 6] = block->vy_vy;
        c[2 + 6] = block->vy_vz;

        c[0 + 12] = block->vx_vz;
        c[1 + 12] = block->vy_vz;
        c[2 + 12] = block->vz_vz;

        // Publish if pvt also filled
        if (vel_pvt_last_time != do_not_use_time)
            db.velocity.set_ptr(std::move(velocity));
    }
}