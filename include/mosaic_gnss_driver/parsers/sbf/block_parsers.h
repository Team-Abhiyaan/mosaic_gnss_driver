#ifndef MOSAIC_GNSS_DRIVER_BLOCK_PARSERS_H
#define MOSAIC_GNSS_DRIVER_BLOCK_PARSERS_H

#include <mosaic_gnss_driver/data_buffers.h>
#include "helpers.h"

namespace sbf::block_parsers
{
    class Geodetic
    {
        mosaic_gnss_driver::DataBuffers &db;

        decltype(db.nav_sat_fix)::ptr_t nav_sat_fix;
        decltype(db.velocity)::ptr_t velocity;

        // const static sbf::u4 max_dt = 100;
        const static sbf::u4 do_not_use_time = 4294967295; // TODO static member of block
        sbf::u4 pos_pvt_last_time{do_not_use_time}, pos_cov_last_time{do_not_use_time};
        sbf::u4 vel_pvt_last_time{do_not_use_time}, vel_cov_last_time{do_not_use_time};

    public:
        Geodetic(mosaic_gnss_driver::DataBuffers &data_buf) : db(data_buf)
        {}

        void PVTGeodetic(const uint8_t *block_ptr, sbf::u2 length);

        void PosCovGeodetic(const uint8_t *block_ptr, sbf::u2 length);

        void VelCovGeodetic(const uint8_t *block_ptr, sbf::u2 length);
    };
}
#endif //MOSAIC_GNSS_DRIVER_BLOCK_PARSERS_H
