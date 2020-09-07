#ifndef MOSAIC_GNSS_DRIVER_SBF_H
#define MOSAIC_GNSS_DRIVER_SBF_H

#include <fstream>

namespace sbf {
    class SBF {
        std::ifstream &data;

    public:
        explicit SBF(std::ifstream &in);

        bool seek_block();
    };
}

#endif //MOSAIC_GNSS_DRIVER_SBF_H
