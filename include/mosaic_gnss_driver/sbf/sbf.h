#ifndef MOSAIC_GNSS_DRIVER_SBF_H
#define MOSAIC_GNSS_DRIVER_SBF_H

#include <fstream>

namespace sbf {
    class SBF {
        static const size_t buffer_size = 256;

        std::ifstream &data;
        char buffer[buffer_size];

    public:
        explicit SBF(std::ifstream &in);

        bool seek_block();

        bool next_block();
    };
}

#endif //MOSAIC_GNSS_DRIVER_SBF_H
