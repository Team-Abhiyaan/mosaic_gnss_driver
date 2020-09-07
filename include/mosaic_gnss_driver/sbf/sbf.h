#ifndef MOSAIC_GNSS_DRIVER_SBF_H
#define MOSAIC_GNSS_DRIVER_SBF_H

#include <fstream>

namespace sbf {
    class SBF {
        std::ifstream &input;

        static const size_t buffer_size = 256;
        char _buffer[buffer_size]; // Any point in std::array ?
        char *const buffer = _buffer;

        bool seek_block();

    public:
        explicit SBF(std::ifstream &in);

        bool parse_next();
    };
}

#endif //MOSAIC_GNSS_DRIVER_SBF_H
