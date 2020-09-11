#ifndef MOSAIC_GNSS_DRIVER_SBF_H
#define MOSAIC_GNSS_DRIVER_SBF_H

#include <fstream>

namespace sbf {
    class SBF {
        // Start of Block String
        static constexpr const uint8_t sync_chars[2] = {'$', '@'};

        // Internal Buffer Bounds and Usage
        static const size_t buffer_size = 256; // Approximately max message size
        char _buffer[buffer_size] = {0}; // Any point in std::array ?
        uint8_t *const buffer = reinterpret_cast<uint8_t *const>(_buffer);
        size_t buffer_use{0};

        // Data Bounds
        const uint8_t *data_start{nullptr};
        const uint8_t *data_end{nullptr};


        const uint8_t *read_ptr{nullptr}; // Current position in buffer + data
        const uint8_t *block_start{nullptr}; // Indicates the start of found block.


        // Helper functions for read()
        bool in_buffer(const uint8_t *const ptr) {
            return buffer <= ptr && ptr < buffer + buffer_size;
        }

        bool in_data(const uint8_t *const ptr) {
            return data_start <= ptr && ptr < data_end;
        }


        const uint8_t *read(size_t size);

        bool seek_block();

        bool parse_block();

    public:

        SBF();

        void parse(const uint8_t *data, size_t size);
    };
}

#endif //MOSAIC_GNSS_DRIVER_SBF_H
