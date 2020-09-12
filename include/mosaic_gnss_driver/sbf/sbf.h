#ifndef MOSAIC_GNSS_DRIVER_SBF_H
#define MOSAIC_GNSS_DRIVER_SBF_H

#include <fstream>

namespace sbf {
    /*
     * Using this class:
     *     - Construct and object of the class. This also initializes the various parsers.
     *     - Whenever you receive data, call `parse`.
     *
     * Adding more SBF Blocks:
     *     - `seek`, `read`, and `unread` are essentially black boxes and the implementation should not matter.
     *     - Add a parser in ____
     *
     */
    class SBF {
        // This string demarcates START_OF_BLOCK
        static constexpr const uint8_t sync_chars[2] = {'$', '@'};

        // Internal Buffer
        static const size_t buffer_size = 256; // Approximately max message size
        char _buffer[buffer_size] = {0}; // Any point in std::array ?
        uint8_t *const buffer = reinterpret_cast<uint8_t *const>(_buffer);
        size_t buffer_use{0};

        // Received Data
        const uint8_t *data_start{nullptr};
        const uint8_t *data_end{nullptr};

        // Helper pointers while parsing the data
        const uint8_t *read_ptr{nullptr}; // Current position in buffer + data
        const uint8_t *block_start{nullptr}; // Indicates the start of found block.


        // Parses the block that starts at `block_start`. Uses `read` to get more bytes.
        bool parse_block();


        // Helper functions for `seek`, `read`, and `unread`
        bool in_buffer(const uint8_t *const ptr) {
            return buffer <= ptr && ptr < buffer + buffer_size;
        }

        bool in_data(const uint8_t *const ptr) {
            return data_start <= ptr && ptr < data_end;
        }

        // Seeks through data until the next set of sync chars or end of data
        bool seek_block();

        // Reads the next `size` bytes
        const uint8_t *read(size_t size);

        // Moves the read_ptr back by rewind_len bytes
        void unread(size_t rewind_len);


        static bool check_crc(const uint8_t *bytes, size_t length, uint16_t crc);

    public:

        SBF();

        // Parse new data.
        void parse(const uint8_t *data, size_t size);
    };
}

#endif //MOSAIC_GNSS_DRIVER_SBF_H
