#ifndef MOSAIC_GNSS_DRIVER_SBF_H
#define MOSAIC_GNSS_DRIVER_SBF_H

#include <unordered_map>
#include <mosaic_gnss_driver/data_buffers.h>
#include "block_parsers.h"

namespace sbf
{
    /**
     * Using this class:
     *     - Construct and object of the class. This also initializes the various parsers.
     *     - Whenever you receive data, call `parse` with the buffer storing the data.
     *
     * Adding more SBF Blocks:
     *     - `seek`, `read`, and `unread` are essentially black boxes and the implementation should not matter.
     *     - Add a parser in ____
     */

    class SBF
    {
        /// The string demarcating start of block.
        static constexpr const uint8_t sync_chars[2] = {'$', '@'};

        /// Internal Buffer
        static const size_t buffer_size = 256; // Approximately max message size
        char _buffer[buffer_size] = {0};       // Any point in std::array ?
        uint8_t *const buffer = reinterpret_cast<uint8_t *const>(_buffer);
        size_t buffer_use{0};

        /// Received Data
        const uint8_t *data_start{nullptr};
        const uint8_t *data_end{nullptr};

        /// Next data to read
        const uint8_t *read_ptr{nullptr};    // Current position in buffer + data
        /// Start of block being parsed
        const uint8_t *block_start{nullptr};

        mosaic_gnss_driver::DataBuffers &data_buf;

        // Block Parsers
        struct
        {
            mosaic_gnss_driver::DataBuffers &data_buf;
            sbf::block_parsers::Geodetic geodetic{data_buf};
        } parsers{data_buf};

        std::unordered_map<sbf::u4, std::function<void(const uint8_t *, const sbf::u2, const sbf::u1)>> parse_table{
                {4007, [&g = parsers.geodetic](auto block_ptr, auto len, auto rev_num)
                       { g.PVTGeodetic(block_ptr, len); }},
                {5906, [&g = parsers.geodetic](auto block_ptr, auto len, auto rev_num)
                       { g.PosCovGeodetic(block_ptr, len); }},
                {5908, [&g = parsers.geodetic](auto block_ptr, auto len, auto rev_num)
                       { g.VelCovGeodetic(block_ptr, len); }}
        };

        /**
         * Parses the block starting at `block_start`
         *
         * @return Whether data is exhausted
         */
        bool parse_block();

        bool in_buffer(const uint8_t *const ptr)
        {
            return buffer <= ptr && ptr < buffer + buffer_size;
        }

        bool in_data(const uint8_t *const ptr)
        {
            return data_start <= ptr && ptr < data_end;
        }

        /**
         * Seeks until sync str of block found, `sync_chars`
         *
         * Sets `block_start` and `read_ptr` to the location where block is found.
         *
         * If no more bytes to be read, clears the internal buffer and returns false.
         * If last byte is the first sync char, copies it to the internal buffer and returns false.
         *
         * @return true on block found, false on end of data
         */
        bool seek_block();

        /**
         * When inside a block, reads bytes. Ensures the read bytes are stored contiguously with the rest of the block.
         * requires `block_start` to be set.
         *
         * If no more bytes to read, copies partially read block to buffer, and returns nullptr
         *
         * @param size number of bytes to read
         * @return pointer to the location of the read bytes, nullptr if data exhausted
         */
        const uint8_t *read(size_t size);

        /**
         * Moves the read pointer backwards.
         * Used when a block turns out to be invalid.
         * @param rewind_len
         */
        void unread(size_t rewind_len);

        /**
         * Performs CRC check according to the SBF specification
         *
         * @param bytes : Start of bytes to be CRC checked
         * @param length  : Number of bytes to be CRC checked
         * @param crc : The correct CRC value
         * @return : Pass/Fail
         */
        static bool check_crc(const uint8_t *bytes, size_t length, uint16_t crc);

    public:
        explicit SBF(mosaic_gnss_driver::DataBuffers &buffers);

        /**
         * Searches for and parses SBF blocks.
         *
         * @param data Points to the new set of data
         * @param size Size of the new set of data
         */
        void parse(const uint8_t *data, size_t size);
    };
} // namespace sbf

#endif //MOSAIC_GNSS_DRIVER_SBF_H
