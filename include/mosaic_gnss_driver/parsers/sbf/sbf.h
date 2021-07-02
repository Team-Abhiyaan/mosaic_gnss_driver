#ifndef MOSAIC_GNSS_DRIVER_SBF_H
#define MOSAIC_GNSS_DRIVER_SBF_H

#include <mosaic_gnss_driver/data_buffers.h>
#include <mosaic_gnss_driver/parsers/sbf/block_parsers.h>
#include <unordered_map>

namespace sbf {
    /**
     * Using this class:
     *     - Construct and object of the class. This also initializes the various parsers.
     *     - Whenever you receive data, call `parse` with the buffer storing the data.
     *     - Use the parsed data in the `DataBuffer`
     *
     * Adding more SBF Blocks:
     *     - `seek`, `read`, and `unread` are essentially black boxes and its implementation should
     * not matter to you.
     *     - To add more sbf blocks, create a parser:
     *          - define it in `block_parsers.h`
     *          - add it to the struct `SBF.parsers`
     *          - add its block ids to `SBF.parse_table`.
     */

    class SBF
    {

        // Reading Data:

        /// The string demarcating start of block.
        static constexpr const uint8_t sync_chars[2] = {'$', '@'};

        /// Internal Buffer
        static const size_t buffer_size = 256; // Should be more than max block size
        char _buffer[buffer_size] = {0};
        uint8_t* const buffer = reinterpret_cast<uint8_t* const>(_buffer);
        size_t buffer_length_used{0};

        /// Received Data
        const uint8_t* data_start{nullptr};
        const uint8_t* data_end{nullptr};

        /// Next data to read
        const uint8_t* read_ptr{nullptr}; // Current position in buffer + data
        /// Start of block currently being parsed
        const uint8_t* block_start{nullptr};

        mosaic_gnss_driver::DataBuffers& data_buf;

        // Parsing Blocks:
        using parse_table_t =
            std::unordered_map<sbf::u4,
                               std::function<void(const uint8_t*, const sbf::u2, const sbf::u1)>>;
        parse_table_t parse_table{};

        /**
         * Parses the block starting at `block_start`
         *
         * @return Whether data is exhausted
         */
        bool parse_block();

        bool in_buffer(const uint8_t* const ptr)
        {
            return buffer <= ptr && ptr < buffer + buffer_size;
        }

        bool in_data(const uint8_t* const ptr) { return data_start <= ptr && ptr < data_end; }

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
         * When inside a block, reads bytes. Ensures the read bytes are stored contiguously with the
         * rest of the block. requires `block_start` to be set.
         *
         * If no more bytes to read, copies partially read block to buffer, and returns nullptr
         *
         * @param size number of bytes to read
         * @return pointer to the location of the read bytes, nullptr if data exhausted
         */
        const uint8_t* read(size_t size);

        /**
         * Moves the read pointer backwards.
         * Used when a block turns out to be invalid.
         * @param rewind_len
         */
        void unread(size_t rewind_len);

        /**
         * Performs CRC check according to the SBF specification
         *
         * TODO: Implement
         *
         * @param bytes : Start of bytes to be CRC checked
         * @param length  : Number of bytes to be CRC checked
         * @param crc : The correct CRC value
         * @return : Pass/Fail
         */
        static bool check_crc(const uint8_t* bytes, const size_t length, const uint16_t crc);

    public:
        explicit SBF(mosaic_gnss_driver::DataBuffers& buffers);

        /**
         * Searches for and parses SBF blocks.
         *
         * @param data Points to the new set of data
         * @param size Size of the new set of data
         */
        void parse(const uint8_t* data, size_t size);

        struct
        {
            mosaic_gnss_driver::DataBuffers& data_buf;
            parse_table_t& pt;

            sbf::block_parsers::Geodetic geodetic{data_buf};
            sbf::block_parsers::Cartesian cartesian{data_buf};

            void enable_geodetic()
            {
                data_buf.nav_sat_fix.enabled = true;
                data_buf.velocity.enabled = true;

                pt[4007] = [&g = geodetic](auto block_ptr, auto len, auto rev_num) {
                    g.PVTGeodetic(block_ptr, len, rev_num);
                };
                pt[5906] = [&g = geodetic](auto block_ptr, auto len, auto rev_num) {
                    g.PosCovGeodetic(block_ptr, len, rev_num);
                };
                pt[5908] = [&g = geodetic](auto block_ptr, auto len, auto rev_num) {
                    g.VelCovGeodetic(block_ptr, len, rev_num);
                };
            }

            void enable_cartesian()
            {
                data_buf.pose.enabled = true;
                data_buf.velocity.enabled = true;

                pt[4006] = [&g = cartesian](auto block_ptr, auto len, auto rev_num) {
                    g.PVTCartesian(block_ptr, len, rev_num);
                };
                pt[5905] = [&g = cartesian](auto block_ptr, auto len, auto rev_num) {
                    g.PosCovCartesian(block_ptr, len, rev_num);
                };
                pt[5907] = [&g = cartesian](auto block_ptr, auto len, auto rev_num) {
                    g.VelCovCartesian(block_ptr, len, rev_num);
                };
            }

        } parsers{data_buf, parse_table};
    };
} // namespace sbf

// CRC lookup table for 16-bit CRC for SBF blocks (makes the computation faster)
static const uint16_t CRC_LOOKUP_16[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b,
    0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
    0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96,
    0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb,
    0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2,
    0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827,
    0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d,
    0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

#endif // MOSAIC_GNSS_DRIVER_SBF_H
