#ifndef MOSAIC_GNSS_DRIVER_SBF_H
#define MOSAIC_GNSS_DRIVER_SBF_H

#include <fstream>

namespace sbf
{
    /**
     * Using this class:
     *     - Construct and object of the class. This also initializes the various parsers.
     *     - Whenever you receive data, call `parse`.
     *
     * Adding more SBF Blocks:
     *     - `seek`, `read`, and `unread` are essentially black boxes and the implementation should not matter.
     *     - Add a parser in ____
     *
     */
    class SBF
    {
        /// This string demarcates START_OF_BLOCK
        static constexpr const uint8_t sync_chars[2] = {'$', '@'};

        /// Internal Buffer
        static const size_t buffer_size = 256; // Approximately max message size
        char _buffer[buffer_size] = {0};       // Any point in std::array ?
        uint8_t *const buffer = reinterpret_cast<uint8_t *const>(_buffer);
        size_t buffer_use{0};

        /// Received Data
        const uint8_t *data_start{nullptr};
        const uint8_t *data_end{nullptr};

        /// Helper pointers while parsing the data
        const uint8_t *read_ptr{nullptr};    // Current position in buffer + data
        const uint8_t *block_start{nullptr}; // Indicates the start of found block.

        /**
         * Parses the block starting at `block_start`
         *
         * @return Whether data is not exhausted (i.e. if any read returns nullptr, returns false)
         */
        bool parse_block();

        /**
         * Helper function for `seek`, `read`, and `unread`
         * 
         */
        bool in_buffer(const uint8_t *const ptr)
        {
            return buffer <= ptr && ptr < buffer + buffer_size;
        }

        /**
         * Helper function for `seek`, `read`, and `unread`
         * 
         */
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
         * @return pointer to the location of the read bytes, nullptr if data over
         */
        const uint8_t *read(size_t size);

        /**
         * Moves the read pointer backwards.
         * Used when a block turns out to be invalid.
         * @param rewind_len
         */
        // TODO: Check
        void unread(size_t rewind_len);

        /**
         * Performs CRC check according to the SBF specification
         *
         * @param bytes : Start of bytes to be CRC checked
         * @param length  : Number of bytes to be CRC checked
         * @param crc : The correct CRC value
         * @return : Pass/Fail
         */
        // TODO: Implement
        static bool check_crc(const uint8_t *bytes, size_t length, uint16_t crc);

    public:
        /**
         * Constructor
         */
        SBF();

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
