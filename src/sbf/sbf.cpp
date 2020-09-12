#include "mosaic_gnss_driver/sbf/sbf.h"
#include "mosaic_gnss_driver/sbf/helpers.h"

#include <iostream>
#include <cassert>
#include <cstring>
#include <tuple>

sbf::SBF::SBF() = default;

/**
 * Searches for and parses SBF blocks.
 *
 * @param data Points to the new set of data
 * @param size Size of the new set of data
 */
void sbf::SBF::parse(const uint8_t *const data, size_t size)
{
    if (data == nullptr || size == 0)
        return;

    data_start = data;
    data_end = data_start + size;

    // Read from buffer if it has data
    read_ptr = buffer_use ? buffer : data_start;

    // Parse Blocks until no more data
    while (seek_block() && parse_block())
        ;
}

/**
 * Parses the block starting at `block_start`
 *
 * @return Whether data is not exhausted (i.e. if any read returns nullptr, returns false)
 */
bool sbf::SBF::parse_block()
{
    const uint8_t *ret;

    // Read Header
    ret = read(8);
    if (!ret)
        return false;
    auto header = reinterpret_cast<const sbf::Header *>(ret);

    assert(header->sync_chars[0] == sync_chars[0] && header->sync_chars[1] == sync_chars[1]);

    uint16_t id;
    uint8_t rev_num;
    std::tie(id, rev_num) = sbf::parse_id(header->ID);
    auto length = header->length;

    // Check Length
    if (length % 4 != 0 || length <= 8 || length > 123)
    {
        // std::cout << "Invalid block length" << std::endl;
        unread(sizeof(Header) - 2);
        return true;
    }

    // Block Length is
    const auto block_data_length = length - 8;
    ret = read(block_data_length);
    if (!ret)
        return false;

    if (!check_crc(block_start + 4, length - 4, header->CRC))
    {
        unread(length - 2);
        return true;
    }

    std::cout << id << "\t" << (int)rev_num << "\t" << length << "\t" << header->CRC << std::endl;

    // Call the parser
    // TODO: Implement
    // auto parser = parsers[id]
    // parser(rev_num, pares_ptr, parse_ptr_end);

    if (id == 4007)
    {
        std::cout << "Found PVTGeodetic" << std::endl;
        auto pvtgeodectic = reinterpret_cast<const sbf::PVTGeodetic *>(ret);
        std::cout << pvtgeodectic->Latitude * 180 / 3.14159 << std::endl
                  << pvtgeodectic->Longitude * 180 / 3.14159 << std::endl
                  << (int)pvtgeodectic->Mode << std::endl
                  << (int)pvtgeodectic->Error << std::endl
                  << (int)pvtgeodectic->num_bases << std::endl
                  << (int)pvtgeodectic->num_satellites << std::endl;
    }

    return true;
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
bool sbf::SBF::seek_block()
{
    block_start = nullptr;

    if (read_ptr == data_end)
    {
        buffer_use = 0;
        return false;
    }

    if (in_data(read_ptr))
    {
        while (read_ptr < data_end - 1)
        { // For all but the last data byte
            if (*read_ptr == sync_chars[0] && *(read_ptr + 1) == sync_chars[1])
            {
                block_start = read_ptr;
                return true;
            }
            read_ptr++;
        }
        if (*read_ptr == sync_chars[0])
        { // Sync broken bw iterations
            buffer[0] = sync_chars[0];
            buffer_use = 1;
            return false;
        }
        buffer_use = 0;
        return false;
    }
    else if (in_buffer(read_ptr))
    {
        while (read_ptr < buffer + buffer_use - 1)
        { // All but last buffer byte
            if (*read_ptr == sync_chars[0] && *(read_ptr + 1) == sync_chars[1])
            {
                block_start = read_ptr;
                return true;
            }
            read_ptr++;
        }
        // Sync broken bw buffer and data
        if (*read_ptr == sync_chars[0] &&
            *data_start == sync_chars[1])
        { // TODO: Is Data is guaranteed to have one byte
            block_start = read_ptr;
            return true;
        }
        // No block in buffer, check in data
        read_ptr = data_start;
        return seek_block();
    }
    assert(false); // Unreachable
}

/**
 * When inside a block, reads bytes. Ensures the read bytes are stored contiguously with the rest of the block.
 * requires `block_start` to be set.
 *
 * If no more bytes to read, copies partially read block to buffer, and returns nullptr
 *
 * @param size number of bytes to read
 * @return pointer to the location of the read bytes, nullptr if data over
 */
const uint8_t *sbf::SBF::read(size_t size)
{
    assert(block_start); // Make sure we are in a block

    // Ran out of data !
    if (read_ptr == data_end)
    { // NO MORE DATA
        if (in_data(block_start))
        { // Block in data, move to buffer
            auto block_size = data_end - block_start;
            if (block_size > buffer_size)
            {                   // Buffer overflow: Can't move block to buffer
                buffer_use = 0; // TODO: Warn?
            }
            else
            {
                // Store the current block in buffer, parse on next call
                std::memcpy(buffer, block_start, block_size);
                buffer_use = block_size;
            }
        }
        return nullptr;
    }

    auto ret = read_ptr;

    if (in_data(block_start))
    {
        if (read_ptr + size >= data_end)
        { // Not enough data
            auto remaining = read_ptr + size - (data_end);
            read_ptr = data_end;

            // END OF DATA
            return read(remaining);
        }
        read_ptr += size;
        return ret;
    }

    if (in_buffer(block_start))
    { // Block is stored in buffer

        if (in_buffer(read_ptr))
        { // There is more data in the buffer
            auto data_in_buffer = buffer + buffer_use - read_ptr;
            if (size < data_in_buffer)
            { // Need only buffer data
                read_ptr += size;
                return ret;
            }
            else
            {
                read_ptr = data_start;
                return read(size - data_in_buffer) ? ret : nullptr; // Get remaining data from data
            }
        }

        if (in_data(read_ptr))
        { // reading from data
            if (buffer_use + size > buffer_size)
            { // Buffer Overflow
                // TODO: Do we move read pointer ahead?
                read_ptr = std::min(data_start + size, data_end);
                return nullptr;
            }

            if (read_ptr + size > data_end)
            { // Not enough data
                auto avail_data = data_end - read_ptr;
                std::memcpy(buffer + buffer_use, read_ptr, avail_data);
                buffer_use += avail_data;
                read_ptr += avail_data;

                // END OF DATA
                return read(size - avail_data);
            }
            std::memcpy(buffer + buffer_use, read_ptr, size);
            buffer_use += size;
            read_ptr += size;
            return ret;
        }
        assert(false); // Unreachable:
    }
    assert(false); // Unreachable:
}

/**
 * Moves the read pointer backwards.
 * Used when a block turns out to be invalid.
 * @param rewind_len
 */
// TODO: Check
void sbf::SBF::unread(size_t rewind_len)
{
    if (in_buffer(block_start) && in_data(read_ptr) &&
        rewind_len > (read_ptr - data_start))
    {
        buffer_use -= read_ptr - data_start; // Forget the copied data
        read_ptr = buffer + buffer_use - (rewind_len - (read_ptr - data_start));
        // read_ptr = data_start;
    }
    else
        read_ptr -= rewind_len;

    // assert( read_ptr != block start) // We dont want to read the same block again and again
}

/**
 * Performs CRC check according to the SBF specification
 *
 * @param bytes : Start of bytes to be CRC checked
 * @param length  : Number of bytes to be CRC checked
 * @param crc : The correct CRC value
 * @return : Pass/Fail
 */
// TODO: Implement
/*static*/ bool sbf::SBF::check_crc(const uint8_t *bytes, size_t length, uint16_t crc)
{
    return true;
}
