#include "mosaic_gnss_driver/sbf/sbf.h"
#include "mosaic_gnss_driver/sbf/helpers.h"

#include <iostream>
#include <cassert>
#include <cstring>

sbf::SBF::SBF() = default;

/*
 * List of errors:
 *  data end
 */

/* Things that cause seek_block
 *   Need block
 *   Invalid length
 *   buffer overflow
 */

/*
 * A block has the following format:
 * sync | CRC | ID | Length | Actual Data ....
 *
 * Sync is eaten up by seek_block
 * CRC is stored in a local variable
 *
 * Everything onwards is stored in the buffer and is required for the CRC check.
 *
 * Length of the block has to be a multiple of 4 and that check is applied
 * Based on the block id, a parser is chosen.
 *
 * The parser receives the revision number and two pointers, the start and end of the Actual Data.
 * The parser _must_ copy any data it requires, as buffer will be overwritten after the parser is called.
 *
 */

/**
 * Parses the given chunk of binary data
 *
 * @param data pointer to the data, none of the data is modified
 * @param size size of the passed data
 */
void sbf::SBF::parse(const uint8_t *const data, size_t size) {
    if (data == nullptr || size == 0) return;
    data_start = data;
    data_length = size;
    block_start = nullptr;


    if (buffer_use) {
        read_ptr = buffer;
    } else {
        read_ptr = data_start;
    }


    while (seek_block() && parse_block());
    // Only buffer and buffer_use should persist.
}

/**
 * TODO: convert IO errors to exceptions
 * @return false for IO error, true otherwise
 */
bool sbf::SBF::parse_block() {
    const uint8_t *ret;

    // SYNC Chars
    ret = read(2);
    if (!ret) return false;
    if (ret[0] != sync_chars[0] && ret[1] != sync_chars[1]) return true;

    // Read CRC
    ret = read(2);
    if (!ret) return false;
    const auto crc = sbf::u2(ret);

    // Read Header (ID and Length)
    ret = read(4);
    if (!ret) return false;

    // Get Header
    auto header = reinterpret_cast<const sbf::Header *>(ret);
    const auto[id, rev_num] = sbf::parse_id(header->ID);
    auto length = header->length;


    // Check Length
    if (length % 4 != 0 || length <= 8 || length > 123) {
        // std::cout << "Invalid block length" << std::endl;
        block_start = nullptr;
        read_ptr -= sizeof(Header);
        return true;
    }

    // Block Length is
    const auto block_data_length = length - 8;
    ret = read(block_data_length);
    if (!ret) return false;

    // TODO: check CRC

    std::cout << id << "\t" << (int) rev_num << "\t" << length << "\t" << crc;



    // Call the parser
    // TODO: Implement
    // auto parser = parsers[id]
    // parser(rev_num, pares_ptr, parse_ptr_end);


    std::cout << std::endl;

    /*if (id == 4007) {
        std::cout << "Found PVTGeodictic" << std::endl;
        auto pvtgeodectic = reinterpret_cast<const sbf::PVTGeodetic *>(ret);
        std::cout << pvtgeodectic->Latitude * 180 / 3.14159 << std::endl
                  << pvtgeodectic->Longitude * 180 / 3.14159 << std::endl <<
                  (int) pvtgeodectic->Mode << std::endl <<
                  (int) pvtgeodectic->Error << std::endl
                  << (int) pvtgeodectic->num_bases << std::endl <<
                  (int) pvtgeodectic->num_satellites << std::endl;

        std::cout << length << " " << sizeof(PVTGeodetic) << std::endl;
    }*/
    return true;
}

/**
 * Seeks until sync str of block found, i.e. [0x24, 0x40]
 *
 * @return true on block found, false on IO error
 */
bool sbf::SBF::seek_block() {
    block_start = nullptr;

    if (in_data(read_ptr)) {
        while (read_ptr < data_start + data_length - 1) {
            if (*read_ptr == sync_chars[0] && *(read_ptr + 1) == sync_chars[1]) {
                block_start = read_ptr;
                return true;
            }
            read_ptr++;
        }
        if (*read_ptr == sync_chars[0]) {
            block_start = read_ptr;
            read_ptr += 1; // Force save to buffer
            read(0); // Copy '$' to the buffer
            return false;
        }
        read_ptr += 1;
        return false;
    } else if (in_buffer(read_ptr)) {
        while (read_ptr < buffer + buffer_use - 1) {
            if (*read_ptr == sync_chars[0] && *(read_ptr + 1) == sync_chars[1]) {
                block_start = read_ptr;
                return true;
            }
            read_ptr++;
        }
        if (*read_ptr == sync_chars[0]) {
            if (*data_start == sync_chars[1]) { // Data is guaranteed to have one byte
                block_start = read_ptr;
                return true;
            }
        }
        // Check in data
        read_ptr = data_start;
        return seek_block();
    }

    buffer_use = 0;
    return false;
}


/*
 * We are in a block
 *
 * read_ptr at end of data:
 *   store current block in buffer
 *   return nullptr
 *
 * block in buffer:
 *   read_ptr in buffer:
 *     increment read_ptr
 *     If need more data:
 *        move read_ptr to data i
 *        call read again if more data required than in buffer
 *   read_ptr in data:
 *     copy from data to buffer
 *     increment buffer_use, read_ptr
 *
 *   block in data:
 *     increment read_ptr
 *     if not enough data:
 *        copy from block_start to the buffer
 *        set buffer_use
 *
 *
 * If not enough data: return nullptr.
 */
const uint8_t *sbf::SBF::read(size_t size) {
    assert(block_start); // Make sure we are in a block

    // Ran out of data !
    if (read_ptr == data_start + data_length) { // NO MORE DATA
        if (buffer <= block_start && block_start < buffer + buffer_size) { // Block already in buffer
            read_ptr = buffer;
        } else if (data_start <= block_start && block_start < data_start + data_length) { // Move block to buffer
            auto block_size = data_start + data_length - block_start;
            if (block_size > buffer_size) { // Buffer overflow: Can't move block to buffer
                buffer_use = 0;
                read_ptr = nullptr;
            } else {
                // Store the current block in buffer, parse on next call
                std::memcpy(buffer, block_start, block_size);
                buffer_use = block_size;
                read_ptr = buffer;
            }
        }

        // This iteration of parse is done, reset vars.
        block_start = nullptr;
        data_start = nullptr;
        data_length = 0;
        return nullptr;
    }

    auto ret = read_ptr;

    if (in_data(block_start)) {
        if (read_ptr + size >= data_start + data_length) {
            auto remaining = read_ptr + size - (data_start + data_length);
            read_ptr = data_start + data_length;

            // Will return nullptr
            return read(remaining);
        }
        read_ptr += size;
        return ret;
    }

    if (in_buffer(block_start)) { // Block is stored in buffer

        if (in_buffer(read_ptr)) { // There is more data in the buffer
            auto data_in_buffer = buffer + buffer_use - read_ptr;
            if (size < data_in_buffer) { // Need only buffer data
                read_ptr += size;
                return ret;
            } else {
                read_ptr = data_start;
                buffer_use = 0;
                return read(size - data_in_buffer) ? ret : nullptr; // Get remaining data from data
            }
        }

        if (in_data(read_ptr)) { // reading from data
            if (buffer_use + size > buffer_size) { // Buffer Overflow
                read_ptr = data_start + std::min(size, data_length);
                return nullptr;
            }

            if (read_ptr + size > data_start + data_length) { // Not enough data
                auto avail_data = data_start + data_length - read_ptr;
                std::memcpy(buffer + buffer_use, read_ptr, avail_data);
                buffer_use += avail_data;
                read_ptr += avail_data;
                // We know the following will return a nullptr
                // Supposed to run the (read_ptr == data_start + data_length) branch
                return read(size - avail_data); // Get remaining data from data
            }
            std::memcpy(buffer + buffer_use, read_ptr, size);
            buffer_use += size;
            read_ptr += size;
            return ret;
        }

        // Unreachable:
        assert(false);
    }

    // Unreachable:
    assert(false);
}
