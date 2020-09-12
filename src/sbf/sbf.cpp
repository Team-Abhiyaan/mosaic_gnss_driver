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
    data_end = data_start + size;

    // Read from buffer if it has data
    read_ptr = buffer_use ? buffer : data_start;

    // Parse Blocks until data exhausted
    while (seek_block() && parse_block());
}

/**
 * Parses the block starting at `block_start`
 *
 * @return success
 */
bool sbf::SBF::parse_block() {
    const uint8_t *ret;

    // SYNC Chars
    ret = read(8);
    if (!ret) return false;

    auto header = reinterpret_cast<const sbf::Header *>(ret);

    assert(header->sync_chars[0] == sync_chars[0] && header->sync_chars[1] == sync_chars[1]);

    const auto[id, rev_num] = sbf::parse_id(header->ID);
    auto length = header->length;

    // Check Length
    if (length % 4 != 0 || length <= 8 || length > 123) {
        // std::cout << "Invalid block length" << std::endl;
        unread(sizeof(Header) - 2);
        return true;
    }

    // Block Length is
    const auto block_data_length = length - 8;
    ret = read(block_data_length);
    if (!ret) return false;

    if (!check_crc(block_start + 4, length - 4, header->CRC)) {
        unread(length - 2);
        return true;
    }

    std::cout << id << "\t" << (int) rev_num << "\t" << length << "\t" << header->CRC;



    // Call the parser
    // TODO: Implement
    // auto parser = parsers[id]
    // parser(rev_num, pares_ptr, parse_ptr_end);


    std::cout << std::endl;

    if (id == 4007) {
        std::cout << "Found PVTGeodictic" << std::endl;
        auto pvtgeodectic = reinterpret_cast<const sbf::PVTGeodetic *>(ret);
        std::cout << pvtgeodectic->Latitude * 180 / 3.14159 << std::endl
                  << pvtgeodectic->Longitude * 180 / 3.14159 << std::endl <<
                  (int) pvtgeodectic->Mode << std::endl <<
                  (int) pvtgeodectic->Error << std::endl
                  << (int) pvtgeodectic->num_bases << std::endl <<
                  (int) pvtgeodectic->num_satellites << std::endl;

        std::cout << length << " " << sizeof(PVTGeodetic) << std::endl;
    }
    return true;
}

/**
 * Seeks until sync str of block found, i.e. [0x24, 0x40]
 *
 * Sets `block_start` and `read_ptr` to the location where block is found.
 * If reaches end of data without finding a block, resets the internal buffer
 * If data ends with the first sync char, copies that to the internal buffer.
 *
 * @return true on block found, false on end of data
 */
bool sbf::SBF::seek_block() {
    block_start = nullptr;

    if (read_ptr == data_end) {
        buffer_use = 0;
        return false;
    }

    if (in_data(read_ptr)) {
        while (read_ptr < data_end - 1) { // For all but the last data byte
            if (*read_ptr == sync_chars[0] && *(read_ptr + 1) == sync_chars[1]) {
                block_start = read_ptr;
                return true;
            }
            read_ptr++;
        }
        if (*read_ptr == sync_chars[0]) { // Sync broken bw iterations
            buffer[0] = sync_chars[0];
            buffer_use = 1;
            return false;
        }
        buffer_use = 0;
        return false;
    } else if (in_buffer(read_ptr)) {
        while (read_ptr < buffer + buffer_use - 1) { // All but last buffer byte
            if (*read_ptr == sync_chars[0] && *(read_ptr + 1) == sync_chars[1]) {
                block_start = read_ptr;
                return true;
            }
            read_ptr++;
        }
        if (*read_ptr == sync_chars[0]) { // Sync broken bw buffer and data
            if (*data_start == sync_chars[1]) { // Data is guaranteed to have one byte
                block_start = read_ptr;
                return true;
            }
        }
        // No block in buffer, check in data
        read_ptr = data_start;
        return seek_block();
    }

    // This should never be reached
    assert(false);
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

/**
 * When inside a block, reads bytes. Ensures the read bytes are stored contiguously with the rest of the block.
 * requires `block_start` to be set.
 *
 * Responsible for dealing with incomplete blocks to buffer when data over.
 *
 * @param size number of bytes to read
 * @return pointer to the location of the read bytes.
 */
const uint8_t *sbf::SBF::read(size_t size) {
    assert(block_start); // Make sure we are in a block

    // Ran out of data !
    if (read_ptr == data_end) { // NO MORE DATA
        if (in_data(block_start)) { // Block in data, move to buffer
            auto block_size = data_end - block_start;
            if (block_size > buffer_size) { // Buffer overflow: Can't move block to buffer
                buffer_use = 0; // TODO: Warn?
            } else {
                // Store the current block in buffer, parse on next call
                std::memcpy(buffer, block_start, block_size);
                buffer_use = block_size;
            }
        }
        return nullptr;
    }

    auto ret = read_ptr;

    if (in_data(block_start)) {
        if (read_ptr + size >= data_end) { // Not enough data
            auto remaining = read_ptr + size - (data_end);
            read_ptr = data_end;

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
                return read(size - data_in_buffer) ? ret : nullptr; // Get remaining data from data
            }
        }

        if (in_data(read_ptr)) { // reading from data
            if (buffer_use + size > buffer_size) { // Buffer Overflow
                // TODO: Do we move read pointer ahead?
                read_ptr = std::min(data_start + size, data_end);
                return nullptr;
            }

            if (read_ptr + size > data_end) { // Not enough data
                auto avail_data = data_end - read_ptr;
                std::memcpy(buffer + buffer_use, read_ptr, avail_data);
                buffer_use += avail_data;
                read_ptr += avail_data;
                // We know the following will return a nullptr
                // Supposed to run the (read_ptr == data_end) branch
                return read(size - avail_data);
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

void sbf::SBF::unread(size_t rewind_len) {
    if (in_buffer(block_start) && in_data(read_ptr) &&
        rewind_len > (read_ptr - data_start)) {
        buffer_use -= read_ptr - data_start; // Forget the copied data
        read_ptr = buffer + buffer_use - (rewind_len - (read_ptr - data_start));
        // read_ptr = data_start;
    } else
        read_ptr -= rewind_len;
}

bool sbf::SBF::check_crc(const uint8_t *bytes, size_t length, uint16_t crc) {
    return true;
}
