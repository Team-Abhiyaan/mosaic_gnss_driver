#include "mosaic_gnss_driver/sbf/sbf.h"
#include "mosaic_gnss_driver/sbf/helpers.h"

#include <iostream>

sbf::SBF::SBF(std::ifstream &in) : input(in) {
    // if (!in) error;
}

/**
 * Seeks until sync str of block found, i.e. [0x24, 0x40]
 *
 * @return true on block found, false on IO error
 */
bool sbf::SBF::seek_block() {
    while (input.read(buffer, 1)) {
        if (*buffer == '$') { // 0x24
            if (input.read(buffer, 1)) {
                if (*buffer == '@') // 0x40
                    return true;
            } else {
                return false;
            }
        }
    }
    return false;
}

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
bool sbf::SBF::parse_next() {
    // Find the next block
    if (!seek_block()) return false;

    // Read CRC
    if (!input.read(buffer, 2)) return false;
    const auto crc = sbf::u2(reinterpret_cast<uint8_t *>(buffer));

    // Now Parse the rest of the block header

    // This ptr walks through the buffer while we parse values
    auto parse_ptr = reinterpret_cast<uint8_t *>(buffer);


    // Read ID and Length
    if (!input.read(buffer, 4)) return false;

    // Get ID and revision number
    const auto raw_id = sbf::u2(parse_ptr);
    parse_ptr += 2; // TODO: Move this to the sbf::u2()

    const auto[id, rev_num] = sbf::parse_id(raw_id);

    // Get Length
    auto length = sbf::u2(parse_ptr);
    parse_ptr += 2;

    // Check Length
    if (length % 4 != 0 || length <= 8) {
        // std::cout << "Invalid block length" << std::endl;
        return parse_next();
    }
    if (length > buffer_size) {
        // std::cout << "\t Buffer overflow" << std::endl;
        // TODO: How to handle.
        // data.seekg(data_length);
        return true;
    }


    // Read the rest of the block
    // Also, we dont want to overwrite ID and length so we can perform CRC checks.

    // Block Length is
    const auto data_length = length - 8;
    if (!input.read(buffer + 4, data_length)) return false;
    // parse_ptr points to current position in while parsing data
    auto parse_ptr_end = parse_ptr + data_length; // End of block data

    std::cout << id << "\t" << (int) rev_num << "\t" << length;


    // Call the parser
    /* TODO: Implement
     * auto parser = parsers[id]
     * parser(rev_num, pares_ptr, parse_ptr_end);
     */


    std::cout << std::endl;
    return true;
}
