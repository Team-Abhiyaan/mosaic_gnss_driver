#include "mosaic_gnss_driver/sbf/sbf.h"
#include "mosaic_gnss_driver/sbf/sbf_conversions.h"

#include <iostream>

sbf::SBF::SBF(std::ifstream &in) : data(in) {
    // if (!in) error;
}

/**
 * Seeks until sync array is found, i.e. 0x24, 0x40
 *
 * @return success
 */
bool sbf::SBF::seek_block() {
    char c;
    while (data >> c) {
        if (c == '$') {
            if (data >> c) {
                if (c == '@')
                    return true;
            } else {
                return false;
            }
        }
    }
    return false;
}

bool sbf::SBF::next_block() {
    auto read_data = reinterpret_cast<uint8_t *>(buffer);
    if (!seek_block()) return false;
    if (!data.read(buffer, 6)) return false;

    auto crc = sbf::u2(read_data);

    auto raw_id = sbf::u2(read_data + 2);
    auto id = raw_id & 0b0001111111111111u;
    auto rev_num = (raw_id & 0b1110000000000000u) >> 13u;

    auto length = sbf::u2(read_data + 4);

    if (length % 4 != 0) {
        // std::cout << "Invalid block length" << std::endl;
        return next_block();
    }
    std::cout << id << "\t" << rev_num << "\t" << length; // << "\t" << data.tellg();

    length -= 8;


    if (length > buffer_size) {
        std::cout << "\t Buffer overflow" << std::endl;
        // data.seekg(length);
        return true;
    }
    if (!data.read(buffer, length)) return false;

    // Time stamp
    std::cout << "\t" << static_cast<int>( sbf::u4(read_data) / 1e3) << "\t"
              << static_cast<int>( sbf::u4(read_data + 4));

    std::cout << std::endl;
    return true;
}
