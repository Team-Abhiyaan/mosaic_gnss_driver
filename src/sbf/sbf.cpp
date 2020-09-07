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
