#include <mosaic_gnss_driver/parsers/sbf/helpers.h>

std::pair<uint16_t, uint8_t> sbf::parse_id(const uint16_t raw_id)
{
    return {raw_id & 0b0001111111111111u, (raw_id & 0b1110000000000000u) >> 13u};
}