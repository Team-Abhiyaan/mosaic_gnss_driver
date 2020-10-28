#include <mosaic_gnss_driver/parsers/sbf/sbf.h>
#include <mosaic_gnss_driver/parsers/sbf/helpers.h>

#include <iostream>
#include <cassert>
#include <cstring>
#include <tuple>

sbf::SBF::SBF(mosaic_gnss_driver::DataBuffers &buffers) : data_buf{buffers}
{}

void sbf::SBF::parse(const uint8_t *const data, size_t size)
{
    if (data == nullptr || size == 0)
        return;

    data_start = data;
    data_end = data_start + size;

    // Read from buffer if it has data
    read_ptr = buffer_use ? buffer : data_start;

    // Parse Blocks until no more data
    while (seek_block() && parse_block());
}

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

    // std::cout << id << "\t" << (int)rev_num << "\t" << length << "\t" << header->CRC << std::endl;

    // Call the parser
    // TODO: Implement
    // auto parser = parsers[id]
    // parser(rev_num, pares_ptr, parse_ptr_end);

#ifdef MOSAIC_SBF_PRINT_ID
    std::cout << id << std::endl;
#endif

    if (id == 4007)
    {
        auto pvtgeodectic = reinterpret_cast<const sbf::PVTGeodetic *>(ret);

        static auto rad2deg = [](auto val) {return val * 180 / 3.141592653;};
        auto ptr = data_buf.nav_sat_fix.get_new_ptr();
        ptr->latitude = rad2deg(pvtgeodectic->Latitude);
        ptr->longitude = rad2deg(pvtgeodectic->Longitude);
        ptr->altitude = pvtgeodectic->Height;
        data_buf.nav_sat_fix.set_ptr(std::move(ptr));

        // field.ptr->status =
        // field.ptr->header.stamp = // TODO: Convert sbf stamp to ros stamp.
    }

    return true;
}

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
    } else if (in_buffer(read_ptr))
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
            } else
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
            } else
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

void sbf::SBF::unread(size_t rewind_len)
{
    if (in_buffer(block_start) && in_data(read_ptr) &&
        rewind_len > (read_ptr - data_start))
    {
        buffer_use -= read_ptr - data_start; // Forget the copied data
        read_ptr = buffer + buffer_use - (rewind_len - (read_ptr - data_start));
        // read_ptr = data_start;
    } else
        read_ptr -= rewind_len;

    // assert( read_ptr != block start) // We dont want to read the same block again and again
}

// TODO: Implement
/*static*/ bool sbf::SBF::check_crc(const uint8_t *bytes, size_t length, uint16_t crc)
{
    return true;
}
