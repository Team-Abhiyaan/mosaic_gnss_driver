#include "mosaic_gnss_driver/sbf/sbf.h"
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
    std::cout << "Starting sbf test" << std::endl;

    std::string filename = "dump_master.sbf";
    if (argc == 2) {
        filename = argv[1];
    }
    std::cout << "Reading " << filename << std::endl;
    std::ifstream file(filename, std::ios::binary);

    sbf::SBF sbf;

    char buffer[256];
    size_t bytes_read;
    while (true) {
        bytes_read = file.readsome(buffer, sizeof(buffer));
        // if (0 < bytes_read && bytes_read != sizeof(buffer))
        //     std::cout << "Read only " << bytes_read << " bytes" << std::endl;
        if (!bytes_read)
            break;
        // std::cout <<"______________________" << std::endl;
        sbf.parse(reinterpret_cast<const uint8_t *>(buffer), bytes_read);
    }

    file.close();
}