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
    size_t bytes_read = 0;
    while (true) {
        bytes_read = file.readsome(buffer, sizeof(buffer));
        // std::cout << "Read " << bytes_read << " bytes" << std::endl;
        if (!bytes_read)
            break;
        sbf.parse(reinterpret_cast<const uint8_t *>(buffer), bytes_read);
    }

    file.close();
}