#include "mosaic_gnss_driver/sbf/sbf.h"
#include <fstream>
#include <iostream>

int main(int argc, char **argv) {
    std::cout << "Starting sbf test" << std::endl;

    std::string filename = "dump_master.sbf";
    if (argc == 2) {
        filename = argv[2];
    }
    std::cout << "Reading " << filename << std::endl;
    std::ifstream file(filename, std::ios::binary);


    sbf::SBF sbf(file);

    size_t num_sync_chars = 0;
    while (sbf.next_block())
        num_sync_chars++;
    std::cout << "Found " << num_sync_chars << " blocks." << std::endl;

}