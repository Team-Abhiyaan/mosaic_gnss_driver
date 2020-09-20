#include <mosaic_gnss_driver/parsers/sbf/sbf.h>
#include <fstream>
#include <iostream>

/**
 * Reads a binary file and passes it to an SBF parser.
 * Filename should be parsed as a commandline argument.
 */

int main(int argc, char **argv)
{
    std::cout << "Starting sbf test" << std::endl;

    std::string filename = "dump_master.sbf";
    if (argc == 2)
        filename = argv[1];

    std::cout << "Reading " << filename << std::endl;
    std::ifstream file(filename, std::ios::binary);

    sbf::SBF sbf;

    char buffer[150];
    size_t bytes_read;
    while ((bytes_read = file.readsome(buffer, sizeof(buffer))))
        sbf.parse(reinterpret_cast<const uint8_t *>(buffer), bytes_read);

    file.close();
}