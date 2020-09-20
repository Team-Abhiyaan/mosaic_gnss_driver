#include <ros/package.h>
#include <ros/ros.h>
#include <mosaic_gnss_driver/mosaic_gnss.h>

#include <mosaic_gnss_driver/parsers/sbf/sbf.h>
#include <mosaic_gnss_driver/connections/pcap.h>

void connectViaPcap(const std::string &filename = "/test/data/capture_002.pcap") {
    mosaic_gnss_driver::GNSS<mosaic_gnss_driver::connections::PCAP, sbf::SBF> gnss{};
    if (!gnss.connect(filename)) return;
    while (gnss.tick());
}


int main(int argc, char **argv) {
    enum RUN_TYPE {
        NONE, PCAP, TCP, HELP
    } runType{NONE};

    std::vector<std::string> args;
    std::string pcap_file, tcp_device;

    std::string *arg = nullptr;
    if (argc > 1) {
        for (auto i = 1; i < argc; ++i) {
            auto length = strlen(argv[i]);

            if (!length) continue; // Empty arg

            if (length >= 2 && argv[i][0] == '-') { // Option
                for (auto j = 1; j < length; j++) {
                    switch (argv[i][j]) {
                        case 'p':
                            runType = PCAP;
                            arg = &pcap_file;
                            break;
                        case 't':
                            runType = TCP;
                            arg = &tcp_device;
                            break;
                        case 'h':
                            runType = HELP;
                            break;
                    }
                }
            } else if (arg) {
                *arg = argv[i];
                arg = nullptr;
            } else {
                args.emplace_back(argv[i])
            }
        }
    }

    ros::Time::init();
    switch (runType) {
        case TCP:
            // if (tcp_device.empty()) connectViaTcp();
            // else connectViaTcp(tcp_device);
            break;
        case PCAP:
            if (pcap_file.empty()) connectViaPcap();
            else connectViaPcap(pcap_file);
            break;
        case HELP:
        case NONE:
            std::cout << "Usage: " << argv[0] << "-[tph]" << std::endl
                      << "\t-h: Print this help text" << std::endl
                      << "\t-t [device]: connect with tcp." << std::endl
                      << "\t-p [filename]: read from pacap file" << std::endl;
            break;
    }

    return 0;
}
