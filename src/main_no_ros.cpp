#include <ros/package.h>
#include <ros/ros.h>

#include <mosaic_gnss_driver/mosaic_gnss.h>

void connectViaPcap(const std::string &filename = "/test/data/capture_002.pcap") {
    mosaic_gnss_driver::MosaicGNSS gnss;

    gnss.connect(filename, mosaic_gnss_driver::MosaicGNSS::PCAP);

    while (gnss.isConnected() && gnss.processData() == mosaic_gnss_driver::MosaicGNSS::READ_SUCCESS);

    // gnss.bufferDump();
    gnss.disconnect();
}

void connectViaTcp(const std::string &device = "192.168.3.1:3001") {
    mosaic_gnss_driver::MosaicGNSS gnss;

    gnss.connect(device, mosaic_gnss_driver::MosaicGNSS::TCP);

    for (int i = 0; i < 50; i++) {
        bool success = gnss.isConnected() && gnss.processData() == mosaic_gnss_driver::MosaicGNSS::READ_SUCCESS;

        if (!success) {
            std::cout << "Error, non successful termination" << std::endl;
            std::cout << gnss.errorMsg() << std::endl;

            break;
        } else {
            std::cout << "Got stream: " << i + 1 << std::endl;
        }
    }

    gnss.bufferDump();
    gnss.disconnect();

}

int main(int argc, char **argv) {
    enum RUN_TYPE {
        NONE, PCAP, TCP, HELP
    } runType{NONE};

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
            } // Argument
        }
    }

    ros::Time::init();
    switch (runType) {
        case TCP:
            if (tcp_device.empty()) connectViaTcp();
            else connectViaTcp(tcp_device);
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
