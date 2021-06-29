

#ifndef NMEAPARSER_H_
#define NMEAPARSER_H_


#include <mosaic_gnss_driver/parsers/nmeaparse/Event.h>
#include <string>
#include <functional>
#include <unordered_map>
#include <vector>
#include <cstdint>
#include <exception>
#include <cstdint>
#include <sstream>
#include <cinttypes>
#include <mosaic_gnss_driver/data_buffers.h>



//read class definition for info
#define NMEA_PARSER_MAX_BUFFER_SIZE 2000

namespace nmea {

    double parseDouble(std::string s);

    int64_t parseInt(std::string s, int radix = 10);

    class NMEAParser;

    class NumberConversionError : public std::exception {
    public:
        std::string message;

        NumberConversionError(std::string msg)
                : message(msg) {};

        virtual ~NumberConversionError() {};

        std::string what() {
            return message;
        }
    };


    class NMEASentence {
        friend NMEAParser;
    private:
        bool isvalid;
    public:
        std::string text;            //whole plaintext of the received command
        std::string name;            //name of the command
        std::vector<std::string> parameters;    //list of parameters from the command
        std::string checksum;
        bool checksumIsCalculated;
        uint8_t parsedChecksum;
        uint8_t calculatedChecksum;

        enum MessageID {        // These ID's are according to NMEA standard.
            Unknown = -1,
            GGA = 0,
            GLL = 1,
            GSA = 2,
            GSV = 3,
            RMC = 4,
            VTG = 5,        // notice missing 6,7
            ZDA = 8
        };
    public:
        NMEASentence();

        virtual ~NMEASentence();

        bool checksumOK() const;

        bool valid() const;

    };

    class NMEAParseError : public std::exception {
    public:
        std::string message;
        NMEASentence nmea;

        NMEAParseError(std::string msg);

        NMEAParseError(std::string msg, NMEASentence n);

        virtual ~NMEAParseError();

        std::string what();
    };


    class NMEAParser {
    private:
        std::unordered_map<std::string, std::function<void(NMEASentence)>> eventTable;
        std::string buffer;
        bool fillingbuffer;
        uint32_t maxbuffersize;        //limit the max size if no newline ever comes... Prevents huge buffer string internally

        void parseText(NMEASentence &nmea,
                       std::string s);        //fills the given NMEA sentence with the results of parsing the string.

        void onInfo(NMEASentence &n, std::string s);

        void onWarning(NMEASentence &n, std::string s);

        void onError(NMEASentence &n, std::string s);

        mosaic_gnss_driver::DataBuffers &data_buf;

    public:
        std::string nmea_buffer;  //for storing nmea sentence for publishing in ros

        explicit NMEAParser(mosaic_gnss_driver::DataBuffers &buffers);


        void parse(const uint8_t *data, size_t size);

        virtual ~NMEAParser();

        bool log;

        Event<void(
                const NMEASentence &)> onSentence;                // called every time parser receives any NMEA sentence
        void setSentenceHandler(std::string cmdKey, std::function<void(
                const NMEASentence &)> handler);    //one handler called for any named sentence where name is the "cmdKey"
        std::string
        getRegisteredSentenceHandlersCSV();                          // show a list of message names that currently have handlers.

        // Byte streaming functions
        void readByte(uint8_t b);

        void readBuffer(const uint8_t *b, uint32_t size);

        void readLine(std::string line);

        // This function expects the data to be a single line with an actual sentence in it, else it throws an error.
        void readSentence(std::string cmd);                // called when parser receives a sentence from the byte stream. Can also be called by user to inject sentences.

        static uint8_t calculateChecksum(std::string);        // returns checksum of string -- XOR
    };

}

#endif /* NMEAPARSER_H_ */
