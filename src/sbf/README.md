# SBF Parser "Septentrio Binary Format"

Takes in data via a block of memory (pointer and length).
`void parse(uint8_t * data, size_t length);`
The memory is not modified and must not be modified during the duration of the call by other threads.
It parses any complete SBF blocks and copies the rest of the data to an internal buffer, to free up use of `data`.


# CRC Resources

https://web.archive.org/web/20181114105930/http://www.ross.net/crc/download/crc_v3.txt
http://srecord.sourceforge.net/crc16-ccitt.html#overview

http://www.nongnu.org/avr-libc/user-manual/group__util__crc.html

see crc.txt
