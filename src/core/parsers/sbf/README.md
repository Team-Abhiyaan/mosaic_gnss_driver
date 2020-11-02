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

# Reg nav_sat_fix::status

# Navigation Satellite fix status for any Global Navigation Satellite System

# Whether to output an augmented fix is determined by both the fix
# type and the last time differential corrections were received.  A
# fix is valid when status >= STATUS_FIX.

int8 STATUS_NO_FIX =  -1        # unable to fix position
int8 STATUS_FIX =      0        # unaugmented fix
int8 STATUS_SBAS_FIX = 1        # with satellite-based augmentation
int8 STATUS_GBAS_FIX = 2        # with ground-based augmentation

int8 status

# Bits defining which Global Navigation Satellite System signals were
# used by the receiver.

uint16 SERVICE_GPS =     1
uint16 SERVICE_GLONASS = 2
uint16 SERVICE_COMPASS = 4      # includes BeiDou.
uint16 SERVICE_GALILEO = 8

uint16 service
