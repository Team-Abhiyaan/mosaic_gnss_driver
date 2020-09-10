# SBF Parser "Septentrio Binary Format"

Takes in data via a block of memory (pointer and length).
`void parse(uint8_t * data, size_t length);`
The memory is not modified and must not be modified during the duration of the call by other threads.
It parses any complete SBF blocks and copies the rest of the data to an internal buffer, to free up use of `data`.


Use commit: a95f7bc75d1a3b003ff41fb4e54e3c917d56231c
Save the output to a file
then use `cat file | sort | uniq -c | sort -hr` to see which blocks are most common

for the first dump received:
```
     10 4105 - DynDNSStatus
      7 4201 - LBandTrackerStatus
      6 5906 - PosCovGeodetic
      4 5921 - EndOfPvt
      4 4001 - DOP [PVT Dilution of Precision ]
      3 4023 - Galileo INAV Page
      3 4017 - GPS Raw CA
      2 5922 - End of Meas
      2 4028 - Relative Position and Velocity wrt bases
      1 4238 - P2PP Status
      1 4120 - BSDIon
      1 4026 - GLO Raw CA
      1 4007 - PVT Geodetic
```
