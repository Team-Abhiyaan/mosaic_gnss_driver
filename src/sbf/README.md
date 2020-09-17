# SBF Parser "Septentrio Binary Format"

Takes in data via a block of memory (pointer and length).
`void parse(uint8_t * data, size_t length);`
The memory is not modified and must not be modified during the duration of the call by other threads.
It parses any complete SBF blocks and copies the rest of the data to an internal buffer, to free up use of `data`.
