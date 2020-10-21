## Define macros at compile time for relevant connection tests
if(TEST_CONN_PCAP)
    add_compile_options(-D RUN_PCAP_TEST)
    message(STATUS "[ MOSAIC-GNSS ] Including tests for PCAP connection")
endif()

if(TEST_CONN_TCP)
    add_compile_options(-D RUN_TCP_TEST)
    message(STATUS "[ MOSAIC-GNSS ] Including tests for TCP connection")
endif()

if(TEST_CONN_UDP)
    add_compile_options(-D RUN_UDP_TEST)
    message(STATUS "[ MOSAIC-GNSS ] Including tests for UDP connection")
endif()

if(TEST_CONN_SERIAL)
    add_compile_options(-D RUN_SERIAL_TEST)
    message(STATUS "[ MOSAIC-GNSS ] Including tests for SERIAL connection")
endif()
