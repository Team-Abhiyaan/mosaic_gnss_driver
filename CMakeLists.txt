cmake_minimum_required(VERSION 3.0.2)
project(mosaic_gnss_driver)

add_compile_options(-std=c++14)

## Build only relevant tests
option(TEST_CONN_PCAP    "Run tests for PCAP connection"    ON)
option(TEST_CONN_TCP     "Run tests for TCP connection"     OFF)
option(TEST_CONN_UDP     "Run tests for UCP connection"     OFF)
option(TEST_CONN_SERIAL  "Run tests for SERIAL connection"  OFF)

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

# TODO: roscpp and roslib are not dependencies for the core_lib
# Maybe one CMakeLists.txt per target?
set(DEPS
    roscpp
    roslib
    std_msgs
    sensor_msgs
    nmea_msgs
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Boost REQUIRED COMPONENTS
    system
)

find_package(catkin REQUIRED COMPONENTS
    ${DEPS}
)

find_library(libpcap_LIBRARIES pcap)
if ("${libpcap_LIBRARIES}" STREQUAL "pcap-NOTFOUND")
    set(libpcap_FOUND FALSE)
else ()
    set(libpcap_FOUND TRUE)
endif ()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${DEPS}
    DEPENDS Boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

## Core Library
add_library(${PROJECT_NAME} STATIC
    src/core/util/serial.cpp
    src/core/connections/tcp.cpp
    src/core/connections/udp.cpp
    src/core/connections/pcap.cpp
    src/core/connections/serial.cpp
    src/core/parsers/sbf/sbf.cpp
    src/core/parsers/sbf/block_parsers/geodetic.cpp
    src/core/parsers/sbf/block_parsers/cartesian.cpp
    src/core/parsers/nmea/NMEAParser.cpp
    src/core/parsers/nmea/GPSService.cpp
    src/core/parsers/nmea/GPSFix.cpp
)

target_compile_definitions(${PROJECT_NAME} PRIVATE MOSAIC_GNSS_CORE_ONLY)
#target_compile_definitions(${PROJECT_NAME} PRIVATE MOSAIC_SBF_PRINT_ID)

# Driver
add_executable(${PROJECT_NAME}_node src/main.cpp)
target_compile_definitions(${PROJECT_NAME}_node PRIVATE MOSAIC_GNSS_FAKE_SLEEP_TIME=0.1)

#add_executable(nmea_parser_test src/core/parsers/nmea/nmea_parser_test.cpp src/core/parsers/nmea/NMEAParser.cpp src/core/parsers/nmea/GPSService.cpp src/core/parsers/nmea/GPSFix.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${libpcap_LIBRARIES}
    ${Boost_LIBRARIES}
)

#target_link_libraries(nmea_parser_test
#        ${PROJECT_NAME}
#        ${catkin_LIBRARIES}
#        ${libpcap_LIBRARIES}
#        ${Boost_LIBRARIES}
#        )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# Mark executables for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html

# install(TARGETS ${PROJECT_NAME}_node
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#     )
# install(TARGETS ${PROJECT_NAME}_noross
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#     )

# Mark libraries for installation
# See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    macro(package_add_test TARGETNAME)
        catkin_add_gtest(${TARGETNAME} ${ARGN})
        target_link_libraries(${TARGETNAME}
            ${PROJECT_NAME}
            ${catkin_LIBRARIES}
            ${libpcap_LIBRARIES}
            ${Boost_LIBRARIES}
        )
    endmacro(package_add_test)

    package_add_test(conn_test test/test_conn.cpp)

endif ()
