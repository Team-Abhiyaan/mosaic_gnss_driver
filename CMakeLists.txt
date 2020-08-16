cmake_minimum_required(VERSION 3.5)

project(mosaic_gnss_driver)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)

include_directories(
    include
)

enable_testing()

add_subdirectory(lib/googletest)
add_subdirectory(tests)

add_library(countOnes src/countOnes.cpp)

add_executable(demo src/main.cpp)
target_link_libraries(demo countOnes)