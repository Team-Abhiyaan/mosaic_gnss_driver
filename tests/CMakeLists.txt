add_executable(count_test_1 test1.cpp)
target_link_libraries(count_test_1 countOnes gtest_main)

add_test(
    NAME test1
    COMMAND test1
)