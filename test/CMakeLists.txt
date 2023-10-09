cmake_minimum_required(VERSION 3.8)

set(This tests)

# set the test sources to everything in this folder that is cpp
file(GLOB SOURCES "*.cpp")
# add the main project sources in ../src to the test sources
file(GLOB MAIN_SOURCES "../src/*.cpp")

# include the headers from the main project, which is located at "../inc/"
include_directories(../inc)
include_directories((../test/))

add_test(
    NAME ${This}
    COMMAND ${This}
)

# add compiler definition for UNIT_TEST
add_definitions(-DUNIT_TEST)

add_executable(${This} ${SOURCES} ${MAIN_SOURCES})
target_link_libraries(${This} PUBLIC gtest_main gtest gmock)
