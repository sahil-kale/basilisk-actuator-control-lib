cmake_minimum_required(VERSION 3.14)

set(This tests)

set(BINARY ${CMAKE_PROJECT_NAME})

project(${This} C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD 99)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_GMOCK ON)

#enable Werror, Wextra, Wall, pedantic, and pedantic-errors
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -Wextra -Wall -Og -g -pedantic -pedantic-errors")

# enable testing
enable_testing()

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs/googletest ${CMAKE_CURRENT_BINARY_DIR}/googletest)


# Glob recurse the sources inside control_loop/
file(GLOB_RECURSE CONTROL_LOOP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/control_loop/*.cpp)

# Glob recurse the sources inside utils/
file(GLOB_RECURSE UTILS_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/util/*.cpp)

# Add a compiler definition -DUNITTEST to the compiler
add_definitions(-DUNIT_TEST)

# Glob recurse the headers inside test/
file(GLOB_RECURSE TEST_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/test/*.hpp)
# Glob recurse the sources inside test/
file(GLOB_RECURSE TEST_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp)

# Include the headers in the above paths and also do so recursively as there is a hierarchy
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hal)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hwbridge/3phase)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hwbridge/h_bridge)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/control_loop)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/control_loop/brushed)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/control_loop/bldc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/control_loop/stepper)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/util)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/util/pid)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/util/math)

# Mocks
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/test/mocks)

# Add an executable with the above sources
add_executable(${This} ${CONTROL_LOOP_SOURCES} ${UTILS_SOURCES} ${TEST_SOURCES})

# Link the executable with the GoogleTest libraries
target_link_libraries(${This} PUBLIC gtest gtest_main gmock)
