cmake_minimum_required(VERSION 3.8)

set(This MAINS)
set(BINARY ${CMAKE_PROJECT_NAME})

project(${This} C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(BUILD_GMOCK ON)

# enable Werror
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror")

enable_testing()

# add the googletest subdirectory, located in the lib folder tree that is one level above this one
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libs/googletest ${CMAKE_CURRENT_BINARY_DIR}/googletest)

# set the HEADERS as everything in the inc folder, as well as test *.hpp files
file(GLOB HEADERS "inc/*.hpp")

# set the SOURCES as everything in the src folder
file(GLOB SOURCES "src/*.cpp")

# add_executable(${BINARY}_run ${SOURCES})
add_library(${This} STATIC ${SOURCES} ${HEADERS})

add_subdirectory(test)

# To find the tests, this is rquired
include_directories(inc)
include_directories(src)