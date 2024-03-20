cmake_minimum_required(VERSION 2.8)
project(mc_naoqi_dcm)
set(ROBOT_NAME "pepper" CACHE STRING "Build with this robot (pepper|nao)")

# NOTE: We cannot use C++11 here, nor C++0x or gnu++11 since we need to keep on supporting
# older version of the ctc toolchain. In particular with boost 1.55 this would fail.

find_package(qibuild)

message(STATUS "Building with ${ROBOT_NAME} robot")
if("${ROBOT_NAME}" STREQUAL "pepper")
  add_definitions(-DPEPPER)
elseif("${ROBOT_NAME}" STREQUAL "nao")
  add_definitions(-DNAO)
else()
  message( FATAL_ERROR "Only PEPPER and NAO robots are supported" )
endif()

include_directories(include)
include_directories("${CMAKE_CURRENT_BINARY_DIR}/include")
add_subdirectory(src)
