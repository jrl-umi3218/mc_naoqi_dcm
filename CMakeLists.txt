cmake_minimum_required(VERSION 3.1)
project(mc_naoqi_dcm)
set(ROBOT_NAME "pepper" CACHE STRING "Build with this robot (pepper|nao)")

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
