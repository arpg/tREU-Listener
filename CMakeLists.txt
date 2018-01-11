project(tREUListener)

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")

find_package(HAL REQUIRED)

include_directories( ${HAL_INCLUDE_DIRS} )
add_executable( tREUListener tREUListener.cpp ComDriver.h )
target_link_libraries(tREUListener
    ${HAL_LIBRARIES}
     )

