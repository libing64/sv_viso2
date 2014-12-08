cmake_minimum_required(VERSION 2.8.3)
project(libviso2)

find_package(catkin REQUIRED)

add_definitions(-msse3 -std=c++11 -Wall)

add_library(viso2
    libviso2/src/filter.cpp
    libviso2/src/matcher.cpp
    libviso2/src/matrix.cpp
    libviso2/src/reconstruction.cpp
    libviso2/src/triangle.cpp
    libviso2/src/viso.cpp
    libviso2/src/viso_mono.cpp
    libviso2/src/viso_stereo.cpp
    )

include_directories(libviso2/src ${catkin_INCLUDE_DIRS})

catkin_package(
    INCLUDE_DIRS libviso2/src
    LIBRARIES viso2
    )
