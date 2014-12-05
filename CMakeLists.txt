project(mcontrol)
cmake_minimum_required(VERSION 2.8)
include(FindPkgConfig)

pkg_check_modules(PKGCONFIG REQUIRED libconfig++ tclap)
string(REPLACE ";" " " PKGCONFIG_CFLAGS "${PKGCONFIG_CFLAGS}")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")

set(SOURCES
   src/interface.cpp
   src/simulated.cpp
   src/controller.cpp
   src/angles.cpp
   src/main.cpp
)

option(HARDWARE "Build with support for real hardware instead of the simulator")
set(HARDWARE_CXXFLAGS "" CACHE STRING "c++ preprocessor flags for hardware support")
set(HARDWARE_LDFLAGS "-lwiringPi" CACHE STRING "linker flags for hardware support")
set(EFFECTIVE_LDFLAGS "")

if(HARDWARE)
   list(APPEND SOURCES src/hardware.cpp)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${HARDWARE_CXXFLAGS}")
   list(APPEND EFFECTIVE_LDFLAGS ${HARDWARE_LDFLAGS})
   add_definitions(-DHARDWARE -DCONFIG_FILE_PATH=\"/etc\")
else()
   add_definitions(-DCONFIG_FILE_PATH=\".\")
endif()

add_executable(mcontrol ${SOURCES})
target_link_libraries(mcontrol ${PKGCONFIG_LDFLAGS} ${EFFECTIVE_LDFLAGS})
