cmake_minimum_required(VERSION 3.5)

project(Freewheeling LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GDAL REQUIRED)

include_directories(Freewheeling ${GDAL_INCLUDE_DIRS})
add_executable(Freewheeling main.cpp DataStructures.h PointMap.h Constants.h PointMap.cpp)

target_link_libraries(Freewheeling ${GDAL_LIBRARIES})

install(TARGETS Freewheeling
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})
