cmake_minimum_required(VERSION 3.5)

project(Freewheeling LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(GDAL REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Marble REQUIRED)

include_directories(Freewheeling ${GDAL_INCLUDE_DIRS})
add_executable(Freewheeling main.cpp DataStructures.h PointMap.h Constants.h PointMap.cpp MapWindow.h MapWindow.cpp MarbleWorker.h MarbleWorker.cpp RouteGraph.h RouteGraph.cpp)

#set(CMAKE_CXX_FLAGS "-no-pie")
set(CMAKE_BUILD_TYPE "Release")

target_link_libraries(Freewheeling ${GDAL_LIBRARIES} Marble Qt5::Widgets Qt5::Core)

install(TARGETS Freewheeling
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

set_target_properties(Freewheeling PROPERTIES AUTOMOC TRUE)
