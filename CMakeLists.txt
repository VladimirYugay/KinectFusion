cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(kinect_fusion)

# Options
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src CACHE PATH "Path to source folder")
set(HELPERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/helpers CACHE PATH "Path to helpers folder")
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to lib folder")
set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/eigen/ CACHE PATH "Path to Eigen source folder")



set(HEADERS 
    ${HELPERS_DIR}/Eigen.h
    ${HELPERS_DIR}/FreeImageHelper.h
    ${HELPERS_DIR}/VirtualSensor.h)

set(SOURCES
    ${HELPERS_DIR}/FreeImageHelper.cpp
    ${SOURCE_DIR}/main.cpp)

link_directories(${FreeImage_LIBRARY_DIR})
add_executable(kinect_fusion ${SOURCES} ${HEADERS})
target_include_directories(kinect_fusion PUBLIC ${Eigen3_INCLUDE_DIR})
target_link_libraries(kinect_fusion general freeimage)