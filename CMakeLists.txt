cmake_minimum_required(VERSION 3.6)

set(CMAKE_CONFIGURATION_TYPES Debug Release CACHE TYPE INTERNAL FORCE)

project(kinect_fusion)

# Options
set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src CACHE PATH "Path to source folder")
set(HELPERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/helpers CACHE PATH "Path to helpers folder")
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to lib folder")
set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/eigen/ CACHE PATH "Path to Eigen source folder")

# Use pre-compiled libraries on Windows
if(WIN32)
    set(FreeImage_INCLUDE_DIR ${LIBRARY_DIR}/FreeImage/x64/ CACHE PATH "Path to FreeImage header file")
    set(FreeImage_LIBRARY_DIR ${LIBRARY_DIR}/FreeImage/x64/ CACHE PATH "Path to FreeImage .lib/.dll folder")
endif(WIN32)


set(HEADERS 
    ${HELPERS_DIR}/Eigen.h
    ${HELPERS_DIR}/FreeImageHelper.h
    ${HELPERS_DIR}/VirtualSensor.h)

set(SOURCES
    ${HELPERS_DIR}/FreeImageHelper.cpp
    ${SOURCE_DIR}/main.cpp)

link_directories(${FreeImage_LIBRARY_DIR})
add_executable(kinect_fusion ${SOURCES} ${HEADERS})
target_include_directories(kinect_fusion PUBLIC ${Eigen3_INCLUDE_DIR} ${FreeImage_INCLUDE_DIR})
target_link_libraries(kinect_fusion general freeimage)

# On Windows copy dll to executable folder
if(WIN32)
    add_custom_command(TARGET kinect_fusion POST_BUILD        # Adds a post-build event to MyTest
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${LIBRARY_DIR}/FreeImage/x64/FreeImage.dll"      # <--this is in-file
            $<TARGET_FILE_DIR:kinect_fusion>)                 # <--this is out-file path
endif(WIN32)