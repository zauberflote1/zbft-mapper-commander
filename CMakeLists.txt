cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(zbft-mapper-cmd VERSION 1.0 LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find Eigen3 package
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
    ${CMAKE_SOURCE_DIR}/include     # Include headers from the 'include' folder
)

# Gather all source files from 'src' directory
file(GLOB_RECURSE all_src_files ${CMAKE_SOURCE_DIR}/src/*.c ${CMAKE_SOURCE_DIR}/src/*.cpp)

# Create the executable
add_executable(${PROJECT_NAME} ${all_src_files})

# Find necessary dual-architecture libraries in /usr/lib64
find_library(MODAL_JSON modal_json HINTS /usr/lib64)
find_library(MODAL_PIPE modal_pipe HINTS /usr/lib64)

# Link the necessary libraries
target_link_libraries(${PROJECT_NAME} 
    pthread
    m                   # Math library
    Eigen3::Eigen       # Eigen library
    /usr/lib64/libmodal_pipe.so
      /usr/lib64/libmodal_json.so
      /usr/lib64/libvoxl_cutils.so
)