cmake_minimum_required(VERSION 3.26)

# Set the toolchain file for Vcpkg
set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")

project(RRTStarProfiling)  

# Set the build type to Release for production
set(CMAKE_BUILD_TYPE Release)

# Find the OpenCV package
find_package(OpenCV CONFIG REQUIRED)

# Your target
add_executable(RRTStarProfiling main.cpp)

# Link your executable with the OpenCV libraries
target_link_libraries(RRTStarProfiling PRIVATE opencv_ml opencv_dnn opencv_core opencv_flann)

# Link your executable with OpenCV
target_link_libraries(RRTStarProfiling PRIVATE ${OpenCV_LIBS})
