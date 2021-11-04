cmake_minimum_required(VERSION 2.8.12)
project(opencv_cap2 CXX)

# set(CMAKE_CXX_FLAGS "-std=c++11 -O2 -Wall")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(${PROJECT_NAME} opencv_cap2.cpp)

find_package(PkgConfig)
pkg_check_modules(RealSense2 REQUIRED realsense2)
include_directories(${RealSense2_INCLUDE_DIRS})
link_directories(${RealSense2_LIBRARY_DIRS})

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenCV REQUIRED)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} ${RealSense2_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
