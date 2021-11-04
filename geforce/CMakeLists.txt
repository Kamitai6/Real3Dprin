cmake_minimum_required(VERSION 2.8.12)
project(geforce CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_PREFIX_PATH "./SerialPort/build/CMakeFiles/Export")

add_executable(${PROJECT_NAME} geforce.cpp)

find_package(PkgConfig)
pkg_check_modules(RealSense2 REQUIRED realsense2)
include_directories(${RealSense2_INCLUDE_DIRS})
link_directories(${RealSense2_LIBRARY_DIRS})

find_package(OpenCV REQUIRED)
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})

find_package(SerialPort)

target_link_libraries(${PROJECT_NAME} ${RealSense2_LIBRARIES} ${OpenCV_LIBS} SerialPort)
