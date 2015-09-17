cmake_minimum_required (VERSION 2.6)

link_libraries(apriltags)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PkgConfig)
pkg_check_modules(PC_Eigen3 REQUIRED eigen3)
include_directories(${PC_Eigen3_INCLUDE_DIRS})

add_executable(apriltags_demo apriltags_demo.cpp Serial.cpp)
target_link_libraries(apriltags_demo ${OpenCV_LIBS} v4l1 v4l2)

add_executable(imu imu.cpp Serial.cpp)
