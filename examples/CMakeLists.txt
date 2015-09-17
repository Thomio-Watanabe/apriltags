cmake_minimum_required (VERSION 2.6)

link_libraries(apriltags)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})
include_directories(/usr/include/eigen3)

add_executable(apriltags_demo apriltags_demo.cpp Serial.cpp)
target_link_libraries(apriltags_demo ${OpenCV_LIBS} v4l1 v4l2)

add_executable(imu imu.cpp Serial.cpp)
