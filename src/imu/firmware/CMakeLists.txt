cmake_minimum_required(VERSION 3.0.2)
project(imu)

include_directories(${ROS_LIB_DIR})

FILE(GLOB_RECURSE ros_src
    "${ROS_LIB_DIR}/*.cpp"
    "${ROS_LIB_DIR}/*.h")
add_library(ros_lib ${ros_src})

add_teensy_executable(imu_interface main.cpp)
target_link_libraries(imu_interface ros_lib)