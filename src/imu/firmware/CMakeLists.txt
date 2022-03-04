cmake_minimum_required(VERSION 3.0.2)
project(imu)

include_directories(${ROS_LIB_DIR})

generate_arduino_firmware(
    imu_interface
    SRCS imu_interface.cpp ${ROS_LIB_DIR}/time.cpp
    HDRS util.h RxData.h
    BOARD teensy40
    PORT /dev/ttyACM0
    AFLAGS -n -w
    SETTINGS opt=o2std
)
