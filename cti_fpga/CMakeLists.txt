cmake_minimum_required(VERSION 3.5)
project(cti_fpga)

find_package(ament_cmake REQUIRED)
find_package(cti_fpga_data REQUIRED)
find_package(cti_fpga_serial REQUIRED)
find_package(cti_fpga_serial_msgs REQUIRED)
find_package(mcu_ota REQUIRED)
ament_package()
