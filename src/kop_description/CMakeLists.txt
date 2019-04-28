cmake_minimum_required(VERSION 2.8.3)

project(kop_description)

find_package(catkin REQUIRED COMPONENTS xacro)

catkin_package()

set(xacro_files
        ${CMAKE_CURRENT_SOURCE_DIR}/urdf/robot.xacro)

xacro_add_files(
        urdf/robot.xacro
        TARGET
        gen_urdfs
        INSTALL DESTINATION
        urdf
)

