cmake_minimum_required(VERSION 2.8.3)
project(kop_control)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  joint_state_controller
  robot_state_publisher
)

catkin_package()

