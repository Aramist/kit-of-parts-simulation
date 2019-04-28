cmake_minimum_required(VERSION 2.8.3)
project(kop_interface)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
        std_msgs
        geometry_msgs
        roscpp
        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(
        interface src/main.cc
)

add_executable(
        velocity_publisher src/statepublisher.cc
)

target_link_libraries(
        interface ${catkin_LIBRARIES}
)

target_link_libraries(
        velocity_publisher ${catkin_LIBRARIES}
)
