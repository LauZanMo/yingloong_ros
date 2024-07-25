cmake_minimum_required(VERSION 3.15)
project(yl_msgs)

if (POLICY CMP0148)
    cmake_policy(SET CMP0148 OLD)
endif ()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

file(GLOB ${PROJECT_NAME}_msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
file(GLOB ${PROJECT_NAME}_srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
rosidl_generate_interfaces(${PROJECT_NAME}
    ${${PROJECT_NAME}_msg_files}
    ${${PROJECT_NAME}_srv_files}
    DEPENDENCIES std_msgs
)

ament_package()
