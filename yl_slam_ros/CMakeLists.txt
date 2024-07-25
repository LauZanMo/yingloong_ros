cmake_minimum_required(VERSION 3.15)
project(yl_slam_ros)

# 设置编译选项
set(CMAKE_CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

if (CMAKE_BUILD_TYPE MATCHES Release)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
elseif (CMAKE_BUILD_TYPE MATCHES RelWithDebInfo)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3")
elseif (CMAKE_BUILD_TYPE MATCHES Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")
endif ()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")

# 编译yl_slam库
add_subdirectory(yl_slam)
install(TARGETS yl_slam DESTINATION lib)

# 编译ros程序
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# euroc2bag
ament_auto_add_executable(euroc2bag
    src/euroc2bag.cpp
)
target_link_libraries(euroc2bag yl_slam)
target_include_directories(euroc2bag PUBLIC ${PROJECT_SOURCE_DIR}/src)

# 生成功能包
# install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/launch/)
ament_auto_package()