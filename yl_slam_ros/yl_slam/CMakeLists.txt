cmake_minimum_required(VERSION 3.15)
project(yl_slam VERSION 0.1.0 LANGUAGES CXX)

# 设置路径
set(YL_SLAM_DIR ${PROJECT_SOURCE_DIR})

# 编译设置
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(setup)

# 编译动态库
file(GLOB_RECURSE YL_SLAM_SRC
    src/common/*.cpp
    src/camera/*.cpp
    src/lidar/*.cpp
    src/system/*.cpp
)
add_library(yl_slam SHARED
    ${YL_SLAM_SRC}
)
target_include_directories(yl_slam PUBLIC
    src
)

# 链接第三方库
find_package(absl REQUIRED)
find_package(spdlog 1.5.0 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV 4.2.0 REQUIRED)
find_package(PCL 1.11.0 REQUIRED)
find_package(GTSAM 4.2.0 EXACT)

target_include_directories(yl_slam PUBLIC
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)
target_link_directories(yl_slam PUBLIC ${PCL_LIBRARY_DIRS})
target_link_libraries(yl_slam
    absl::strings
    absl::str_format
    absl::time
    spdlog::spdlog
    ${YAML_CPP_LIBRARIES}
    Sophus::Sophus
    TBB::tbb
    ${OpenCV_LIBS}
    ${PCL_LIBRARIES}
    gtsam
)
