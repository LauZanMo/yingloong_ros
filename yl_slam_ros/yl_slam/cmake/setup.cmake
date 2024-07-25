# 编译示例文件
option(YL_BUILD_EXAMPLES "Build examples." OFF)

# 设置记录等级，trace和debug会执行额外的检查，会影响性能
# 0:trace, 1:debug, 2:info, 3:warn, 4:error, 5:fatal
set(YL_LOG_LEVEL 0)

# 设置浮点数精度
set(YL_FLOAT_PRECISION float)

# 生成配置头文件
configure_file(
    ${YL_SLAM_DIR}/src/common/setup.h.in
    ${YL_SLAM_DIR}/src/common/setup.h
)

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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -march=native")

if (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "arm")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon")
elseif (CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2")
endif ()
