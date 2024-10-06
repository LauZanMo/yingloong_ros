#pragma once

#include "common/eigen_types.h"

namespace YL_SLAM {

/**
 * @brief IMU数据类
 * @details 该类用于存储IMU数据，包括时间戳、陀螺仪数据和加速度计数据
 */
struct Imu {
    /**
     * @brief 构造函数
     * @details 构造一个时间戳为-1，值全为0的IMU数据
     */
    Imu();

    /**
     * @brief 构造函数
     * @param timestamp 时间戳（单位：ns）
     * @param gyr 陀螺仪数据
     * @param acc 加速度计数据
     */
    Imu(int64_t timestamp, Vec3f gyr, Vec3f acc);

    /**
     * @brief 默认析构函数
     */
    ~Imu() = default;

    /**
     * @brief 判断IMU数据是否已初始化
     * @return IMU数据是否已初始化
     * @note 通过默认构造函数构造的IMU数据是未初始化的
     */
    explicit operator bool() const;

    /**
     * @brief 输出IMU数据
     * @param out 输出流
     * @param imu IMU数据
     * @return 输出流
     */
    friend std::ostream &operator<<(std::ostream &out, const Imu &imu);

    /**
     * @brief 对两个IMU数据进行线性插值
     * @param imu0 第一个IMU数据
     * @param imu1 第二个IMU数据
     * @param timestamp 插值时刻的时间戳（单位：ns）
     * @return 插值后的IMU数据
     */
    static Imu interpolate(const Imu &imu0, const Imu &imu1, int64_t timestamp);

    int64_t timestamp; ///< 时间戳（ns）
    Vec3f gyr;         ///< 陀螺仪数据（rad/s）
    Vec3f acc;         ///< 加速度计数据（m/s^2）
};

using Imus = std::vector<Imu, Eigen::aligned_allocator<Imu>>;

} // namespace YL_SLAM
