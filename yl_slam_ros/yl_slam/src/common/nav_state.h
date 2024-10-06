#pragma once

#include "common/eigen_types.h"

namespace YL_SLAM {

/**
 * @brief 导航状态类
 * @details 该类用于存储导航状态，包括时间戳、位姿、速度、陀螺仪零偏和加速度计零偏
 */
struct NavState {
    /**
     * @brief 构造函数
     * @details 构造一个时间戳为-1，值全为0的导航状态
     */
    NavState();

    /**
     * @brief 构造函数
     * @param timestamp 时间戳（单位：ns）
     * @param T 位姿
     * @param vel 速度
     * @param bg 陀螺仪零偏（rad/s）
     * @param ba 加速度计零偏（m/s^2）
     */
    NavState(int64_t timestamp, const SE3f &T, Vec3f vel, Vec3f bg, Vec3f ba);

    /**
     * @brief 默认析构函数
     */
    ~NavState() = default;

    /**
     * @brief 判断导航状态是否已初始化
     * @return 导航状态是否已初始化
     * @note 通过默认构造函数构造的导航状态是未初始化的
     */
    explicit operator bool() const;

    /**
     * @brief 输出导航状态
     * @param out 输出流
     * @param state 导航状态
     * @return 输出流
     */
    friend std::ostream &operator<<(std::ostream &out, const NavState &state);

    int64_t timestamp; ///< 时间戳（ns）
    SE3f T;            ///< 位姿
    Vec3f vel;         ///< 速度（m/s）
    Vec3f bg;          ///< 陀螺仪零偏（rad/s）
    Vec3f ba;          ///< 加速度计零偏（m/s^2）
};

using NavStates = std::vector<NavState, Eigen::aligned_allocator<NavState>>;

} // namespace YL_SLAM
