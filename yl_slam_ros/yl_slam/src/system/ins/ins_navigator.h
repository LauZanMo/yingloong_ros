#pragma once

#include "common/eigen_types.h"
#include "common/nav_state.h"
#include "common/sensor/imu.h"

namespace YL_SLAM {

/**
 * @brief 惯性导航器类
 */
class InsNavigator {
public:
    /**
     * @brief 构造函数
     * @param g_w 世界坐标系下的重力向量
     */
    explicit InsNavigator(Vec3f g_w);

    /**
     * @brief 默认析构函数
     */
    ~InsNavigator() = default;

    /**
     * @brief 积分递推（机械编排）
     * @param imus IMU数据容器
     */
    void propagate(const Imus &imus);

    /**
     * @brief 获取当前导航状态
     * @return 当前导航状态
     */
    [[nodiscard]] const NavState &state() const;

    /**
     * @brief 获取当前位姿
     * @return 当前位姿
     */
    [[nodiscard]] const SE3f &Twb() const;

    /**
     * @brief 更新当前导航状态
     * @param state 输入导航状态
     * @warning 当前导航状态的时间戳也会改变，可能需要重新积分到当前时刻
     */
    void update(const NavState &state);

    /**
     * @brief 更新当前位姿
     * @param T_wb 输入位姿
     */
    void update(const SE3f &T_wb);

private:
    NavState state_; ///< 当前时刻状态
    Vec3f g_w_;      ///< 世界坐标系下的重力向量
};

} // namespace YL_SLAM
