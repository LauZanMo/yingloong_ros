#pragma once

#include "system/init/initializer_base.h"

namespace YL_SLAM {

/**
 * @brief 静态初始化器类
 * @details 静态初始化器实现逻辑：<br/>
 *          1. 保持静止至达到初始化时长<br/>
 *          2. 通过静止时IMU的数据计算姿态和零偏，位置和速度置零，由此得到初始导航状态<br/>
 *          3. 将当前帧束的状态设置为初始导航状态，完成初始化
 */
class StaticInitializer : public InitializerBase {
public:
    /**
     * @brief 构造函数
     * @param parameters 输入参数，顺序：初始化周期、陀螺仪零速阈值、加速度零速阈值
     * @param g_w 世界坐标系下的重力向量
     */
    StaticInitializer(const VecXf &parameters, Vec3f g_w);

    /**
     * @brief 进行初始化
     * @param bundle 新的帧束
     * @param imus 新的IMU数据
     * @return 是否初始化成功
     */
    bool initialize(const FrameBundle::sPtr &bundle, const Imus &imus) override;

    /**
     * @brief 重置初始化器
     */
    void reset() override;

    /**
     * @brief 打印初始化器参数
     * @param out 输出流实例
     */
    void print(std::ostream &out) const override;

private:
    Imus imus_; ///< 用于初始化的IMU数据容器

    FloatType zero_gyr_thresh_; ///< 陀螺仪零速阈值
    FloatType zero_acc_thresh_; ///< 加速度零速阈值
    int64_t init_period_;       ///< 初始化时长（ns）
    Vec3f g_w_;                 ///< 世界坐标系下的重力向量
};

} // namespace YL_SLAM
