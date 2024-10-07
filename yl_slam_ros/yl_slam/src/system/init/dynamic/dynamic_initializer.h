#pragma once

#include "system/init/initializer_base.h"

namespace YL_SLAM {

/**
 * @brief 动态初始化器类
 * @details 尚未实现，请勿使用
 */
class DynamicInitializer : public InitializerBase {
public:
    /**
     * @brief 构造函数
     * @param parameters 输入参数
     * @param g_w 世界坐标系下的重力向量
     */
    DynamicInitializer(const VecXf &parameters, Vec3f g_w);

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
    Vec3f g_w_; ///< 世界坐标系下的重力向量
};

} // namespace YL_SLAM
