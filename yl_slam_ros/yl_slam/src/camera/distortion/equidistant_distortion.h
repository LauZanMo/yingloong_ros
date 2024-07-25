#pragma once

#include "common/eigen_types.h"

namespace YL_SLAM {

/**
 * @brief 等距畸变的畸变模型类
 * @details 该类为畸变模型的等距畸变实现
 */
class EquidistantDistortion {
public:
    /**
     * @brief 构造函数
     * @param parameters 相机畸变参数向量，顺序[k1, k2, k3, k4]
     */
    explicit EquidistantDistortion(const VecXf &parameters);

    /**
     * @brief 析构函数
     */
    ~EquidistantDistortion() = default;

    /**
     * @brief 将无畸变的归一化平面坐标转换为畸变的归一化平面坐标
     * @param uv 无畸变的归一化平面坐标
     * @return 畸变的归一化平面坐标
     */
    [[nodiscard]] Vec2f distort(const Vec2f &uv) const;

    /**
     * @brief 将畸变的归一化平面坐标转换为无畸变的归一化平面坐标
     * @param uv 畸变的归一化平面坐标
     * @return 无畸变的归一化平面坐标
     */
    [[nodiscard]] Vec2f undistort(const Vec2f &uv) const;

    /**
     * @brief 计算畸变的归一化平面坐标对无畸变的归一化平面坐标的雅可比矩阵
     * @return 畸变的归一化平面坐标对无畸变的归一化平面坐标的雅可比矩阵
     */
    [[nodiscard]] Mat22f jacobian(const Vec2f &uv) const;

    /**
     * @brief 打印畸变模型参数
     * @param out 输出流实例
     */
    void print(std::ostream &out) const;

    /**
     * @brief 获取相机畸变参数
     * @return 包含相机畸变参数的向量
     */
    [[nodiscard]] VecXf distortionParameters() const;

private:
    /**
     * @brief 将无畸变的theta转换为畸变的theta
     * @param theta 无畸变的theta
     * @return 畸变的theta
     */
    [[nodiscard]] FloatType distortTheta(FloatType theta) const;

    /**
     * @brief 计算畸变的theta对无畸变的theta的导数
     * @param theta 无畸变的theta
     * @return 畸变的theta对无畸变的theta的导数
     */
    [[nodiscard]] FloatType jacobianTheta(FloatType theta) const;

    FloatType k1_;
    FloatType k2_;
    FloatType k3_;
    FloatType k4_;

    static constexpr FloatType thresh_{YL_FLOAT(1e-8)};
};

} // namespace YL_SLAM
