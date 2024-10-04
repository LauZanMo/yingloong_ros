#pragma once

#include "common/eigen_types.h"

#include <memory>

namespace YL_SLAM {

/**
 * @brief 激光雷达几何类
 * @details 该类为相机的基础类，所有相机都通过YAML配置文件实现动态加载
 */
class LidarGeometryBase {
public:
    using sPtr      = std::shared_ptr<LidarGeometryBase>;
    using sConstPtr = std::shared_ptr<const LidarGeometryBase>;

    /**
     * @brief 构造函数
     * @param label 激光雷达标签
     * @param scan_line 激光雷达线数
     * @param nearest_distance 激光雷达所测最近距离
     * @param farthest_distance 激光雷达所测最远距离
     */
    LidarGeometryBase(std::string label, uint32_t scan_line, FloatType nearest_distance, FloatType farthest_distance);

    /**
     * @brief 从YAML配置文件中加载激光雷达
     * @param config_file YAML配置文件路径（可以是相对路径）
     * @return 加载激光雷达的指针
     * @warning 如果加载失败，则返回空指针
     */
    static sPtr loadFromYaml(const std::string &config_file);

    /**
     * @brief 将激光雷达参数写入YAML配置文件
     * @param config_file YAML配置文件路径（可以是相对路径）
     */
    void writeToYaml(const std::string &config_file) const;

    /**
     * @brief 默认析构函数
     */
    ~LidarGeometryBase() = default;

    /**
     * @brief 获取激光雷达id
     * @return 激光雷达id
     */
    [[nodiscard]] int id() const;

    /**
     * @brief 设置激光雷达id
     * @param id 激光雷达id
     */
    void setId(int id);

    /**
     * @brief 获取激光雷达标签
     * @return 激光雷达标签
     */
    [[nodiscard]] const std::string &label() const;

    /**
     * @brief 获取激光雷达线数
     * @return 激光雷达线数
     */
    [[nodiscard]] uint32_t scanLine() const;

    /**
     * @brief 获取激光雷达测量最近距离
     * @return 激光雷达测量最近距离
     */
    [[nodiscard]] FloatType nearestDistance() const;

    /**
     * @brief 获取激光雷达测量最远距离
     * @return 激光雷达测量最远距离
     */
    [[nodiscard]] FloatType farthestDistance() const;

    /**
     * @brief 检查激光雷达所测三维点是否有效
     * @param point 激光雷达所测三维点
     * @return 激光雷达所测三维点是否有效
     */
    template<typename DerivedPoint>
    [[nodiscard]] bool isPointValid(const Eigen::MatrixBase<DerivedPoint> &point) const;

    /**
     * @brief 打印激光雷达参数
     * @param out 输出流实例
     */
    void print(std::ostream &out) const;

private:
    int id_{-1};
    std::string label_;
    uint32_t scan_line_;
    FloatType nearest_dist_;
    FloatType nearest_dist2_;
    FloatType farthest_dist_;
    FloatType farthest_dist2_;
};

} // namespace YL_SLAM

#include "lidar/lidar_geometry_base.hpp"
