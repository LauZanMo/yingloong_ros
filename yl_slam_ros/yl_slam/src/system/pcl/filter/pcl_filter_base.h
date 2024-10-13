#pragma once

#include "common/eigen_types.h"
#include "common/yaml/yaml_serialization.h"
#include "lidar/lidar_types.h"

namespace YL_SLAM {

/**
 * @brief 点云滤波器接口类
 * @details 该类为点云滤波器的接口类，所有点云滤波器都通过YAML配置文件实现动态加载
 */
class PclFilterBase {
public:
    using sPtr = std::shared_ptr<PclFilterBase>;

    /**
     * @brief 默认构造函数
     */
    PclFilterBase() = default;

    /**
     * @brief 默认析构函数
     */
    virtual ~PclFilterBase() = default;

    /**
     * @brief 从YAML节点中加载点云滤波器
     * @param config YAML节点
     * @return 所加载的点云滤波器指针
     */
    static sPtr loadFromYaml(const YAML::Node &config);

    /**
     * @brief 将点云滤波器参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 处理点云
     * @param point_cloud 处理的点云指针
     */
    virtual void process(const RawLidarPointCloud::Ptr &point_cloud) = 0;

    /**
     * @brief 打印点云滤波器参数接口
     * @param out 输出流实例
     */
    virtual void print(std::ostream &out) const = 0;

    /**
     * @brief 获取点云滤波器类型
     * @return 点云滤波器类型
     */
    [[nodiscard]] virtual std::string type() const = 0;

    /**
     * @brief 获取点云滤波器参数
     * @return 点云滤波器参数
     */
    [[nodiscard]] virtual VecXf parameters() const = 0;
};

} // namespace YL_SLAM
