#pragma once

#include "common/non_copyable.h"
#include "system/pcl/filter/pcl_filter_base.h"

namespace YL_SLAM {

/**
 * @brief 点云预处理器类
 * @details 该类加载一系列点云滤波器后，即可通过这些滤波器对点云进行预处理
 */
class PclPreprocessor : public NonCopyable {
public:
    using uPtr = std::unique_ptr<PclPreprocessor>;

    /**
     * @brief 构造函数
     * @param filters 点云滤波器指针容器
     */
    explicit PclPreprocessor(std::vector<PclFilterBase::sPtr> filters);

    /**
     * @brief 默认析构函数
     */
    ~PclPreprocessor() = default;

    /**
     * @brief 从YAML节点中加载点云预处理器
     * @param config YAML节点
     * @return 所加载的点云预处理器指针
     */
    static uPtr loadFromYaml(const YAML::Node &config);

    /**
     * @brief 将点云预处理器参数写入YAML节点
     * @return YAML节点
     */
    [[nodiscard]] YAML::Node writeToYaml() const;

    /**
     * @brief 处理点云
     * @param point_cloud 点云
     * @return 处理后的点云
     * @note 函数会先从点云创建一个新点云指针，再对点云进行一系列滤波
     */
    RawLidarPointCloud::Ptr process(const RawLidarPointCloud &point_cloud);

    /**
     * @brief 打印点云滤波器参数
     * @param out 输出流实例
     */
    void print(std::ostream &out) const;

private:
    std::vector<PclFilterBase::sPtr> filters_; ///< 点云滤波器指针容器
};

} // namespace YL_SLAM
