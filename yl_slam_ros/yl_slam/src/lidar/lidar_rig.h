#pragma once

#include "common/eigen_types.h"
#include "common/non_copyable.h"
#include "lidar/lidar_geometry_base.h"

namespace YL_SLAM {

class LidarRig : NonCopyable {
public:
    using sPtr      = std::shared_ptr<LidarRig>;
    using TbsVector = std::vector<SE3f, Eigen::aligned_allocator<SE3f>>;

    /**
     * @brief 构造函数
     * @param label 激光雷达组标签
     * @param lidars 激光雷达组中的所有的激光雷达实例指针
     * @param T_bs_vec 激光雷达组中每个激光雷达的外参
     * @warning 输入的激光雷达实例指针和外参向量的长度需要一致
     */
    LidarRig(std::string label, const std::vector<LidarGeometryBase::sPtr> &lidars, TbsVector T_bs_vec);

    /**
     * @brief 析构函数
     */
    ~LidarRig() = default;

    /**
     * @brief 从YAML配置文件中加载激光雷达组
     * @param config_file YAML配置文件路径（可以是相对路径）
     * @return 加载激光雷达组的指针
     * @warning 如果加载失败，则返回空指针
     */
    static sPtr loadFromYaml(const std::string &config_file);

    /**
     * @brief 将激光雷达组参数写入YAML配置文件
     * @param config_file YAML配置文件路径（可以是相对路径）
     */
    void writeToYaml(const std::string &config_file) const;

    /**
     * @brief 获取激光雷达组标签
     * @return 激光雷达组标签
     */
    [[nodiscard]] const std::string &label() const;

    /**
     * @brief 获取激光雷达组中的指定索引的激光雷达实例指针
     * @param idx 激光雷达索引
     * @return 激光雷达实例指针
     */
    [[nodiscard]] const LidarGeometryBase::sPtr &lidar(size_t idx) const;

    /**
     * @brief 获取激光雷达组中实例的总数
     * @return 激光雷达实例总数
     */
    [[nodiscard]] size_t numLidars() const;

    /**
     * @brief 获取激光雷达组中指定索引的激光雷达外参
     * @param idx 激光雷达索引
     * @return 激光雷达外参
     */
    [[nodiscard]] const SE3f &T_bs(size_t idx) const;

    /**
     * @brief 打印激光雷达组参数
     * @param out 输出流实例
     */
    void print(std::ostream &out) const;

private:
    std::string label_;
    std::vector<LidarGeometryBase::sPtr> lidars_;
    TbsVector T_bs_vec_;
};

} // namespace YL_SLAM
