#pragma once

#include "camera/camera_geometry_base.h"
#include "common/eigen_types.h"
#include "common/non_copyable.h"

namespace YL_SLAM {

/**
 * @brief 相机组类
 * @details 相机组类用于管理多个相机实例及其对应的外参
 */
class CameraRig : public NonCopyable {
public:
    using sPtr      = std::shared_ptr<CameraRig>;
    using TbsVector = std::vector<SE3f, Eigen::aligned_allocator<SE3f>>;

    /**
     * @brief 构造函数
     * @param label 相机组标签
     * @param cameras 相机组中的所有的相机实例指针
     * @param T_bs_vec 相机组中每个相机的外参
     * @warning 输入的相机实例指针和外参向量的长度需要一致
     */
    CameraRig(std::string label, const std::vector<CameraGeometryBase::sPtr> &cameras, TbsVector T_bs_vec);

    /**
     * @brief 析构函数
     */
    ~CameraRig() = default;

    /**
     * @brief 从YAML配置文件中加载相机组
     * @param config_file YAML配置文件路径（可以是相对路径）
     * @return 加载相机组的指针
     * @warning 如果加载失败，则返回空指针
     */
    static sPtr loadFromYaml(const std::string &config_file);

    /**
     * @brief 将相机组参数写入YAML配置文件
     * @param config_file YAML配置文件路径（可以是相对路径）
     */
    void writeToYaml(const std::string &config_file) const;

    /**
     * @brief 获取相机组标签
     * @return 相机组标签
     */
    [[nodiscard]] const std::string &label() const;

    /**
     * @brief 获取相机组中的指定索引的相机实例指针
     * @param idx 相机索引
     * @return 相机实例指针
     */
    [[nodiscard]] const CameraGeometryBase::sPtr &camera(size_t idx) const;

    /**
     * @brief 获取相机组中实例的总数
     * @return 相机实例总数
     */
    [[nodiscard]] size_t numCameras() const;

    /**
     * @brief 获取相机组中指定索引的相机外参
     * @param idx 相机索引
     * @return 相机外参
     */
    [[nodiscard]] const SE3f &T_bs(size_t idx) const;

    /**
     * @brief 打印相机组参数
     * @param out 输出流实例
     */
    void print(std::ostream &out) const;

private:
    std::string label_;
    std::vector<CameraGeometryBase::sPtr> cameras_;
    TbsVector T_bs_vec_;
};

} // namespace YL_SLAM
