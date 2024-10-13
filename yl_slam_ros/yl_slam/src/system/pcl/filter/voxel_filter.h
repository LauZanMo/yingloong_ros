#pragma once

#include "system/pcl/filter/pcl_filter_base.h"

#include <pcl/filters/voxel_grid.h>

namespace YL_SLAM {

/**
 * @brief 体素点云滤波器
 * @details 该滤波器用于对点云进行下采样，实现逻辑：<br/>
 *          1. 根据输入点云，计算一个刚好包裹住该点云的立方体<br/>
 *          2. 根据设定的体素尺寸，将1中的大立方体分割成若干小立方体<br/>
 *          3. 对于每一个小立方体内的点云，计算其质心，并使用该质心代替该立方体内的点云
 */
class VoxelFilter : public PclFilterBase {
public:
    /**
     * @brief 构造函数
     * @param parameters 输入参数，顺序：体素尺寸（m）
     */
    explicit VoxelFilter(const VecXf &parameters);

    /**
     * @brief 处理点云
     * @param point_cloud 处理的点云指针
     */
    void process(const RawLidarPointCloud::Ptr &point_cloud) override;

    /**
     * @brief 打印点云滤波器参数接口
     * @param out 输出流实例
     */
    void print(std::ostream &out) const override;

    /**
     * @brief 获取点云滤波器类型
     * @return 点云滤波器类型
     */
    [[nodiscard]] std::string type() const override;

    /**
     * @brief 获取点云滤波器参数
     * @return 点云滤波器参数
     */
    [[nodiscard]] VecXf parameters() const override;

private:
    pcl::VoxelGrid<RawLidarPoint> voxel_filter_; ///< 体素网格实例
    float leaf_size_;                            ///< 体素尺寸（m）
};

} // namespace YL_SLAM
