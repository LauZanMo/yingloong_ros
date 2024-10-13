#pragma once

#include "system/pcl/filter/pcl_filter_base.h"

#include <pcl/filters/crop_box.h>

namespace YL_SLAM {

/**
 * @brief 载体点云滤波器
 * @details 该滤波器用于滤除由于载体自身遮挡而获得的点云，实现逻辑：<br/>
 *          对点云进行遍历，若点云在以自身为中心，边长为2倍裁剪尺寸的立方体内，则进行滤除
 */
class BodyFilter : public PclFilterBase {
public:
    /**
     * @brief 构造函数
     * @param parameters 输入参数，顺序：裁剪尺寸（m）
     */
    explicit BodyFilter(const VecXf &parameters);

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
    pcl::CropBox<RawLidarPoint> crop_box_; ///< 裁剪框实例
    float crop_size_;                      ///< 裁剪尺寸（m）
};

} // namespace YL_SLAM
