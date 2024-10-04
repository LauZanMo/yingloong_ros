#pragma once

#include "lidar/lidar_types.h"

namespace YL_SLAM::lidar_helper {

/**
 * @brief 将ouster激光雷达输出点云转换为系统输入点云
 * @param src ouster激光雷达输出点云
 * @return 系统输入点云指针
 * @warning 返回值的包头时间戳单位为ns，与pcl官方定义不同，使用toROSMsg前需要注意时间戳问题
 */
RawLidarPointCloud::Ptr convert(OusterLidarPointCloud &src);

/**
 * @brief 将velodyne激光雷达输出点云转换为系统输入点云
 * @param src velodyne激光雷达输出点云
 * @return 系统输入点云指针
 * @warning 返回值的包头时间戳单位为ns，与pcl官方定义不同，使用toROSMsg前需要注意时间戳问题
 */
RawLidarPointCloud::Ptr convert(VelodyneLidarPointCloud &src);

/**
 * @brief 将livox激光雷达输出点云转换为系统输入点云
 * @param src livox激光雷达输出点云
 * @return 系统输入点云指针
 * @warning 返回值的包头时间戳单位为ns，与pcl官方定义不同，使用toROSMsg前需要注意时间戳问题
 */
RawLidarPointCloud::Ptr convert(LivoxLidarPointCloud &src);

} // namespace YL_SLAM::lidar_helper
