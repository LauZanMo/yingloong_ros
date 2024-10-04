#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace YL_SLAM {

using LidarPoint      = pcl::PointXYZI;
using LidarPointCloud = pcl::PointCloud<LidarPoint>;

struct RawLidarPoint {
    PCL_ADD_POINT4D;  ///< 三维点（齐次形式）
    float intensity;  ///< 强度
    double timestamp; ///< 绝对时间戳（ns）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
using RawLidarPointCloud = pcl::PointCloud<RawLidarPoint>;

struct OusterLidarPoint {
    PCL_ADD_POINT4D; ///< 三维点（齐次形式）
    float intensity; ///< 强度
    uint32_t t;      ///< Ouster：相对于扫描开始时间的差值（ns）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
using OusterLidarPointCloud = pcl::PointCloud<OusterLidarPoint>;

struct VelodyneLidarPoint {
    PCL_ADD_POINT4D; ///< 三维点（齐次形式）
    float intensity; ///< 强度
    float time;      ///< Velodyne：相对于扫描开始时间的差值（s）
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
using VelodyneLidarPointCloud = pcl::PointCloud<VelodyneLidarPoint>;

using LivoxLidarPoint      = RawLidarPoint;
using LivoxLidarPointCloud = RawLidarPointCloud;

} // namespace YL_SLAM

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(YL_SLAM::RawLidarPoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(YL_SLAM::OusterLidarPoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (uint32_t, t, t))

POINT_CLOUD_REGISTER_POINT_STRUCT(YL_SLAM::VelodyneLidarPoint,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (float, time, time))
// clang-format on
