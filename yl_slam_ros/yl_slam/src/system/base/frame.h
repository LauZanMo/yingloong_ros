#pragma once

#include "common/non_copyable.h"
#include "lidar/lidar_geometry_base.h"
#include "lidar/lidar_types.h"

namespace YL_SLAM {

/**
 * @brief 帧类
 * @details 帧类包含了帧的基本信息，如时间戳、传感器及相关数据等
 */
class Frame : public NonCopyable {
public:
    using sPtr      = std::shared_ptr<Frame>;
    using sConstPtr = std::shared_ptr<const Frame>;

    /**
     * @brief 构造函数
     * @param timestamp 帧时间戳（ns）
     * @param lidar 帧所属的激光雷达指针
     * @param T_bs 帧所属传感器坐标系到body坐标系（通常是IMU）的变换
     * @param raw_pcl 帧原始点云
     */
    Frame(int64_t timestamp, const LidarGeometryBase::sPtr &lidar, const SE3f &T_bs, RawLidarPointCloud::Ptr raw_pcl);

    /**
     * @brief 默认析构函数
     */
    ~Frame() = default;

    /**
     * @brief 获取帧时间戳（ns）
     * @return 帧时间戳
     */
    [[nodiscard]] int64_t timestamp() const;

    /**
     * @brief 获取帧id
     * @return 帧id
     */
    [[nodiscard]] long id() const;

    /**
     * @brief 获取帧所属的帧束id
     * @return 帧束id
     * @warning 如果返回-1，表示帧还没有被加入到帧束中
     */
    [[nodiscard]] long bundleId() const;

    /**
     * @brief 设置帧所属的帧束id
     * @param bundle_id 帧束id
     */
    void setBundleId(long bundle_id);

    /**
     * @brief 获取帧所属的激光雷达指针
     * @return 帧所属的激光雷达指针
     */
    [[nodiscard]] const LidarGeometryBase::sConstPtr &lidar() const;

    /**
     * @brief 获取世界坐标系到帧坐标系的变换
     * @return 世界坐标系到帧坐标系的变换
     * @warning 帧创建时该值属于未设置状态，需要设置后才能使用
     */
    [[nodiscard]] const SE3f &Twf() const;

    /**
     * @brief 设置世界坐标系到帧坐标系的变换
     * @param T_wf 世界坐标系到帧坐标系的变换
     */
    void setTwf(const SE3f &T_wf);

    /**
     * @brief 获取帧所属传感器坐标系到body坐标系（通常是IMU）的变换
     * @return 帧所属传感器坐标系到body坐标系（通常是IMU）的变换
     */
    [[nodiscard]] const SE3f &Tbs() const;

    /**
     * @brief 设置帧所属传感器坐标系到body坐标系（通常是IMU）的变换
     * @param T_bs 帧所属传感器坐标系到body坐标系（通常是IMU）的变换
     */
    void setTbs(const SE3f &T_bs);

    /**
     * @brief 获取帧原始点云常量指针
     * @return 帧原始点云常量指针
     */
    [[nodiscard]] RawLidarPointCloud::ConstPtr rawPointCloud() const;

    /**
     * @brief 获取帧原始点云指针
     * @return 帧原始点云指针
     */
    [[nodiscard]] RawLidarPointCloud::Ptr &mutableRawPointCloud();

    /**
     * @brief 获取帧点云常量指针
     * @return 帧点云常量指针
     */
    [[nodiscard]] LidarPointCloud::ConstPtr pointCloud() const;

    /**
     * @brief 获取帧点云指针
     * @return 帧点云指针
     */
    [[nodiscard]] LidarPointCloud::Ptr &mutablePointCloud();

private:
    // 帧信息
    int64_t timestamp_;                  ///< 时间戳（ns）
    long id_;                            ///< 帧id（历史唯一）
    long bundle_id_{-1};                 ///< 帧束id（用于多目，历史唯一）
    LidarGeometryBase::sConstPtr lidar_; ///< 帧所属的激光雷达
    SE3f T_wf_;                          ///< 世界坐标系到帧坐标系的变换
    SE3f T_bs_;                          ///< 帧所属传感器坐标系到body坐标系（通常是IMU）的变换
    RawLidarPointCloud::Ptr raw_pcl_;    ///< 帧原始点云
    LidarPointCloud::Ptr pcl_;           ///< 帧点云（去畸变后的点云）
};

} // namespace YL_SLAM
