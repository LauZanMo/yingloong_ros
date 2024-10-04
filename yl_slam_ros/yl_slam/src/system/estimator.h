#pragma once

#include "common/concurrent_container_types.h"
#include "common/sensor/imu.h"
#include "common/timer.h"
#include "lidar/lidar_rig.h"
#include "system/base/map_server.h"
#include "system/drawer_base.h"

namespace YL_SLAM {

/**
 * @brief 估计器类
 * @details 估计器类是SLAM系统的核心，负责接收图像和IMU数据，进行状态估计，并将关键帧束输入后端优化
 */
class Estimator : public NonCopyable {
public:
    using uPtr = std::unique_ptr<Estimator>;

    /**
     * @brief 构造函数
     * @param config YAML配置节点
     * @param drawer 绘制器指针
     */
    Estimator(const YAML::Node &config, DrawerBase::sPtr drawer);

    /**
     * @brief 析构函数
     */
    ~Estimator();

    /**
     * @brief 重置估计器
     */
    void reset();

    /**
     * @brief 输入图像束
     * @param timestamp 时间戳（ns）
     * @param images 同一时间戳下多相机的图像
     * @note 仅对输入图像做可视化
     */
    void addImageBundle(int64_t timestamp, const std::vector<cv::Mat> &images);

    /**
     * @brief 输入原始激光雷达点云束
     * @param timestamp 时间戳（ns）
     * @param pointclouds 同一时间戳下多激光雷达的原始点云
     * @note 需要按照激光雷达组（lidar_rig）的顺序输入激光雷达原始点云
     */
    void addPointCloudBundle(int64_t timestamp, const std::vector<RawLidarPointCloud::Ptr> &point_clouds);

    /**
     * @brief 输入IMU数据
     * @param imu IMU数据
     */
    void addImu(const Imu &imu);

private:
    /**
     * @brief 估计线程
     */
    void estimateLoop();

    /**
     * @brief 重置估计器的内部实现
     */
    void internalReset();

    /**
     * @brief 同步缓冲区，使缓冲区头部元素时间戳对齐
     * @details 阻塞地同步缓冲区，使缓冲区头部元素时间戳对齐，若检测到野值或系统关闭，则返回失败
     * @return 是否成功同步缓冲区
     */
    bool syncBuffer();

    /**
     * @brief 从缓冲区中获取测量值
     * @details 阻塞地从缓冲区分别获取当前帧束及其时间戳之前的IMU，若检测到野值或系统关闭，则返回失败
     * @param imus 上一帧束到当前帧束间所有的IMU
     * @return 是否成功获取测量值
     * @note 获取的帧束可在cur_frame_bundle_中获取
     */
    bool getMeasurementFromBuffer(Imus &imus);

    /**
     * @brief 从缓冲区中获取指定时间戳之前的IMU
     * @details 阻塞地从缓冲区获取指定时间戳之前的IMU，若检测到野值或系统关闭，则返回失败
     * @param timestamp 时间戳
     * @param imus 时间戳之前的IMU
     * @return 是否成功获取IMU
     */
    bool getImusFromBuffer(int64_t timestamp, Imus &imus);

    // 系统
    LidarRig::sPtr lidar_rig_;   ///< 激光雷达组
    MapServer::sPtr map_server_; ///< 地图服务器
    DrawerBase::sPtr drawer_;    ///< 绘制器

    // 多线程
    std::atomic<bool> running_{false};             ///< 运行标志位
    std::atomic<bool> reset_{false};               ///< 重置标志位
    spin_rw_mutex_t reset_mutex_;                  ///< 重置互斥量（自旋）
    std::unique_ptr<std::thread> estimate_thread_; ///< 估计线程指针

    // 缓冲区
    conc_queue<FrameBundle::sPtr> frame_bundle_buffer_;         ///< 帧束缓冲区
    conc_queue<Imu, Eigen::aligned_allocator<Imu>> imu_buffer_; ///< IMU缓冲区
    FrameBundle::sPtr cur_frame_bundle_, last_frame_bundle_;    ///< 当前帧束和上一帧束
    Imu last_imu_;                                              ///< 上一IMU

    // 计时器
    YL_DECLARE_TIMER(deskew_timer_);
};

} // namespace YL_SLAM
