#pragma once

#include "common/concurrent_container_types.h"
#include "common/sensor/imu.h"
#include "system/base/map_server.h"

#include <atomic>
#include <thread>

namespace YL_SLAM {

/**
 * @brief 绘制器类
 * @details 绘制器类用于可视化SLAM系统中的各种数据
 */
class DrawerBase : public NonCopyable {
public:
    using sPtr = std::shared_ptr<DrawerBase>;

    /**
     * @brief 构造函数
     * @details 构造函数会启动一个内部线程，用于执行绘制任务
     */
    DrawerBase();

    /**
     * @brief 析构函数
     */
    virtual ~DrawerBase();

    /**
     * @brief 设置地图服务器
     * @param server 地图服务器指针
     */
    void setMapServer(const MapServer::sPtr &server);

    /**
     * @brief 更新原始图像束
     * @param timestamp 原始图像束时间戳
     * @param bundle 原始图像束
     */
    void updateRawImageBundle(int64_t timestamp, const std::vector<cv::Mat> &bundle);

    /**
     * @brief 更新原始IMU数据
     * @param timestamp 原始IMU数据时间戳
     * @param imu 原始IMU数据
     */
    void updateRawImu(int64_t timestamp, const Imu &imu);

    /**
     * @brief 更新参考真值位姿
     * @param timestamp 参考真值位姿时间戳
     * @param pose 参考真值位姿
     */
    void updateRefPose(int64_t timestamp, const SE3f &pose);

protected:
    /**
     * @brief 绘制原始图像束
     * @param timestamp 原始图像束时间戳
     * @param bundle 原始图像束
     */
    virtual void drawRawImageBundle(int64_t timestamp, const std::vector<cv::Mat> &bundle) = 0;

    /**
     * @brief 绘制原始IMU数据
     * @param timestamp 原始IMU数据时间戳
     * @param imu 原始IMU数据
     */
    virtual void drawRawImu(int64_t timestamp, const Imu &imu) = 0;

    /**
     * @brief 绘制参考真值位姿
     * @param timestamp 参考真值位姿时间戳
     * @param pose 参考真值位姿
     */
    virtual void drawRefPose(int64_t timestamp, const SE3f &pose) = 0;

    MapServer::sPtr map_server_;

private:
    using draw_task_t = std::function<void()>;

    /**
     * @brief 绘制线程，不断从缓冲区中取出任务进行绘制
     */
    void drawLoop();

    std::atomic<bool> running_{false};
    std::unique_ptr<std::thread> draw_thread_;

    conc_queue<draw_task_t> draw_task_buffer_;
};

} // namespace YL_SLAM
