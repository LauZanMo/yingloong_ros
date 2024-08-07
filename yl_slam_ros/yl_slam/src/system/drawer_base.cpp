#include "drawer_base.h"

namespace YL_SLAM {

DrawerBase::DrawerBase() {
    YL_INFO("Starting drawer...");

    // 启动绘制线程
    running_     = true;
    draw_thread_ = std::make_unique<std::thread>(&DrawerBase::drawLoop, this);
}

DrawerBase::~DrawerBase() {
    YL_INFO("Closing drawer...");

    // 结束绘制线程
    running_ = false;
    draw_task_buffer_.push(nullptr);
    draw_thread_->join();
}

void DrawerBase::setMapServer(const std::shared_ptr<MapServer> &server) {
    map_server_ = server;
}

void DrawerBase::reset() {
    YL_INFO("Resetting drawer...");
    reset_ = true;
}

void DrawerBase::updateRawImageBundle(int64_t timestamp, const std::vector<cv::Mat> &bundle) {
    if (!reset_) {
        draw_task_buffer_.push([this, timestamp, bundle]() {
            drawRawImageBundle(timestamp, bundle);
        });
    }
}

void DrawerBase::updateRawImu(int64_t timestamp, const Imu &imu) {
    if (!reset_) {
        draw_task_buffer_.push([this, timestamp, imu]() {
            drawRawImu(timestamp, imu);
        });
    }
}

void DrawerBase::updateRefPose(int64_t timestamp, const SE3f &pose) {
    if (!reset_) {
        draw_task_buffer_.push([this, timestamp, pose]() {
            drawRefPose(timestamp, pose);
        });
    }
}

void DrawerBase::drawLoop() {
    while (true) {
        // 从缓冲区获取任务
        draw_task_t task;
        draw_task_buffer_.pop(task);

        // 如果不在运行状态则退出
        if (!running_) {
            break;
        }

        // 执行绘制任务
        task();

        // 检查是否需要重置
        if (reset_) {
            internalReset();
        }
    }
}

void DrawerBase::internalReset() {
    // 清空缓冲区
    draw_task_buffer_.clear();

    // 重置标志位
    reset_ = false;
}

} // namespace YL_SLAM
