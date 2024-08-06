#include "system/estimator.h"
#include "common/yaml/yaml_serialization.h"

#include <absl/strings/str_cat.h>
#include <iostream>

namespace YL_SLAM {

Estimator::Estimator(const YAML::Node &config, DrawerBase::sPtr drawer) : drawer_(std::move(drawer)) {
    YL_INFO("Starting estimator...");

    // 读取相机文件路径，并初始化相机组
    const auto camera_rig_file = YAML::get<std::string>(config, "camera_rig_file");
    camera_rig_                = CameraRig::loadFromYaml(camera_rig_file);
    camera_rig_->print(std::cout);

    // 初始化地图服务器
    map_server_ = std::make_shared<MapServer>(config["map"]);
    drawer_->setMapServer(map_server_);

    // 读取估计器配置
    const auto estimator_config = config["estimator"];
    image_align_max_level_      = YAML::get<size_t>(estimator_config, "image_align_max_level");

    // 启动估计线程
    running_         = true;
    estimate_thread_ = std::make_unique<std::thread>(&Estimator::estimateLoop, this);
}

Estimator::~Estimator() {
    if (running_) {
        YL_INFO("Closing estimator...");

        // 结束估计线程
        running_ = false;
        frame_bundle_buffer_.push(nullptr);
        imu_buffer_.push(Imu());
        estimate_thread_->join();
    }
}

void Estimator::reset() {
    YL_INFO("Resetting estimator...");
    reset_ = true;
}

void Estimator::addImageBundle(int64_t timestamp, const std::vector<cv::Mat> &images) {
    YL_START_TIMER(pyr_timer_);

    // 生成帧束
    YL_CHECK(images.size() == camera_rig_->numCameras(), "The number of images should match the number of cameras!");
    std::vector<Frame::sPtr> frames(images.size());
    for (size_t i = 0; i < images.size(); ++i) {
        // TODO: 1. 测试多线程是否会加速 2. timestamp需要减去后端估计的延迟
        frames[i] = std::make_shared<Frame>(timestamp, camera_rig_->camera(i), camera_rig_->T_bc(i), images[i],
                                            image_align_max_level_);
    }

    // 将帧束加入缓冲区
    if (!reset_) {
        spin_rwlock_t lock(reset_mutex_, false);
        frame_bundle_buffer_.push(std::make_shared<FrameBundle>(frames));
    }

    YL_STOP_TIMER(pyr_timer_);
}

void Estimator::addImu(const Imu &imu) {
    // 将IMU加入缓冲区
    if (!reset_) {
        spin_rwlock_t lock(reset_mutex_, false);
        imu_buffer_.push(imu);
    }
}

void Estimator::estimateLoop() {
    // 同步缓冲区
    if (!syncBuffer())
        return;

    // 估计循环
    while (true) {
        // 从缓冲区中获取测量值
        Imus imus;
        if (!getMeasurementFromBuffer(imus)) {
            return;
        }

        YL_INFO("Image bundle timestamp: {}ns", cur_frame_bundle_->timestamp());
        YL_INFO("IMU timestamp range: {}ns - {}ns", imus.front().timestamp, imus.back().timestamp);

        // 检查是否需要重置，重置后需要同步缓冲区
        if (reset_) {
            internalReset();
            if (!syncBuffer())
                return;
        }
    }
}

void Estimator::internalReset() {
    spin_rwlock_t lock(reset_mutex_, true);

    // 清空缓冲区
    frame_bundle_buffer_.clear();
    imu_buffer_.clear();
    cur_frame_bundle_.reset();
    last_frame_bundle_.reset();
    last_imu_ = Imu();
    YL_INFO("Estimator reset!");

    // 重置标志位
    reset_ = false;
}

bool Estimator::syncBuffer() {
    // 从缓冲区中获取一个历元的测量值，用于比较
    frame_bundle_buffer_.pop(last_frame_bundle_);
    imu_buffer_.pop(last_imu_);
    if (!running_) {
        return false;
    }

    // 对IMU和图像进行对齐
    while (last_imu_.timestamp >= last_frame_bundle_->timestamp()) {
        frame_bundle_buffer_.pop(last_frame_bundle_);
        if (!running_) {
            return false;
        }
    }
    Imus imus;
    cur_frame_bundle_ = last_frame_bundle_;
    return getImusFromBuffer(cur_frame_bundle_->timestamp(), imus);
}

bool Estimator::getMeasurementFromBuffer(Imus &imus) {
    // 保存上一帧束，并从图像缓冲区中取出新帧束
    last_frame_bundle_ = cur_frame_bundle_;
    frame_bundle_buffer_.pop(cur_frame_bundle_);
    if (!running_) {
        return false;
    }

    // 从缓冲区中获取当前帧束之前的IMU
    return getImusFromBuffer(cur_frame_bundle_->timestamp(), imus);
}

bool Estimator::getImusFromBuffer(int64_t timestamp, Imus &imus) {
    // 将上一个IMU加入容器
    YL_CHECK(last_imu_, "Last IMU should be initialized!");
    imus.push_back(last_imu_);

    // 从IMU缓冲区中取出timestamp之前的IMU
    while (true) {
        Imu imu;
        imu_buffer_.pop(imu);
        if (!running_) {
            return false;
        }

        // 根据时间戳对IMU做对应处理
        if (imu.timestamp < timestamp) {
            imus.push_back(std::move(imu));
        } else {
            last_imu_ = Imu::interpolate(imus.back(), imu, timestamp);
            imus.push_back(last_imu_);
            return true;
        }
    }
}

} // namespace YL_SLAM
