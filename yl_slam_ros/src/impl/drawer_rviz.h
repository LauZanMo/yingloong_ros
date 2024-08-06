#pragma once

#include "system/drawer_base.h"

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace YL_SLAM {

class DrawerRviz : public DrawerBase {
public:
    /**
     * @brief 构造函数
     * @param config YAML配置节点
     * @param node ROS节点，用于发布消息
     */
    DrawerRviz(const YAML::Node &config, rclcpp::Node &node);

private:
    /**
     * @brief 绘制原始图像束
     * @param timestamp 原始图像束时间戳
     * @param bundle 原始图像束
     */
    void drawRawImageBundle(int64_t timestamp, const std::vector<cv::Mat> &bundle) override;

    /**
     * @brief 绘制原始IMU数据
     * @param timestamp 原始IMU数据时间戳
     * @param imu 原始IMU数据
     */
    void drawRawImu(int64_t timestamp, const Imu &imu) override;

    /**
     * @brief 绘制参考真值位姿
     * @param timestamp 参考真值位姿时间戳
     * @param pose 参考真值位姿
     */
    void drawRefPose(int64_t timestamp, const SE3f &pose) override;

    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> raw_image_pubs_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_imu_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::vector<cv_bridge::CvImage> cv_images_;
    std::vector<std::string> camera_frame_ids_;
    std::string imu_frame_id_;
    std::string map_frame_id_;
    std::string gt_frame_id_;
};

} // namespace YL_SLAM
