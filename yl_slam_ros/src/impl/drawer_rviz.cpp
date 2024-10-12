#include "impl/drawer_rviz.h"
#include "common/yaml/yaml_serialization.h"

#include <pcl_conversions/pcl_conversions.h>

namespace YL_SLAM {

/**
 * @brief 由cv::Mat类型转换为ros图像消息的编码
 * @param type cv::Mat类型
 * @return ros图像消息的编码
 */
std::string toImageEncoding(int type) {
    switch (type) {
        case CV_8UC3:
            return "bgr8";
        case CV_8UC1:
            return "mono8";
        case CV_16UC1:
            return "mono16";
        default:
            YL_FATAL("Unsupported image type: {}", type);
    }
}

DrawerRviz::DrawerRviz(const YAML::Node &config, rclcpp::Node &node) {
    // 获取话题参数
    const auto topic_config            = config["topic"];
    const auto raw_image_topics        = YAML::get<std::vector<std::string>>(topic_config, "raw_images");
    const auto raw_point_cloud_topics  = YAML::get<std::vector<std::string>>(topic_config, "raw_point_clouds");
    const auto raw_imu_topic           = YAML::get<std::string>(topic_config, "raw_imu");
    const auto ref_pose_topic          = YAML::get<std::string>(topic_config, "ref_pose");
    const auto current_nav_state_topic = YAML::get<std::string>(topic_config, "current_nav_state");

    // 获取坐标系id参数
    const auto frame_id_config = config["frame_id"];
    camera_frame_ids_          = YAML::get<std::vector<std::string>>(frame_id_config, "cameras");
    lidar_frame_ids_           = YAML::get<std::vector<std::string>>(frame_id_config, "lidars");
    imu_frame_id_              = YAML::get<std::string>(frame_id_config, "imu");
    map_frame_id_              = YAML::get<std::string>(frame_id_config, "map");
    gt_frame_id_               = YAML::get<std::string>(frame_id_config, "ground_truth");

    // 执行必要的检查
    YL_CHECK(raw_image_topics.size() == camera_frame_ids_.size(),
             "The size of raw images and cameras should be the same!");
    YL_CHECK(raw_point_cloud_topics.size() == lidar_frame_ids_.size(),
             "The size of raw point clouds and lidars should be the same!");

    // 设置服务质量（QoS）
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::SystemDefault);
    qos.durability(rclcpp::DurabilityPolicy::SystemDefault);

    // 创建发布者
    for (size_t i = 0; i < raw_image_topics.size(); ++i) {
        raw_image_pubs_.push_back(node.create_publisher<sensor_msgs::msg::Image>(raw_image_topics[i], qos));
        cv_images_.emplace_back();
        cv_images_.back().header.frame_id = camera_frame_ids_[i];
    }
    for (const auto &topic: raw_point_cloud_topics) {
        raw_point_cloud_pubs_.push_back(node.create_publisher<sensor_msgs::msg::PointCloud2>(topic, qos));
    }
    raw_imu_pub_           = node.create_publisher<sensor_msgs::msg::Imu>(raw_imu_topic, qos);
    ref_pose_pub_          = node.create_publisher<geometry_msgs::msg::TransformStamped>(ref_pose_topic, qos);
    current_nav_state_pub_ = node.create_publisher<nav_msgs::msg::Odometry>(current_nav_state_topic, qos);
    tf_broadcaster_        = std::make_unique<tf2_ros::TransformBroadcaster>(node);
}

void DrawerRviz::drawRawImageBundle(int64_t timestamp, const std::vector<cv::Mat> &bundle) {
    YL_CHECK(raw_image_pubs_.size() == bundle.size(), "Publishers and image bundle should have the same size!");
    rclcpp::Time ros_timestamp(timestamp);
    for (size_t i = 0; i < bundle.size(); ++i) {
        if (raw_image_pubs_[i]->get_subscription_count() != 0) {
            cv_images_[i].header.stamp = ros_timestamp;
            cv_images_[i].encoding     = toImageEncoding(bundle[i].type());
            cv_images_[i].image        = bundle[i];
            raw_image_pubs_[i]->publish(*cv_images_[i].toImageMsg());
        }
    }
}

void DrawerRviz::drawRawPointCloudBundle(int64_t timestamp, const std::vector<RawLidarPointCloud::Ptr> &bundle) {
    YL_CHECK(raw_point_cloud_pubs_.size() == bundle.size(), "Publishers and lidar bundle should have the same size!");
    rclcpp::Time ros_timestamp(timestamp);
    for (size_t i = 0; i < bundle.size(); ++i) {
        if (raw_point_cloud_pubs_[i]->get_subscription_count() != 0) {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*bundle[i], msg);
            msg.header.stamp    = ros_timestamp;
            msg.header.frame_id = lidar_frame_ids_[i];
            raw_point_cloud_pubs_[i]->publish(msg);
        }
    }
}

void DrawerRviz::drawRawImu(int64_t timestamp, const Imu &imu) {
    if (raw_imu_pub_->get_subscription_count() != 0) {
        sensor_msgs::msg::Imu msg;
        msg.header.stamp          = rclcpp::Time(timestamp);
        msg.header.frame_id       = imu_frame_id_;
        msg.angular_velocity.x    = imu.gyr[0];
        msg.angular_velocity.y    = imu.gyr[1];
        msg.angular_velocity.z    = imu.gyr[2];
        msg.linear_acceleration.x = imu.acc[0];
        msg.linear_acceleration.y = imu.acc[1];
        msg.linear_acceleration.z = imu.acc[2];
        raw_imu_pub_->publish(msg);
    }
}

void DrawerRviz::drawRefPose(int64_t timestamp, const SE3f &pose) {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp            = rclcpp::Time(timestamp);
    msg.header.frame_id         = map_frame_id_;
    msg.child_frame_id          = gt_frame_id_;
    msg.transform.translation.x = pose.translation()[0];
    msg.transform.translation.y = pose.translation()[1];
    msg.transform.translation.z = pose.translation()[2];
    msg.transform.rotation.x    = pose.unit_quaternion().x();
    msg.transform.rotation.y    = pose.unit_quaternion().y();
    msg.transform.rotation.z    = pose.unit_quaternion().z();
    msg.transform.rotation.w    = pose.unit_quaternion().w();
    if (raw_imu_pub_->get_subscription_count() != 0) {
        ref_pose_pub_->publish(msg);
    }
    tf_broadcaster_->sendTransform(msg);
}

void DrawerRviz::drawCurrentNavState(int64_t timestamp, const NavState &state) {
    if (raw_imu_pub_->get_subscription_count() != 0) {
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp            = rclcpp::Time(timestamp);
        odom_msg.header.frame_id         = map_frame_id_;
        odom_msg.child_frame_id          = imu_frame_id_;
        odom_msg.pose.pose.position.x    = state.T.translation()[0];
        odom_msg.pose.pose.position.y    = state.T.translation()[1];
        odom_msg.pose.pose.position.z    = state.T.translation()[2];
        odom_msg.pose.pose.orientation.x = state.T.unit_quaternion().x();
        odom_msg.pose.pose.orientation.y = state.T.unit_quaternion().y();
        odom_msg.pose.pose.orientation.z = state.T.unit_quaternion().z();
        odom_msg.pose.pose.orientation.w = state.T.unit_quaternion().w();
        odom_msg.twist.twist.linear.x    = state.vel[0];
        odom_msg.twist.twist.linear.y    = state.vel[1];
        odom_msg.twist.twist.linear.z    = state.vel[2];
        current_nav_state_pub_->publish(odom_msg);
    }

    geometry_msgs::msg::TransformStamped trans_msg;
    trans_msg.header.stamp            = rclcpp::Time(timestamp);
    trans_msg.header.frame_id         = map_frame_id_;
    trans_msg.child_frame_id          = imu_frame_id_;
    trans_msg.transform.translation.x = state.T.translation()[0];
    trans_msg.transform.translation.y = state.T.translation()[1];
    trans_msg.transform.translation.z = state.T.translation()[2];
    trans_msg.transform.rotation.x    = state.T.unit_quaternion().x();
    trans_msg.transform.rotation.y    = state.T.unit_quaternion().y();
    trans_msg.transform.rotation.z    = state.T.unit_quaternion().z();
    trans_msg.transform.rotation.w    = state.T.unit_quaternion().w();
    tf_broadcaster_->sendTransform(trans_msg);
}

} // namespace YL_SLAM
