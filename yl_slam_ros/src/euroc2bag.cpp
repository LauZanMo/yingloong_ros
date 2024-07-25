#include "common/logger.h"
#include "common/timer.h"

#include <absl/strings/numbers.h>
#include <absl/strings/str_cat.h>
#include <absl/strings/str_split.h>
#include <common/yaml/yaml_serialization.h>
#include <fstream>
#include <opencv2/imgcodecs.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

using namespace YL_SLAM;

void convertImages(const std::unique_ptr<rosbag2_cpp::Writer> &writer, const std::string &folder,
                   const std::string &label);

void convertImu(const std::unique_ptr<rosbag2_cpp::Writer> &writer, const std::string &folder,
                const std::string &label);

void convertGT(const std::unique_ptr<rosbag2_cpp::Writer> &writer, const std::string &folder, const std::string &label,
               const std::string &frame_id);

int main(int argc, char **argv) {
    // 输入检查
    if (argc < 2) {
        YL_FATAL("Usage: ros2 run yl_slam_ros euroc2bag <config_file>");
    }

    // 初始化Logger
    const auto log_path = absl::StrCat(YL_SLAM_DIR, "/logs/", Timer::currentTime());
    Logger::initialize(true, log_path, argv[0]);

    // 加载配置
    std::string file_name(argv[1]);
    if (file_name.front() != '/') { // 相对路径
        file_name = absl::StrCat(YL_SLAM_DIR, "/", file_name);
    }
    const auto config = YAML::load(file_name);
    auto dataset_path = YAML::get<std::string>(config, "dataset_path");
    if (dataset_path.front() != '/') { // 相对路径
        dataset_path = absl::StrCat(YL_SLAM_DIR, "/", dataset_path);
    }
    const auto dataset_names = YAML::get<std::vector<std::string>>(config, "dataset_names");

    // 将Euroc数据集转换为ros2 bag
    YL_INFO("Converting Euroc dataset to ros2 bag...");
    for (const auto &name: dataset_names) {
        YL_INFO("Converting dataset: {}", name);
        const auto dataset_folder = absl::StrCat(dataset_path, "/", name);
        const auto mav0_folder    = absl::StrCat(dataset_folder, "/mav0");
        const auto writer         = std::make_unique<rosbag2_cpp::Writer>();
        writer->open(absl::StrCat(dataset_folder, "/", name));
        convertImages(writer, mav0_folder, "cam0");
        convertImages(writer, mav0_folder, "cam1");
        convertImu(writer, mav0_folder, "imu0");
        convertGT(writer, mav0_folder, "state_groundtruth_estimate0", "imu0");
    }
    YL_INFO("Conversion done!");

    // 关闭Logger
    Logger::shutdown();

    return 0;
}

void convertImages(const std::unique_ptr<rosbag2_cpp::Writer> &writer, const std::string &folder,
                   const std::string &label) {
    // 初始化转换对象和话题
    cv_bridge::CvImage cv_image;
    cv_image.header.frame_id  = label;
    cv_image.encoding         = "mono8";
    const auto raw_label      = absl::StrCat(label, "/raw");
    const auto compress_label = absl::StrCat(label, "/compressed");

    // 读取数据并处理
    auto csv = std::ifstream(absl::StrCat(folder, "/", label, "/data.csv"));
    std::string line;
    csv.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // 跳过第一行（标题）
    while (std::getline(csv, line)) {
        const std::vector<absl::string_view> splits =
                absl::StrSplit(line, absl::ByAnyChar(", \t\r"), absl::SkipWhitespace());

        // 转换时间戳
        int64_t timestamp;
        if (!absl::SimpleAtoi(splits[0], &timestamp)) {
            YL_FATAL("Failed to convert timestamp: {}", splits[0]);
        }
        rclcpp::Time ros_timestamp(timestamp);
        cv_image.header.stamp = ros_timestamp;

        // 加载图像
        cv_image.image = cv::imread(absl::StrCat(folder, "/", label, "/data/", splits[1]), cv::IMREAD_GRAYSCALE);

        // 写入数据
        writer->write(*cv_image.toImageMsg(), raw_label, ros_timestamp);
        writer->write(*cv_image.toCompressedImageMsg(cv_bridge::Format::PNG), compress_label, ros_timestamp);
    }
}

void convertImu(const std::unique_ptr<rosbag2_cpp::Writer> &writer, const std::string &folder,
                const std::string &label) {
    // 初始化信息
    sensor_msgs::msg::Imu imu;
    imu.header.frame_id = label;

    // 读取数据并处理
    auto csv = std::ifstream(absl::StrCat(folder, "/", label, "/data.csv"));
    std::string line;
    csv.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // 跳过第一行（标题）
    while (std::getline(csv, line)) {
        const std::vector<absl::string_view> splits =
                absl::StrSplit(line, absl::ByAnyChar(", \t\r"), absl::SkipWhitespace());

        // 转换时间戳
        int64_t timestamp;
        if (!absl::SimpleAtoi(splits[0], &timestamp)) {
            YL_FATAL("Failed to convert timestamp: {}", splits[0]);
        }
        rclcpp::Time ros_timestamp(timestamp);
        imu.header.stamp = ros_timestamp;

        // 转换IMU数据
        auto &gyr = imu.angular_velocity;
        auto &acc = imu.linear_acceleration;
        if (!absl::SimpleAtod(splits[1], &gyr.x) || !absl::SimpleAtod(splits[2], &gyr.y) ||
            !absl::SimpleAtod(splits[3], &gyr.z) || !absl::SimpleAtod(splits[4], &acc.x) ||
            !absl::SimpleAtod(splits[5], &acc.y) || !absl::SimpleAtod(splits[6], &acc.z)) {
            YL_FATAL("Failed to convert IMU data: {}", line);
        }

        // 写入数据
        writer->write(imu, label, ros_timestamp);
    }
}

void convertGT(const std::unique_ptr<rosbag2_cpp::Writer> &writer, const std::string &folder, const std::string &label,
               const std::string &frame_id) {
    // 初始化信息
    tf2_msgs::msg::TFMessage tf_message;
    geometry_msgs::msg::TransformStamped stamped_transform;
    stamped_transform.header.frame_id = "gt";
    stamped_transform.child_frame_id  = frame_id;

    // 读取数据并处理
    auto csv = std::ifstream(absl::StrCat(folder, "/", label, "/data.csv"));
    std::string line;
    csv.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // 跳过第一行（标题）
    while (std::getline(csv, line)) {
        const std::vector<absl::string_view> splits =
                absl::StrSplit(line, absl::ByAnyChar(", \t\r"), absl::SkipWhitespace());

        // 转换时间戳
        int64_t timestamp;
        if (!absl::SimpleAtoi(splits[0], &timestamp)) {
            YL_FATAL("Failed to convert timestamp: {}", splits[0]);
        }
        rclcpp::Time ros_timestamp(timestamp);
        stamped_transform.header.stamp = ros_timestamp;

        // 转换位姿
        if (!absl::SimpleAtod(splits[1], &stamped_transform.transform.translation.x) ||
            !absl::SimpleAtod(splits[2], &stamped_transform.transform.translation.y) ||
            !absl::SimpleAtod(splits[3], &stamped_transform.transform.translation.z) ||
            !absl::SimpleAtod(splits[4], &stamped_transform.transform.rotation.w) ||
            !absl::SimpleAtod(splits[5], &stamped_transform.transform.rotation.x) ||
            !absl::SimpleAtod(splits[6], &stamped_transform.transform.rotation.y) ||
            !absl::SimpleAtod(splits[7], &stamped_transform.transform.rotation.z)) {
            YL_FATAL("Failed to convert pose: {}", line);
        }

        // 写入
        writer->write(stamped_transform, stamped_transform.header.frame_id, ros_timestamp);
        tf_message.transforms.push_back(stamped_transform);
        writer->write(tf_message, "tf", ros_timestamp);
        tf_message.transforms.clear();
    }
}