#include "common/concurrent_container_types.h"
#include "common/logger.h"
#include "common/path_helper.h"
#include "common/sensor/imu.h"
#include "common/timer.h"
#include "common/yaml/yaml_serialization.h"

#include <absl/strings/numbers.h>
#include <absl/strings/str_cat.h>
#include <opencv2/core.hpp>
#include <regex>
#include <thread>

#include <cv_bridge/cv_bridge.h>
#include <rclcpp/serialization.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_transport/reader_writer_factory.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

using namespace YL_SLAM;

/**
 * @brief 数据集读取器类
 */
class DatasetReader {
public:
    /**
     * @brief 构造函数
     * @param config 配置节点
     */
    explicit DatasetReader(const YAML::Node &config) {
        // 打开数据集
        rosbag2_storage::StorageOptions storage_options;
        storage_options.uri = YAML::get<std::string>(config, "uri");
        reader_             = rosbag2_transport::ReaderWriterFactory::make_reader(storage_options);
        reader_->open(storage_options);

        // 读取配置
        play_speed_           = YAML::get<double>(config, "play_speed");
        use_compressed_image_ = YAML::get<bool>(config, "use_compressed_image");
        image_regex_          = use_compressed_image_ ? std::regex("[/_[:alnum:]]*cam([0-9]+)/compressed$")
                                                      : std::regex("[/_[:alnum:]]*cam([0-9]+)/raw$");
        imu_regex_            = std::regex("[/_[:alnum:]]*imu([0-9]+)$");

        // 初始化缓冲区
        const auto image_rate     = YAML::get<long>(config, "image_rate");
        const auto cache_duration = YAML::get<long>(config, "cache_duration");
        const auto topics         = reader_->get_all_topics_and_types();
        for (const auto &topic: topics) {
            if (std::regex_match(topic.name, image_regex_)) {
                stamped_image_buffers_.emplace_back();
                stamped_image_buffers_.back().set_capacity(image_rate * cache_duration);
            }
        }
        YL_CHECK(!stamped_image_buffers_.empty(), "Image topics should not be empty!");

        // 启动图像同步线程
        YL_INFO("Start reading dataset {}...", storage_options.uri);
        running_     = true;
        sync_thread_ = std::make_unique<std::thread>(&DatasetReader::syncLoop, this);
    }

    /**
     * @brief 析构函数
     */
    ~DatasetReader() {
        // 等待缓存清空
        while (!stamped_image_buffers_[0].empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // 结束同步线程
        running_ = false;
        for (auto &stamped_image_buffer: stamped_image_buffers_) {
            stamped_image_buffer.emplace(-1, cv::Mat());
        }
        sync_thread_->join();

        YL_INFO("Finish reading dataset!");
    }

    /**
     * @brief 读取循环
     * @details 该函数会不断从数据集中读取数据包，对数据包进行对应解码，直到数据集末尾，其中：<br/>
     *          1. 图像会加入到缓冲区中，通过同步线程进行同步后，在进行使用<br/>
     *          2. IMU数据会直接使用
     */
    void readLoop() {
        Timer timer;
        int64_t last_timestamp{-1};

        // 若未到达数据集末尾则读取数据包
        while (reader_->has_next()) {
            const auto msg = reader_->read_next();

            // 根据话题名进行对应解码
            std::smatch sub_match;
            if (std::regex_match(msg->topic_name, sub_match, image_regex_)) {
                size_t idx;
                if (absl::SimpleAtoi(sub_match[1].str(), &idx)) {
                    pushImageMsg(idx, msg);
                } else {
                    YL_ERROR("Invalid image index: {}", sub_match[1].str());
                }
            } else if (std::regex_match(msg->topic_name, sub_match, imu_regex_)) {
                size_t idx;
                if (absl::SimpleAtoi(sub_match[1].str(), &idx)) {
                    pushImuMsg(idx, msg);
                } else {
                    YL_ERROR("Invalid imu index: {}", sub_match[1].str());
                }
            } else {
                continue;
            }

            // 控制播放倍速（通过休眠实现）
            if (last_timestamp >= 0) {
                const auto dt = msg->time_stamp - last_timestamp;
                if (dt > 0) {
                    const auto sleep_time =
                            static_cast<int64_t>(static_cast<double>(dt) / play_speed_) - timer.costInNsec();
                    if (sleep_time > 0) {
                        std::this_thread::sleep_for(std::chrono::nanoseconds(sleep_time));
                    } else {
                        YL_DEBUG("Data delay {} ns to play", -sleep_time);
                    }
                }
            }
            timer.restart();
            last_timestamp = msg->time_stamp;
        }
    }

private:
    using StampedImage = std::pair<int64_t, cv::Mat>;

    /**
     * @brief 同步循环
     * @details 该函数会不断从多个图像缓冲区中读取图像，对不同缓冲区的图像进行同步，再进行使用，直到系统结束
     */
    void syncLoop() {
        int64_t ref_timestamp{-1};
        std::vector<cv::Mat> image_bundle(stamped_image_buffers_.size());
        std::bitset<8> match_flag;
        StampedImage stamped_image;
        auto &[timestamp, image] = stamped_image;

        // 若系统运行则持续同步图像
        while (running_) {
            match_flag.reset();
            for (size_t i = 0; i < stamped_image_buffers_.size(); ++i) {
                if (!match_flag[i]) {
                    // 读取缓冲直至数据时间戳大于等于当前时间戳
                    while (stamped_image_buffers_[i].pop(stamped_image)) {
                        if (!running_)
                            return;

                        if (timestamp >= ref_timestamp) {
                            break;
                        }
                    }

                    // 若为当前图像束第一个匹配上时间戳的图像，则不做检查，否则需要检查时间戳是否匹配
                    if (match_flag.none()) {
                        image_bundle[i] = image;
                        match_flag.set(i);
                        ref_timestamp = timestamp;
                    } else {
                        image_bundle[i] = image;
                        match_flag.set(i);

                        // 检测到时间戳不匹配，则重置标志位，重新进行匹配
                        if (timestamp > ref_timestamp) {
                            YL_WARN("Image [{}] time jump found at {}", i, ref_timestamp);
                            match_flag.reset();
                            match_flag.set(i);
                            ref_timestamp = timestamp;
                            i             = 0;
                        }
                    }
                }
            }

            YL_INFO("Consume image bundle, timestamp: {}", ref_timestamp);
        }
    }

    /**
     * @brief 对图像数据包进行解码，并加入对应索引的图像缓冲区
     * @param idx 图像缓冲区的索引
     * @param msg 图像数据包
     */
    void pushImageMsg(size_t idx, const rosbag2_storage::SerializedBagMessageSharedPtr &msg) {
        YL_CHECK(idx < stamped_image_buffers_.size(), "Index should be less than buffers size");

        const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        if (use_compressed_image_) {
            const auto ros_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
            compressed_image_serial_.deserialize_message(&serialized_msg, ros_msg.get());
            stamped_image_buffers_[idx].push({msg->time_stamp, cv_bridge::toCvCopy(ros_msg)->image});
        } else {
            const auto ros_msg = std::make_shared<sensor_msgs::msg::Image>();
            image_serial_.deserialize_message(&serialized_msg, ros_msg.get());
            stamped_image_buffers_[idx].push({msg->time_stamp, cv_bridge::toCvCopy(ros_msg)->image});
        }
    }

    /**
     * @brief 对IMU数据包进行解码，并直接使用
     * @param idx IMU对应的索引
     * @param msg IMU数据包
     * @todo 加入多IMU支持
     */
    void pushImuMsg([[maybe_unused]] size_t idx, const rosbag2_storage::SerializedBagMessageSharedPtr &msg) {
        YL_CHECK(idx == 0, "Index should be 0"); ///< 目前只支持单IMU

        const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        const auto ros_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serial_.deserialize_message(&serialized_msg, ros_msg.get());
        Imu imu;
        imu.timestamp = msg->time_stamp;
        imu.gyr       = {YL_FLOAT(ros_msg->angular_velocity.x), YL_FLOAT(ros_msg->angular_velocity.y),
                         YL_FLOAT(ros_msg->angular_velocity.z)};
        imu.acc       = {YL_FLOAT(ros_msg->linear_acceleration.x), YL_FLOAT(ros_msg->linear_acceleration.y),
                         YL_FLOAT(ros_msg->linear_acceleration.z)};
        // YL_INFO("Consume imu: {}", imu);
    }

    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    std::regex image_regex_, imu_regex_;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> compressed_image_serial_;
    rclcpp::Serialization<sensor_msgs::msg::Image> image_serial_;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serial_;

    std::unique_ptr<std::thread> sync_thread_;
    std::atomic<bool> running_{false};
    std::vector<conc_queue<StampedImage>> stamped_image_buffers_;

    bool use_compressed_image_;
    double play_speed_;
};

int main(int argc, char **argv) {
    // 输入检查
    if (argc != 2) {
        YL_FATAL("Usage: ros2 run yl_slam_ros yl_slam_dataset <config_file>");
    }

    // 初始化Logger
    const auto log_path = absl::StrCat(YL_SLAM_DIR, "/logs/", Timer::currentTime());
    Logger::initialize(true, log_path, argv[0]);

    // 加载配置
    const std::string file_name(path_helper::completePath(argv[1]));
    const auto config = YAML::load(file_name);

    { // 创建数据集读取器并进行读取
        auto dataset_reader = std::make_unique<DatasetReader>(config["dataset"]);
        dataset_reader->readLoop();
    }

    // 关闭Logger
    Logger::shutdown();

    return 0;
}
