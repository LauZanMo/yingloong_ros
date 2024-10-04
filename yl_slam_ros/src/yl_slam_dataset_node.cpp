#include "common/concurrent_container_types.h"
#include "common/logger.h"
#include "common/path_helper.h"
#include "common/sensor/imu.h"
#include "common/timer.h"
#include "common/yaml/yaml_serialization.h"
#include "impl/drawer_rviz.h"
#include "lidar/lidar_helper.h"
#include "system/estimator.h"

#include <absl/strings/numbers.h>
#include <absl/strings/str_cat.h>
#include <regex>
#include <thread>

#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.h>

using namespace YL_SLAM;

/**
 * @brief 数据集读取器类
 */
class DatasetReader {
public:
    /**
     * @brief 构造函数
     * @param config 配置节点
     * @param estimator 估计器指针
     */
    explicit DatasetReader(const YAML::Node &config, Estimator::uPtr estimator) : estimator_(std::move(estimator)) {
        // 打开数据集
        const auto uri = YAML::get<std::string>(config, "uri");
        reader_        = std::make_unique<rosbag2_cpp::Reader>();
        reader_->open(uri);

        // 读取配置
        play_speed_        = YAML::get<double>(config, "play_speed");
        point_cloud_regex_ = std::regex("[/_[:alnum:]]*lidar([0-9]+)/point_cloud_raw");
        imu_regex_         = std::regex("[/_[:alnum:]]*imu([0-9]+)/data$");

        // 初始化缓冲区
        const auto slam_sensor_rate = YAML::get<long>(config, "slam_sensor_rate");
        const auto cache_duration   = YAML::get<long>(config, "cache_duration");
        const auto topics           = reader_->get_all_topics_and_types();
        for (const auto &topic: topics) {
            if (std::regex_match(topic.name, point_cloud_regex_)) {
                point_cloud_buffers_.emplace_back();
                point_cloud_buffers_.back().set_capacity(slam_sensor_rate * cache_duration);
            }
        }
        YL_CHECK(!point_cloud_buffers_.empty(), "Point cloud topics should not be empty!");
        lidar_types_.resize(point_cloud_buffers_.size());

        // 启动点云同步线程
        YL_INFO("Start reading dataset {}...", uri);
        running_     = true;
        sync_thread_ = std::make_unique<std::thread>(&DatasetReader::syncLoop, this);
    }

    /**
     * @brief 析构函数
     */
    ~DatasetReader() {
        // 等待缓存清空
        while (!point_cloud_buffers_[0].empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        // 结束同步线程
        running_ = false;
        for (auto &buffer: point_cloud_buffers_) {
            buffer.emplace(nullptr);
        }
        sync_thread_->join();

        YL_INFO("Finish reading dataset!");
    }

    /**
     * @brief 读取循环
     * @details 该函数会不断从数据集中读取数据包，对数据包进行对应解码，直到数据集末尾，其中：<br/>
     *          1. 点云会加入到缓冲区中，通过同步线程进行同步后，在进行使用<br/>
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
            if (std::regex_match(msg->topic_name, sub_match, point_cloud_regex_)) {
                size_t idx;
                if (absl::SimpleAtoi(sub_match[1].str(), &idx)) {
                    pushPointCloudMsg(idx, msg);
                } else {
                    YL_ERROR("Invalid point cloud index: {}", sub_match[1].str());
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
    /**
     * @brief 同步循环
     * @details 该函数会不断从多个点云缓冲区中读取点云，对不同缓冲区的点云进行同步，再进行使用，直到系统结束
     */
    void syncLoop() {
        int64_t ref_timestamp{-1};
        std::vector<RawLidarPointCloud::Ptr> point_cloud_bundle(point_cloud_buffers_.size());
        std::bitset<16> match_flag;
        RawLidarPointCloud::Ptr point_cloud;
        int64_t timestamp;

        // 若系统运行则持续同步点云
        while (running_) {
            match_flag.reset();
            for (size_t i = 0; i < point_cloud_buffers_.size(); ++i) {
                if (!match_flag[i]) {
                    // 读取缓冲直至数据时间戳大于等于当前时间戳
                    while (true) {
                        point_cloud_buffers_[i].pop(point_cloud);
                        timestamp = static_cast<int64_t>(point_cloud->header.stamp);

                        if (!running_)
                            return;

                        if (timestamp >= ref_timestamp) {
                            break;
                        }
                    }

                    // 若为当前点云束第一个匹配上时间戳的点云，则不做检查，否则需要检查时间戳是否匹配
                    if (match_flag.none()) {
                        point_cloud_bundle[i] = point_cloud;
                        match_flag.set(i);
                        ref_timestamp = timestamp;
                    } else {
                        point_cloud_bundle[i] = point_cloud;
                        match_flag.set(i);

                        // 检测到时间戳不匹配，则重置标志位，重新进行匹配
                        if (timestamp > ref_timestamp) {
                            YL_WARN("Point cloud [{}] time jump found at {}", i, ref_timestamp);
                            match_flag.reset();
                            match_flag.set(i);
                            ref_timestamp = timestamp;
                            i             = 0;
                        }
                    }
                }
            }

            YL_INFO("Consume point cloud bundle, timestamp: {}ns", ref_timestamp);
            estimator_->addPointCloudBundle(ref_timestamp, point_cloud_bundle);
        }
    }

    /**
     * @brief 对点云数据包进行解码，并加入对应索引的点云缓冲区
     * @param idx 点云缓冲区的索引
     * @param msg 点云数据包
     */
    void pushPointCloudMsg(size_t idx, const rosbag2_storage::SerializedBagMessageSharedPtr &msg) {
        YL_CHECK(idx < point_cloud_buffers_.size(), "Index should be less than buffers size");

        // 解码信息
        const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        const auto ros_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        point_cloud_serial_.deserialize_message(&serialized_msg, ros_msg.get());

        // 根据点云自动检测激光雷达类型
        if (lidar_types_[idx].empty()) {
            lidar_types_[idx] = detectLidarType(*ros_msg);
        }

        // 将信息转换为系统输入类型
        RawLidarPointCloud::Ptr point_cloud;
        if (lidar_types_[idx] == "ouster") {
            const auto os_point_cloud = std::make_shared<OusterLidarPointCloud>();
            pcl::fromROSMsg(*ros_msg, *os_point_cloud);
            point_cloud = lidar_helper::convert(*os_point_cloud);
        } else if (lidar_types_[idx] == "velodyne") {
            const auto vld_point_cloud = std::make_shared<VelodyneLidarPointCloud>();
            pcl::fromROSMsg(*ros_msg, *vld_point_cloud);
            point_cloud = lidar_helper::convert(*vld_point_cloud);
        } else if (lidar_types_[idx] == "livox") {
            const auto lv_point_cloud = std::make_shared<LivoxLidarPointCloud>();
            pcl::fromROSMsg(*ros_msg, *lv_point_cloud);
            point_cloud = lidar_helper::convert(*lv_point_cloud);
        } else {
            return;
        }

        // 将点云加入对应缓冲区，等待同步
        point_cloud_buffers_[idx].push(std::move(point_cloud));
    }

    /**
     * @brief 对IMU数据包进行解码，并直接使用
     * @param idx IMU对应的索引
     * @param msg IMU数据包
     * @todo 加入多IMU支持
     */
    void pushImuMsg([[maybe_unused]] size_t idx, const rosbag2_storage::SerializedBagMessageSharedPtr &msg) {
        YL_CHECK(idx == 0, "Index should be 0"); ///< 目前只支持单IMU

        // 解码信息
        const rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
        const auto ros_msg = std::make_shared<sensor_msgs::msg::Imu>();
        imu_serial_.deserialize_message(&serialized_msg, ros_msg.get());

        // 将信息转换为系统输入类型
        Imu imu;
        imu.timestamp = msg->time_stamp;
        imu.gyr       = {YL_FLOAT(ros_msg->angular_velocity.x), YL_FLOAT(ros_msg->angular_velocity.y),
                         YL_FLOAT(ros_msg->angular_velocity.z)};
        imu.acc       = {YL_FLOAT(ros_msg->linear_acceleration.x), YL_FLOAT(ros_msg->linear_acceleration.y),
                         YL_FLOAT(ros_msg->linear_acceleration.z)};

        // 输入系统
        estimator_->addImu(imu);
    }

    /**
     * @brief 根据点云信息检测激光雷达类型
     * @param msg 点云信息
     * @return 激光雷达类型
     */
    static std::string detectLidarType(const sensor_msgs::msg::PointCloud2 &msg) {
        for (const auto &field: msg.fields) {
            if (field.name == "t") {
                return "ouster";
            } else if (field.name == "time") {
                return "velodyne";
            } else if (field.name == "timestamp") {
                return "livox";
            }
        }
        YL_ERROR("Can not detect lidar type!");
        return {};
    }

    Estimator::uPtr estimator_;

    std::unique_ptr<rosbag2_cpp::Reader> reader_;
    std::regex point_cloud_regex_, imu_regex_;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> point_cloud_serial_;
    rclcpp::Serialization<sensor_msgs::msg::Imu> imu_serial_;

    std::unique_ptr<std::thread> sync_thread_;
    std::atomic<bool> running_{false};
    std::vector<conc_queue<RawLidarPointCloud::Ptr>> point_cloud_buffers_;
    std::vector<std::string> lidar_types_;

    double play_speed_;
};

int main(int argc, char **argv) {
    // 输入检查
    if (argc != 2) {
        std::cout << "Usage: ros2 run yl_slam_ros yl_slam_dataset_node <config_file>" << std::endl;
        return -1;
    }

    // 初始化Logger
    const auto log_path     = absl::StrCat(YL_SLAM_DIR, "/logs/", Timer::currentTime());
    const auto program_name = path_helper::getFileName(argv[0]);
    Logger::initialize(true, log_path, program_name);

    // 启动ROS节点
    rclcpp::init(argc, argv);
    rclcpp::Node node(program_name);

    // 加载配置
    const std::string file_name(path_helper::completePath(argv[1]));
    const auto config = YAML::load(file_name);

    { // 创建数据集读取器并进行读取
        DrawerBase::sPtr drawer   = std::make_shared<DrawerRviz>(config["drawer"], node);
        auto estimator            = std::make_unique<Estimator>(config, drawer);
        const auto dataset_reader = std::make_unique<DatasetReader>(config["dataset"], std::move(estimator));
        dataset_reader->readLoop();
    }

    // 关闭Logger
    Logger::shutdown();

    return 0;
}
