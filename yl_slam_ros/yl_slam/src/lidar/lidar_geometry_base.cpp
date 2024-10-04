#include "lidar/lidar_geometry_base.h"
#include "common/logger.h"
#include "common/path_helper.h"
#include "common/yaml/yaml_serialization.h"
#include "lidar/yaml/lidar_yaml_serialization.h"

namespace YL_SLAM {

LidarGeometryBase::LidarGeometryBase(std::string label, uint32_t scan_line, FloatType nearest_distance,
                                     FloatType farthest_distance)
    : label_(std::move(label)),
      scan_line_(scan_line),
      nearest_dist_(nearest_distance),
      nearest_dist2_(nearest_distance * nearest_distance),
      farthest_dist_(farthest_distance),
      farthest_dist2_(farthest_distance * farthest_distance) {}

LidarGeometryBase::sPtr LidarGeometryBase::loadFromYaml(const std::string &config_file) {
    // 检查并转换路径（如果有需要）
    YL_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 根据配置文件加载激光雷达
    const auto node = YAML::load(path_helper::completePath(config_file));
    return YAML::get<LidarGeometryBase::sPtr>(node, "");
}

void LidarGeometryBase::writeToYaml(const std::string &config_file) const {
    // 检查并转换路径（如果有需要）
    YL_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 将激光雷达写入配置文件
    YAML::Node node;
    node = *this;
    YAML::dump(node, path_helper::completePath(config_file));
}

int LidarGeometryBase::id() const {
    return id_;
}

void LidarGeometryBase::setId(int id) {
    id_ = id;
}
const std::string &LidarGeometryBase::label() const {
    return label_;
}

uint32_t LidarGeometryBase::scanLine() const {
    return scan_line_;
}

FloatType LidarGeometryBase::nearestDistance() const {
    return nearest_dist_;
}

FloatType LidarGeometryBase::farthestDistance() const {
    return farthest_dist_;
}

void LidarGeometryBase::print(std::ostream &out) const {
    out << label_ << std::endl
        << "  name = " << label_ << std::endl
        << "  scan line = " << scan_line_ << std::endl
        << "  nearest distance = " << nearest_dist_ << std::endl
        << "  farthest distance = " << farthest_dist_ << std::endl;
}

} // namespace YL_SLAM
