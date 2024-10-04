#include "lidar/yaml/lidar_yaml_serialization.h"
#include "common/logger.h"
#include "common/yaml/yaml_serialization.h"

using namespace YL_SLAM;

namespace YAML {

Node convert<LidarGeometryBase>::encode(const LidarGeometryBase &lidar) {
    Node node;
    node["label"]             = lidar.label();
    node["scan_line"]         = lidar.scanLine();
    node["nearest_distance"]  = lidar.nearestDistance();
    node["farthest_distance"] = lidar.farthestDistance();

    return node;
}

bool convert<LidarGeometryBase>::decode(const Node & /*node*/, LidarGeometryBase & /*lidar*/) {
    YL_ERROR("Unsupported action: Directly decode with LidarGeometryBase object, try to decode with "
             "LidarGeometryBase::sPtr!");
    return false;
}

Node convert<LidarGeometryBase::sPtr>::encode(const LidarGeometryBase::sPtr &lidar) {
    YL_CHECK(lidar != nullptr, "The lidar is nullptr!");
    return convert<LidarGeometryBase>::encode(*lidar);
}

bool convert<LidarGeometryBase::sPtr>::decode(const Node &node, LidarGeometryBase::sPtr &lidar) {
    YL_CHECK(node.IsMap(), "Unable to parse the lidar because the node is not a map!");

    // 加载常规参数
    const auto label             = YAML::get<std::string>(node, "label");
    const auto scan_line         = YAML::get<uint32_t>(node, "scan_line");
    const auto nearest_distance  = YAML::get<FloatType>(node, "nearest_distance");
    const auto farthest_distance = YAML::get<FloatType>(node, "farthest_distance");

    // 初始化
    lidar = std::make_shared<LidarGeometryBase>(label, scan_line, nearest_distance, farthest_distance);
    return true;
}

} // namespace YAML
