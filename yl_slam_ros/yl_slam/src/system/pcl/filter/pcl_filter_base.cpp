#include "system/pcl/filter/pcl_filter_base.h"
#include "common/yaml/yaml_eigen_serialization.h"
#include "system/pcl/filter/body_filter.h"
#include "system/pcl/filter/voxel_filter.h"

namespace YL_SLAM {

PclFilterBase::sPtr PclFilterBase::loadFromYaml(const YAML::Node &config) {
    const auto type       = YAML::get<std::string>(config, "type");
    const auto parameters = YAML::get<VecXf>(config, "parameters");
    if (type == "voxel") {
        return std::make_shared<VoxelFilter>(parameters);
    } else if (type == "body") {
        return std::make_shared<BodyFilter>(parameters);
    } else {
        YL_FATAL("Invalid filter type: {}", type);
    }
}

YAML::Node PclFilterBase::writeToYaml() const {
    YAML::Node node;
    node["type"]       = type();
    node["parameters"] = parameters();
    return node;
}

} // namespace YL_SLAM
