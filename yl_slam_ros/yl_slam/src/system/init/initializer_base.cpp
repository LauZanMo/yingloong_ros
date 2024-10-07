#include "system/init/initializer_base.h"
#include "common/yaml/yaml_eigen_serialization.h"
#include "system/init/dynamic/dynamic_initializer.h"
#include "system/init/static/static_initializer.h"

namespace YL_SLAM {

InitializerBase::uPtr InitializerBase::loadFromYaml(const YAML::Node &config, const Vec3f &g_w) {
    const auto type       = YAML::get<std::string>(config, "type");
    const auto parameters = YAML::get<VecXf>(config, "parameters");
    if (type == "static") {
        return std::make_unique<StaticInitializer>(parameters, g_w);
    } else if (type == "dynamic") {
        return std::make_unique<DynamicInitializer>(parameters, g_w);
    } else {
        YL_ERROR("Invalid initializer type: {}", type);
        return {};
    }
}

bool InitializerBase::isInitialized() const {
    return initialized_;
}

} // namespace YL_SLAM
