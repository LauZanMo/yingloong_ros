#include "camera/camera_geometry_base.h"
#include "camera/yaml/camera_yaml_serialization.h"
#include "common/yaml/yaml_serialization.h"

#include <absl/strings/str_cat.h>

namespace YL_SLAM {

CameraGeometryBase::sPtr CameraGeometryBase::loadFromYaml(const std::string &config_file) {
    // 检查并转换路径（如果有需要）
    YL_CHECK(!config_file.empty(), "config_file should not be empty!");
    auto file_name = config_file;
    if (file_name.front() != '/') { // 相对路径
        file_name = absl::StrCat(YL_SLAM_DIR, "/", file_name);
    }

    // 根据配置文件加载相机
    const auto node = YAML::load(file_name);
    return YAML::get<CameraGeometryBase::sPtr>(node, "");
}

void CameraGeometryBase::writeToYaml(const std::string &config_file) const {
    // 检查并转换路径（如果有需要）
    YL_CHECK(!config_file.empty(), "config_file should not be empty!");
    auto file_name = config_file;
    if (file_name.front() != '/') { // 相对路径
        file_name = absl::StrCat(YL_SLAM_DIR, "/", file_name);
    }

    // 将相机写入配置文件
    YAML::Node node;
    node = *this;
    YAML::dump(node, file_name);
}

} // namespace YL_SLAM
