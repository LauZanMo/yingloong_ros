#include "camera/camera_rig.h"
#include "camera/yaml/camera_rig_yaml_serialization.h"
#include "common/logger.h"
#include "common/yaml/yaml_serialization.h"

#include <absl/strings/str_cat.h>

namespace YL_SLAM {

CameraRig::CameraRig(std::string label, const std::vector<CameraGeometryBase::sPtr> &cameras, TbcVector T_bc_vec)
    : label_(std::move(label)), cameras_(cameras), T_bc_vec_(std::move(T_bc_vec)) {
    YL_CHECK(cameras_.size() == T_bc_vec_.size(), "Cameras size should be equal to T_bc_vec size!");
}

CameraRig::sPtr CameraRig::loadFromYaml(const std::string &config_file) {
    // 检查并转换路径（如果有需要）
    YL_CHECK(!config_file.empty(), "yaml_file should not be empty!");
    auto file_name = config_file;
    if (file_name.front() != '/') { // 相对路径
        file_name = absl::StrCat(YL_SLAM_DIR, "/", file_name);
    }

    // 根据配置文件加载相机
    const auto node = YAML::load(file_name);
    return YAML::get<CameraRig::sPtr>(node, "");
}

void CameraRig::writeToYaml(const std::string &config_file) const {
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

const std::string &CameraRig::label() const {
    return label_;
}

const CameraGeometryBase::sPtr &CameraRig::camera(size_t idx) const {
    YL_CHECK(idx < cameras_.size(), "Index should be less than cameras size!");
    return cameras_[idx];
}

size_t CameraRig::numCameras() const {
    return cameras_.size();
}

const SE3f &CameraRig::T_bc(size_t idx) const {
    YL_CHECK(idx < T_bc_vec_.size(), "Index should be less than T_bc_vec size!");
    return T_bc_vec_[idx];
}

void CameraRig::print(std::ostream &out) const {
    out << "Camera rig: " << label_ << std::endl;
    for (size_t i = 0; i < cameras_.size(); ++i) {
        out << "Camera #" << i << std::endl;
        cameras_[i]->print(out);
        out << "T_bc[" << i << "] = " << YL_MATRIX_FMT(T_bc_vec_[i].matrix()) << std::endl;
    }
}

} // namespace YL_SLAM
