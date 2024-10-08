#include "camera/camera_rig.h"
#include "camera/yaml/camera_rig_yaml_serialization.h"
#include "common/logger.h"
#include "common/path_helper.h"
#include "common/yaml/yaml_serialization.h"

namespace YL_SLAM {

CameraRig::CameraRig(std::string label, const std::vector<CameraGeometryBase::sPtr> &cameras, TbsVector T_bs_vec)
    : label_(std::move(label)), cameras_(cameras), T_bs_vec_(std::move(T_bs_vec)) {
    YL_CHECK(cameras_.size() == T_bs_vec_.size(), "Cameras size should be equal to T_bs_vec size!");
}

CameraRig::sPtr CameraRig::loadFromYaml(const std::string &config_file) {
    // 检查并转换路径（如果有需要）
    YL_CHECK(!config_file.empty(), "yaml_file should not be empty!");

    // 根据配置文件加载相机
    const auto node = YAML::load(path_helper::completePath(config_file));
    return YAML::get<CameraRig::sPtr>(node, "");
}

void CameraRig::writeToYaml(const std::string &config_file) const {
    // 检查并转换路径（如果有需要）
    YL_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 将相机写入配置文件
    YAML::Node node;
    node = *this;
    YAML::dump(node, path_helper::completePath(config_file));
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

const SE3f &CameraRig::T_bs(size_t idx) const {
    YL_CHECK(idx < T_bs_vec_.size(), "Index should be less than T_bs_vec size!");
    return T_bs_vec_[idx];
}

void CameraRig::print(std::ostream &out) const {
    out << "Camera rig: " << label_ << std::endl;
    for (size_t i = 0; i < cameras_.size(); ++i) {
        out << "Camera #" << i << std::endl;
        cameras_[i]->print(out);
        out << "T_bs[" << i << "] = " << YL_MATRIX_FMT(T_bs_vec_[i].matrix()) << std::endl;
    }
}

} // namespace YL_SLAM
