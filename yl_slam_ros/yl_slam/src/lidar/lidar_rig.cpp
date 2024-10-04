#include "lidar/lidar_rig.h"
#include "common/logger.h"
#include "common/path_helper.h"
#include "common/yaml/yaml_serialization.h"
#include "lidar/yaml/lidar_rig_yaml_serialization.h"

namespace YL_SLAM {

LidarRig::LidarRig(std::string label, const std::vector<LidarGeometryBase::sPtr> &lidars, LidarRig::TbsVector T_bs_vec)
    : label_(std::move(label)), lidars_(lidars), T_bs_vec_(std::move(T_bs_vec)) {
    YL_CHECK(lidars.size() == T_bs_vec_.size(), "Lidars size should be equal to T_bs_vec size!");
}

LidarRig::sPtr LidarRig::loadFromYaml(const std::string &config_file) {
    // 检查并转换路径（如果有需要）
    YL_CHECK(!config_file.empty(), "yaml_file should not be empty!");

    // 根据配置文件加载相机
    const auto node = YAML::load(path_helper::completePath(config_file));
    return YAML::get<LidarRig::sPtr>(node, "");
}

void LidarRig::writeToYaml(const std::string &config_file) const {
    // 检查并转换路径（如果有需要）
    YL_CHECK(!config_file.empty(), "config_file should not be empty!");

    // 将相机写入配置文件
    YAML::Node node;
    node = *this;
    YAML::dump(node, path_helper::completePath(config_file));
}

const std::string &LidarRig::label() const {
    return label_;
}

const LidarGeometryBase::sPtr &LidarRig::lidar(size_t idx) const {
    YL_CHECK(idx < lidars_.size(), "Index should be less than lidars size!");
    return lidars_[idx];
}

size_t LidarRig::numLidars() const {
    return lidars_.size();
}

const SE3f &LidarRig::T_bs(size_t idx) const {
    YL_CHECK(idx < T_bs_vec_.size(), "Index should be less than T_bs_vec size!");
    return T_bs_vec_[idx];
}

void LidarRig::print(std::ostream &out) const {
    out << "Lidar rig: " << label_ << std::endl;
    for (size_t i = 0; i < lidars_.size(); ++i) {
        out << "Lidar #" << i << std::endl;
        lidars_[i]->print(out);
        out << "T_bs[" << i << "] = " << YL_MATRIX_FMT(T_bs_vec_[i].matrix()) << std::endl;
    }
}

} // namespace YL_SLAM
