#include "system/pcl/filter/voxel_filter.h"

namespace YL_SLAM {

VoxelFilter::VoxelFilter(const VecXf &parameters) {
    YL_CHECK(parameters.size() == 1, "Parameters size should be 1! Order: [leaf_size]");
    leaf_size_ = static_cast<float>(parameters[0]);

    voxel_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
}

void VoxelFilter::process(const RawLidarPointCloud::Ptr &point_cloud) {
    voxel_filter_.setInputCloud(point_cloud);
    voxel_filter_.filter(*point_cloud);
}

void VoxelFilter::print(std::ostream &out) const {
    out << "Voxel filter:\n"
        << "  leaf_size = " << leaf_size_ << std::endl;
}

std::string VoxelFilter::type() const {
    return "voxel";
}

VecXf VoxelFilter::parameters() const {
    VecXf parameters(1);
    parameters[0] = YL_FLOAT(leaf_size_);
    return parameters;
}

} // namespace YL_SLAM
