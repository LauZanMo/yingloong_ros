#include "system/pcl/filter/body_filter.h"

namespace YL_SLAM {

BodyFilter::BodyFilter(const VecXf &parameters) {
    YL_CHECK(parameters.size() == 1, "Parameters size should be 1! Order: [crop_size]");
    crop_size_ = static_cast<float>(parameters[0]);

    crop_box_.setNegative(true);
    crop_box_.setMin(Eigen::Vector4f(-crop_size_, -crop_size_, -crop_size_, 1.0f));
    crop_box_.setMax(Eigen::Vector4f(crop_size_, crop_size_, crop_size_, 1.0f));
}

void BodyFilter::process(const RawLidarPointCloud::Ptr &point_cloud) {
    crop_box_.setInputCloud(point_cloud);
    crop_box_.filter(*point_cloud);
}

void BodyFilter::print(std::ostream &out) const {
    out << "Body filter:\n"
        << "  crop_size = " << crop_size_ << std::endl;
}

std::string BodyFilter::type() const {
    return "body";
}

VecXf BodyFilter::parameters() const {
    VecXf parameters(1);
    parameters[0] = YL_FLOAT(crop_size_);
    return parameters;
}

} // namespace YL_SLAM
