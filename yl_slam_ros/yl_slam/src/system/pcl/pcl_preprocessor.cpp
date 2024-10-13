#include "system/pcl/pcl_preprocessor.h"

namespace YL_SLAM {

PclPreprocessor::PclPreprocessor(std::vector<PclFilterBase::sPtr> filters) : filters_(std::move(filters)) {}

PclPreprocessor::uPtr PclPreprocessor::loadFromYaml(const YAML::Node &config) {
    const auto filters_node = config["filters"];
    YL_CHECK(filters_node.IsSequence(), "Filters node should be a sequence!");
    const auto num_filters = filters_node.size();

    std::vector<PclFilterBase::sPtr> filters;
    for (size_t filter_idx = 0; filter_idx < num_filters; ++filter_idx) {
        const auto filter_node = filters_node[filter_idx];
        YL_CHECK(filter_node && filter_node.IsMap(), "Unable to get filter node for filter #{}!", filter_idx);

        auto filter = PclFilterBase::loadFromYaml(filter_node["filter"]);
        filters.push_back(std::move(filter));
    }

    return std::make_unique<PclPreprocessor>(std::move(filters));
}

YAML::Node PclPreprocessor::writeToYaml() const {
    YAML::Node filters_node;
    for (const auto &filter: filters_) {
        YAML::Node filter_node;
        filter_node["filter"] = filter->writeToYaml();
        filters_node.push_back(filter_node);
    }

    YAML::Node node;
    node["filters"] = filters_node;
    return node;
}

RawLidarPointCloud::Ptr PclPreprocessor::process(const RawLidarPointCloud &point_cloud) {
    auto ret = point_cloud.makeShared();
    for (const auto &filter: filters_) {
        filter->process(ret);
    }
    return ret;
}

void PclPreprocessor::print(std::ostream &out) const {
    out << "PclPreprocessor: " << std::endl;
    for (size_t i = 0; i < filters_.size(); ++i) {
        out << "Filter #" << i << std::endl;
        filters_[i]->print(out);
    }
}

} // namespace YL_SLAM
