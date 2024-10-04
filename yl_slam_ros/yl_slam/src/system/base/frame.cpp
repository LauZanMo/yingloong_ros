#include "system/base/frame.h"

namespace YL_SLAM {

static std::atomic<long> frame_counter{0};

Frame::Frame(int64_t timestamp, const LidarGeometryBase::sPtr &lidar, const SE3f &T_bs, RawLidarPointCloud::Ptr raw_pcl)
    : timestamp_(timestamp), id_(frame_counter++), lidar_(lidar), T_bs_(T_bs), raw_pcl_(std::move(raw_pcl)) {}

int64_t Frame::timestamp() const {
    return timestamp_;
}

long Frame::id() const {
    return id_;
}

long Frame::bundleId() const {
    return bundle_id_;
}

void Frame::setBundleId(long bundle_id) {
    bundle_id_ = bundle_id;
}

const LidarGeometryBase::sConstPtr &Frame::lidar() const {
    return lidar_;
}

const SE3f &Frame::Twf() const {
    return T_wf_;
}

void Frame::setTwf(const SE3f &T_wf) {
    T_wf_ = T_wf;
}

const SE3f &Frame::Tbs() const {
    return T_bs_;
}

void Frame::setTbs(const SE3f &T_bs) {
    T_bs_ = T_bs;
}

RawLidarPointCloud::ConstPtr Frame::rawPointCloud() const {
    return raw_pcl_;
}

RawLidarPointCloud::Ptr &Frame::mutableRawPointCloud() {
    return raw_pcl_;
}

LidarPointCloud::ConstPtr Frame::pointCloud() const {
    return pcl_;
}

LidarPointCloud::Ptr &Frame::mutablePointCloud() {
    return pcl_;
}

} // namespace YL_SLAM
