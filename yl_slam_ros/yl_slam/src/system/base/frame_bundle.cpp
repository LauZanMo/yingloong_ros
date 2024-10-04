#include "system/base/frame_bundle.h"
#include "common/logger.h"

namespace YL_SLAM {

static std::atomic<long> bundle_counter{0};

FrameBundle::FrameBundle(const std::vector<Frame::sPtr> &frames) : bundle_id_(bundle_counter++), frames_(frames) {
    YL_CHECK(!frames.empty(), "FrameBundle should contain at least one frame!");
    for (const auto &frame: frames_) {
        YL_CHECK(timestamp() == frame->timestamp(), "All frames in FrameBundle should have the same timestamp!");
        frame->setBundleId(bundle_id_);
    }
}

int64_t FrameBundle::timestamp() const {
    return frames_.front()->timestamp();
}

long FrameBundle::bundleId() const {
    return bundle_id_;
}

const SE3f &FrameBundle::Twb() const {
    return T_wb_;
}

void FrameBundle::setTwb(const SE3f &T_wb) {
    T_wb_ = T_wb;
    for (const auto &frame: frames_) {
        frame->setTwf(T_wb * frame->Tbs());
    }
}

size_t FrameBundle::numFrames() const {
    return frames_.size();
}

Frame::sConstPtr FrameBundle::frame(size_t idx) const {
    YL_CHECK(idx < frames_.size(), "Index should be less than the number of frames!");
    return frames_[idx];
}

Frame::sPtr &FrameBundle::mutableFrame(size_t idx) {
    YL_CHECK(idx < frames_.size(), "Index should be less than the number of frames!");
    return frames_[idx];
}

} // namespace YL_SLAM
