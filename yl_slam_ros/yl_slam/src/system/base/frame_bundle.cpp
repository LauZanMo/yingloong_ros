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

const NavState &FrameBundle::state() const {
    YL_CHECK(state_, "State should be initialized before access!");
    return state_;
}

void FrameBundle::setState(const NavState &state) {
    YL_CHECK(!state_ || timestamp() == state.timestamp,
             "State timestamp should be equal to the frame bundle timestamp!");
    state_ = state;
    for (const auto &frame: frames_) {
        frame->setTwf(state_.T * frame->Tbs());
    }
}

const SE3f &FrameBundle::Twb() const {
    YL_CHECK(state_, "State should be initialized before access!");
    return state_.T;
}

void FrameBundle::setTwb(const SE3f &T_wb) {
    state_.T = T_wb;
    for (const auto &frame: frames_) {
        frame->setTwf(T_wb * frame->Tbs());
    }
}

void FrameBundle::setTbs(const std::vector<SE3f> &T_bs_vec) {
    YL_CHECK(T_bs_vec.size() == frames_.size(), "T_bs_vec size should be equal to the number of frames!");
    for (size_t i = 0; i < T_bs_vec.size(); ++i) {
        frames_[i]->setTbs(T_bs_vec[i]);
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
