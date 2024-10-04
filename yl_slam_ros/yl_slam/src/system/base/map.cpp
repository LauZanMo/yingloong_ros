#include "system/base/map.h"
#include "common/logger.h"

namespace YL_SLAM {

void Map::reset() {
    keyframe_bundles_.clear();
    keyframes_.clear();
}

void Map::addKeyframeBundle(const FrameBundle::sPtr &bundle) {
    YL_CHECK(keyframe_bundles_.empty() || keyframe_bundles_.back()->bundleId() < bundle->bundleId(),
             "Keyframe bundle id should be incremental!");
    for (size_t i = 0; i < bundle->numFrames(); ++i) {
        auto &frame = bundle->mutableFrame(i);
        keyframes_.insert_or_assign(frame->id(), frame);
    }
    keyframe_bundles_.push_back(bundle);
}

void Map::removeKeyframeBundle(size_t idx) {
    YL_CHECK(idx < keyframe_bundles_.size(), "Index should be less than the number of keyframe bundles!");
    const auto &keyframe_bundle = keyframe_bundles_[idx];
    for (size_t i = 0; i < keyframe_bundle->numFrames(); ++i) {
        keyframes_.erase(keyframe_bundle->frame(i)->id());
    }
    if (idx == 0) {
        keyframe_bundles_.pop_front();
    } else {
        keyframe_bundles_.erase(keyframe_bundles_.begin() + static_cast<long>(idx));
    }
}

size_t Map::numKeyframeBundles() const {
    return keyframe_bundles_.size();
}

FrameBundle::sConstPtr Map::keyframeBundle(size_t idx) const {
    YL_CHECK(idx < keyframe_bundles_.size(), "Index should be less than the number of keyframe bundles!");
    return keyframe_bundles_[idx];
}

FrameBundle::sPtr &Map::mutableKeyframeBundle(size_t idx) {
    YL_CHECK(idx < keyframe_bundles_.size(), "Index should be less than the number of keyframe bundles!");
    return keyframe_bundles_[idx];
}

} // namespace YL_SLAM
