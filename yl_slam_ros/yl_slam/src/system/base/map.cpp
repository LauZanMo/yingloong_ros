#include "system/base/map.h"
#include "common/logger.h"

namespace YL_SLAM {

Map::Map(size_t window_size) : window_size_(window_size) {}

void Map::reset() {
    keyframe_bundles_.clear();
    keyframes_.clear();
}

bool Map::isFull() const {
    return keyframe_bundles_.size() >= window_size_;
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

void Map::addTrashPoint(const Point::sConstPtr &point) {
    YL_CHECK(point != nullptr, "Point should not be nullptr!");
    trash_points_.push_back(point);
}

void Map::removeTrashPoints() {
    for (const auto &point: trash_points_) {
        const auto frame_id = point->seedFrame()->id();
        const auto idx      = point->seedIdx();
        YL_CHECK(keyframes_.find(frame_id) != keyframes_.end(),
                 "The seed frame of the trash point is not found in map!");
        keyframes_[frame_id]->removeSeed(idx);
    }
    trash_points_.clear();
}

} // namespace YL_SLAM
