#include "system/base/point.h"
#include "system/base/frame.h"

namespace YL_SLAM {

static std::atomic<long> point_counter{0};

Point::Point(Position pos, const FrameSPtr &seed_frame, size_t seed_idx, FloatType depth, Type type)
    : id_(point_counter++),
      pos_(std::move(pos)),
      seed_frame_(seed_frame),
      seed_idx_(seed_idx),
      depth_(depth),
      type_(type) {}

long Point::id() const {
    return id_;
}

const Position &Point::pos() const {
    return pos_;
}

void Point::setPos(const Position &pos) {
    pos_ = pos;
}

Frame::sConstPtr Point::seedFrame() const {
    return seed_frame_.lock();
}

Frame::sPtr Point::mutableSeedFrame() {
    return seed_frame_.lock();
}

size_t Point::seedIdx() const {
    return seed_idx_;
}

FloatType Point::depth() const {
    return depth_;
}

void Point::setDepth(FloatType depth) {
    depth_ = depth;
}

Point::Type Point::type() const {
    return type_;
}

void Point::setType(Point::Type type) {
    type_ = type;
}

void Point::addObservation(const Frame::sPtr &frame, size_t idx) {
    observations_.emplace_back(frame, idx);
}

size_t Point::numObservations() const {
    return observations_.size();
}

std::pair<Frame::sConstPtr, size_t> Point::observation(size_t idx) const {
    YL_CHECK(idx < observations_.size(), "Index should be less than the number of observation!");
    return {observations_[idx].first.lock(), observations_[idx].second};
}

} // namespace YL_SLAM
