#include "common/nav_state.h"
#include "common/logger.h"

namespace YL_SLAM {

NavState::NavState() : timestamp(-1), T(Mat44f::Identity()), vel(Vec3f::Zero()), bg(Vec3f::Zero()), ba(Vec3f::Zero()) {}

NavState::NavState(int64_t timestamp, const SE3f &T, Vec3f vel, Vec3f bg, Vec3f ba)
    : timestamp(timestamp), T(T), vel(std::move(vel)), bg(std::move(bg)), ba(std::move(ba)) {}

NavState::operator bool() const {
    return timestamp >= 0;
}

std::ostream &operator<<(std::ostream &out, const NavState &state) {
    return out << "timestamp: " << state.timestamp << " T: " << YL_MATRIX_FMT(state.T.params().transpose())
               << " vel: " << YL_MATRIX_FMT(state.vel.transpose()) << " bg: " << YL_MATRIX_FMT(state.bg.transpose())
               << " ba: " << YL_MATRIX_FMT(state.ba.transpose());
}

} // namespace YL_SLAM
