#include "system/ins/ins_navigator.h"

#include "common/logger.h"
#include <utility>

namespace YL_SLAM {

InsNavigator::InsNavigator(Vec3f g_w) : g_w_(std::move(g_w)) {}

void InsNavigator::propagate(const Imus &imus) {
    YL_CHECK(state_, "State should be initialized first!");
    YL_CHECK(imus.size() > 1, "Input IMUs size should be greater than 1!");
    YL_CHECK(state_.timestamp == imus[0].timestamp,
             "First IMU timestamp should be aligned with current state timestamp!");

    for (size_t i = 1; i < imus.size(); ++i) {
        const auto &imu0 = imus[i - 1];
        const auto &imu1 = imus[i];
        const auto dt    = YL_FLOAT(imu1.timestamp - imu0.timestamp) * 1e-9;

        Vec3f mid_gyr = 0.5 * (imu0.gyr + imu1.gyr) - state_.bg;
        const SO3f q0 = state_.T.so3();
        state_.T.so3() *= SO3f::exp(mid_gyr * dt);
        state_.T.normalize();

        Vec3f mid_acc = 0.5 * (q0 * imu0.acc + state_.T.so3() * imu1.acc) + g_w_;
        state_.T.translation() += state_.vel * dt + 0.5 * mid_acc * dt * dt;
        state_.vel += mid_acc * dt;
    }
}

const NavState &InsNavigator::state() const {
    return state_;
}

const SE3f &InsNavigator::Twb() const {
    return state_.T;
}

void InsNavigator::update(const NavState &state) {
    state_ = state;
}

void InsNavigator::update(const SE3f &T_wb) {
    state_.T = T_wb;
}

} // namespace YL_SLAM
