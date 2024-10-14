#include "system/init/static/static_initializer.h"
#include "common/logger.h"
#include "common/rotation_helper.h"
#include "system/ins/ins_helper.h"

namespace YL_SLAM {

StaticInitializer::StaticInitializer(const VecXf &parameters, Vec3f g_w) : g_w_(std::move(g_w)) {
    YL_CHECK(parameters.size() == 3,
             "Parameters size should be 3! Order: [init_period, zero_gyr_thresh, zero_acc_thresh]");
    init_period_     = static_cast<int64_t>(parameters[0] * 1e9);
    zero_gyr_thresh_ = parameters[1];
    zero_acc_thresh_ = parameters[2];
}

bool StaticInitializer::initialize(const FrameBundle::sPtr &bundle, const Imus &imus) {
    YL_CHECK(!imus.empty(), "Imus should not be empty!");

    if (!initialized_) {
        // 追加IMU并检测零速
        long offset = imus_.empty() ? 0 : 1; // 上一imu容器尾部元素与当前imu容器头部元素相等，需要剔除
        imus_.insert(imus_.end(), imus.begin() + offset, imus.end());
        if (!ins_helper::detectZeroVelocity(imus_, zero_gyr_thresh_, zero_acc_thresh_)) {
            YL_WARN("Please keep static to initialize!");
            imus_.clear();
            return false;
        }

        if (imus_.back().timestamp - imus_.front().timestamp < init_period_) {
            return false;
        }

        // 计算平均值
        const auto size_inv = YL_FLOAT(1.0) / YL_FLOAT(imus.size());
        Vec3f ave_gyr(Vec3f::Zero()), ave_acc(Vec3f::Zero());
        for (const auto &imu: imus) {
            ave_gyr += imu.gyr;
            ave_acc += imu.acc;
        }
        ave_gyr *= size_inv;
        ave_acc *= size_inv;

        // 重力调平
        Vec3f att;
        att[0] = YL_FLOAT(atan2(-ave_acc[1], -ave_acc[2]));
        att[1] = YL_FLOAT(atan(ave_acc[0] / sqrt(ave_acc[1] * ave_acc[1] + ave_acc[2] * ave_acc[2])));
        att[2] = 0.0;

        // 构建初始状态
        const auto q_wb = rotation_helper::toSO3(att);
        NavState state(imus_.back().timestamp, {q_wb, Vec3f ::Zero()}, Vec3f::Zero(), ave_gyr,
                       ave_acc - q_wb.inverse() * g_w_);
        bundle->setState(state);
        initialized_ = true;
        YL_INFO("Static initialize done! State: {}", state);
    }

    return true;
}

void StaticInitializer::reset() {
    initialized_ = false;
    imus_.clear();
}

void StaticInitializer::print(std::ostream &out) const {
    out << "Static initializer:\n"
        << "  [zero_gyr_thresh, zero_acc_thresh] = [" << zero_gyr_thresh_ << ", " << zero_acc_thresh_ << "]\n"
        << "  init_period = " << static_cast<double>(init_period_) * 1e-9 << std::endl;
}

std::string StaticInitializer::type() const {
    return "static";
}

VecXf StaticInitializer::parameters() const {
    VecXf parameters(3);
    parameters[0] = YL_FLOAT(static_cast<double>(init_period_) * 1e-9);
    parameters[1] = zero_gyr_thresh_;
    parameters[2] = zero_acc_thresh_;
    return parameters;
}

} // namespace YL_SLAM
