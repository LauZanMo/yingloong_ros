#include "system/ins/ins_helper.h"
#include "common/logger.h"

namespace YL_SLAM::ins_helper {

bool detectZeroVelocity(const Imus &imus, FloatType zero_gyr_thresh, FloatType zero_acc_thresh) {
    const auto size_inv = YL_FLOAT(1.0) / YL_FLOAT(imus.size());

    // 计算平均值
    Vec3f ave_gyr(Vec3f::Zero()), ave_acc(Vec3f::Zero());
    for (const auto &imu: imus) {
        ave_gyr += imu.gyr;
        ave_acc += imu.acc;
    }
    ave_gyr *= size_inv;
    ave_acc *= size_inv;

    // 计算标准差
    Vec3f std_gyr(Vec3f::Zero()), std_acc(Vec3f::Zero());
    for (const auto &imu: imus) {
        std_gyr += (imu.gyr - ave_gyr).cwiseAbs2();
        std_acc += (imu.acc - ave_acc).cwiseAbs2();
    }
    std_gyr = (std_gyr * size_inv).cwiseSqrt();
    std_acc = (std_acc * size_inv).cwiseSqrt();

    YL_TRACE("Zero velocity detector: \n  Gyroscope std: {} \n  Accelerator std: {}",
             YL_MATRIX_FMT(std_gyr.transpose()), YL_MATRIX_FMT(std_acc.transpose()));

    return (std_gyr.array() < zero_gyr_thresh).all() && (std_acc.array() < zero_acc_thresh).all();
}

} // namespace YL_SLAM::ins_helper
