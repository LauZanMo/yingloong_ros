#include "common/sensor/imu.h"
#include "common/logger.h"

namespace YL_SLAM {

Imu::Imu() : timestamp(-1), gyr(Vec3f::Zero()), acc(Vec3f::Zero()) {}

Imu::Imu(int64_t timestamp, Vec3f gyr, Vec3f acc) : timestamp(timestamp), gyr(std::move(gyr)), acc(std::move(acc)) {}

Imu::operator bool() const {
    return timestamp >= 0;
}

std::ostream &operator<<(std::ostream &out, const Imu &imu) {
    return out << "timestamp: " << imu.timestamp << " gyr: " << imu.gyr.transpose() << " acc: " << imu.acc.transpose();
}

Imu Imu::interpolate(const Imu &imu0, const Imu &imu1, int64_t timestamp) {
    // 边界检查
    YL_CHECK(imu0.timestamp <= timestamp && timestamp <= imu1.timestamp,
             "Interpolation timestamp should be within [imu0.timestamp, imu1.timestamp]!");

    // 线性插值
    Imu imu;
    const auto ratio = YL_FLOAT(timestamp - imu0.timestamp) / YL_FLOAT(imu1.timestamp - imu0.timestamp);
    imu.timestamp    = timestamp;
    imu.gyr          = imu0.gyr + ratio * (imu1.gyr - imu0.gyr);
    imu.acc          = imu0.acc + ratio * (imu1.acc - imu0.acc);
    return imu;
}

} // namespace YL_SLAM
