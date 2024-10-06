#pragma once

#include "common/sensor/imu.h"

namespace YL_SLAM::ins_helper {

/**
 * @brief 检测是否处于零速
 * @param imus IMU数据容器
 * @param zero_gyr_thresh 陀螺仪零速阈值
 * @param zero_acc_thresh 加速度零速阈值
 * @return 是否处于零速
 */
bool detectZeroVelocity(const Imus &imus, FloatType zero_gyr_thresh, FloatType zero_acc_thresh);

} // namespace YL_SLAM::ins_helper
