#pragma once

#include "common/setup.h"

#include <Eigen/Core>
#include <sophus/se3.hpp>

namespace YL_SLAM {

using Vec2f = Eigen::Matrix<FloatType, 2, 1>;
using Vec3f = Eigen::Matrix<FloatType, 3, 1>;
using Vec4f = Eigen::Matrix<FloatType, 4, 1>;
using Vec6f = Eigen::Matrix<FloatType, 6, 1>;
using Vec7f = Eigen::Matrix<FloatType, 7, 1>;
using VecXf = Eigen::Matrix<FloatType, Eigen::Dynamic, 1>;

using Mat22f = Eigen::Matrix<FloatType, 2, 2>;
using Mat23f = Eigen::Matrix<FloatType, 2, 3>;
using Mat32f = Eigen::Matrix<FloatType, 3, 2>;
using Mat33f = Eigen::Matrix<FloatType, 3, 3>;
using Mat44f = Eigen::Matrix<FloatType, 4, 4>;
using Mat2Xf = Eigen::Matrix<FloatType, 2, Eigen::Dynamic>;
using Mat3Xf = Eigen::Matrix<FloatType, 3, Eigen::Dynamic>;
using Mat4Xf = Eigen::Matrix<FloatType, 4, Eigen::Dynamic>;
using MatXf  = Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>;

using Diag2f = Eigen::DiagonalMatrix<FloatType, 2>;

using Quatf = Eigen::Quaternion<FloatType>;
using SO3f  = Sophus::SO3<FloatType>;
using SE3f  = Sophus::SE3<FloatType>;

} // namespace YL_SLAM
