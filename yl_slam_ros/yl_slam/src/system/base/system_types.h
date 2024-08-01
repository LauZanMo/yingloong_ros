#pragma once

#include "common/eigen_types.h"

#include <opencv2/core.hpp>
#include <vector>

namespace YL_SLAM {

using ImagePyr = std::vector<cv::Mat>;

using Keypoint   = Vec2f;
using Bearing    = Vec3f;
using Position   = Vec3f;
using Gradient   = Vec2f;
using SeedState  = Vec4f;
using Keypoints  = Mat2Xf;
using Bearings   = Mat3Xf;
using Positions  = Mat3Xf;
using Gradients  = Mat2Xf;
using SeedStates = Mat4Xf;

} // namespace YL_SLAM
