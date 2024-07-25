#include "camera/distortion/no_distortion.h"

namespace YL_SLAM {

Vec2f NoDistortion::distort(const Vec2f &uv) {
    return uv;
}

Vec2f NoDistortion::undistort(const Vec2f &uv) {
    return uv;
}

Mat22f NoDistortion::jacobian(const Vec2f & /*uv*/) {
    return Mat22f::Identity();
}

void NoDistortion::print(std::ostream &out) {
    out << "  Distortion: None" << std::endl;
}

VecXf NoDistortion::distortionParameters() {
    return {};
}

} // namespace YL_SLAM
