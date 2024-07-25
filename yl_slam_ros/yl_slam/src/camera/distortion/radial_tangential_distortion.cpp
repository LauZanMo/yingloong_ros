#include "camera/distortion/radial_tangential_distortion.h"
#include "common/logger.h"

namespace YL_SLAM {

RadialTangentialDistortion::RadialTangentialDistortion(const VecXf &parameters) {
    YL_CHECK(parameters.size() == 4, "Parameters size should be 4! Order: [k1, k2, p1, p2]");
    k1_ = parameters[0];
    k2_ = parameters[1];
    p1_ = parameters[2];
    p2_ = parameters[3];
}

Vec2f RadialTangentialDistortion::distort(const Vec2f &uv) const {
    const auto &u     = uv[0];
    const auto &v     = uv[1];
    const auto u2     = u * u;
    const auto v2     = v * v;
    const auto _uv    = u * v;
    const auto uv_t2  = YL_FLOAT(2.0) * _uv;
    const auto r2     = u2 + v2;
    const auto c_dist = (k1_ + k2_ * r2) * r2;
    return {u + u * c_dist + p1_ * uv_t2 + p2_ * (r2 + YL_FLOAT(2.0) * u2),
            v + v * c_dist + p2_ * uv_t2 + p1_ * (r2 + YL_FLOAT(2.0) * v2)};
}

Vec2f RadialTangentialDistortion::undistort(const Vec2f &uv) const {
    Vec2f ret(uv);
    const auto &u0 = uv[0];
    const auto &v0 = uv[1];
    auto &u        = ret[0];
    auto &v        = ret[1];
    for (auto i = 0; i < 5; ++i) {
        const auto u2         = u * u;
        const auto v2         = v * v;
        const auto _uv        = u * v;
        const auto uv_t2      = YL_FLOAT(2.0) * _uv;
        const auto r2         = u2 + v2;
        const auto c_dist_inv = YL_FLOAT(1.0) / (YL_FLOAT(1.0) + (k1_ + k2_ * r2) * r2);
        const auto du         = p1_ * uv_t2 + p2_ * (r2 + YL_FLOAT(2.0) * u2);
        const auto dv         = p2_ * uv_t2 + p1_ * (r2 + YL_FLOAT(2.0) * v2);

        u = (u0 - du) * c_dist_inv;
        v = (v0 - dv) * c_dist_inv;
    }
    return ret;
}

Mat22f RadialTangentialDistortion::jacobian(const Vec2f &uv) const {
    const auto &u        = uv[0];
    const auto &v        = uv[1];
    const auto u2        = u * u;
    const auto v2        = v * v;
    const auto _uv       = u * v;
    const auto r2        = u2 + v2;
    const auto c_dist    = (k1_ + k2_ * r2) * r2;
    const auto k2r2_t4   = k2_ * r2 * YL_FLOAT(4.0);
    const auto c_dist_p1 = c_dist + YL_FLOAT(1.0);

    Mat22f J_dist;
    J_dist(0, 0) =
            c_dist_p1 + k1_ * YL_FLOAT(2.0) * u2 + k2r2_t4 * u2 + YL_FLOAT(2.0) * p1_ * v + YL_FLOAT(6.0) * p2_ * u;
    J_dist(1, 1) =
            c_dist_p1 + k1_ * YL_FLOAT(2.0) * v2 + k2r2_t4 * v2 + YL_FLOAT(2.0) * p2_ * u + YL_FLOAT(6.0) * p1_ * v;
    J_dist(1, 0) = YL_FLOAT(2.0) * k1_ * _uv + k2r2_t4 * _uv + YL_FLOAT(2.0) * p1_ * u + YL_FLOAT(2.0) * p2_ * v;
    J_dist(0, 1) = J_dist(1, 0);
    return J_dist;
}

void RadialTangentialDistortion::print(std::ostream &out) const {
    out << "  Distortion: RadTan(" << YL_FLOAT_FMT(k1_) << ", " << YL_FLOAT_FMT(k2_) << ", " << YL_FLOAT_FMT(p1_)
        << ", " << YL_FLOAT_FMT(p2_) << ")" << std::endl;
}

VecXf RadialTangentialDistortion::distortionParameters() const {
    VecXf distortion(4);
    distortion[0] = k1_;
    distortion[1] = k2_;
    distortion[2] = p1_;
    distortion[3] = p2_;
    return distortion;
}

} // namespace YL_SLAM
