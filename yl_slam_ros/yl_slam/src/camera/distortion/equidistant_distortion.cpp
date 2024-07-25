#include "camera/distortion/equidistant_distortion.h"
#include "common/logger.h"

namespace YL_SLAM {

EquidistantDistortion::EquidistantDistortion(const VecXf &parameters) {
    YL_CHECK(parameters.size() == 4, "Parameters size should be 4! Order: [k1, k2, k3, k4]");
    k1_ = parameters[0];
    k2_ = parameters[1];
    k3_ = parameters[2];
    k4_ = parameters[3];
}

Vec2f EquidistantDistortion::distort(const Vec2f &uv) const {
    const auto &u = uv[0];
    const auto &v = uv[1];
    const auto r  = std::sqrt(u * u + v * v);
    if (r < thresh_) {
        return uv;
    }

    const auto theta      = std::atan(r);
    const auto theta_dist = distortTheta(theta);
    const auto scaling    = theta_dist / r;
    return {u * scaling, v * scaling};
}

Vec2f EquidistantDistortion::undistort(const Vec2f &uv) const {
    const auto &u         = uv[0];
    const auto &v         = uv[1];
    const auto theta_dist = std::sqrt(u * u + v * v);
    auto theta            = theta_dist;
    for (int i = 0; i < 5; ++i) {
        const auto theta2 = theta * theta;
        const auto theta4 = theta2 * theta2;
        const auto theta6 = theta4 * theta2;
        const auto theta8 = theta4 * theta4;
        theta             = theta_dist / (YL_FLOAT(1.0) + k1_ * theta2 + k2_ * theta4 + k3_ * theta6 + k4_ * theta8);
    }
    const auto scaling = std::tan(theta) / theta_dist;
    return {u * scaling, v * scaling};
}

Mat22f EquidistantDistortion::jacobian(const Vec2f &uv) const {
    const auto r = uv.norm();
    if (r < thresh_) {
        return Mat22f::Identity();
    }

    const auto r_inv = YL_FLOAT(1.0) / r;
    const auto r2    = r * r;
    const auto dr_du = uv[0] * r_inv;
    const auto dr_dv = uv[1] * r_inv;

    const auto theta     = std::atan(r);
    const auto dtheta_dr = YL_FLOAT(1.0) / (YL_FLOAT(1.0) + r * r);

    const auto theta_dist         = distortTheta(theta);
    const auto dtheta_dist_dtheta = jacobianTheta(theta);
    const auto dtheta_dist_dr     = dtheta_dist_dtheta * dtheta_dr;

    const auto scaling     = theta_dist / r;
    const auto dscaling_du = (dtheta_dist_dr * dr_du * r - dr_du * theta_dist) / r2;
    const auto dscaling_dv = (dtheta_dist_dr * dr_dv * r - dr_dv * theta_dist) / r2;

    Mat22f J_dist;
    J_dist(0, 0) = dscaling_du * uv[0] + scaling;
    J_dist(0, 1) = dscaling_dv * uv[0];
    J_dist(1, 0) = dscaling_du * uv[1];
    J_dist(1, 1) = dscaling_dv * uv[1] + scaling;
    return J_dist;
}

void EquidistantDistortion::print(std::ostream &out) const {
    out << "  Distortion: Equidistant(" << YL_FLOAT_FMT(k1_) << ", " << YL_FLOAT_FMT(k2_) << ", " << YL_FLOAT_FMT(k3_)
        << ", " << YL_FLOAT_FMT(k4_) << ")" << std::endl;
}

VecXf EquidistantDistortion::distortionParameters() const {
    VecXf distortion(4);
    distortion[0] = k1_;
    distortion[1] = k2_;
    distortion[2] = k3_;
    distortion[3] = k4_;
    return distortion;
}

FloatType EquidistantDistortion::distortTheta(FloatType theta) const {
    const auto theta2     = theta * theta;
    const auto theta4     = theta2 * theta2;
    const auto theta6     = theta4 * theta2;
    const auto theta8     = theta4 * theta4;
    const auto theta_dist = theta * (YL_FLOAT(1.0) + k1_ * theta2 + k2_ * theta4 + k3_ * theta6 + k4_ * theta8);
    return theta_dist;
}

FloatType EquidistantDistortion::jacobianTheta(FloatType theta) const {
    const auto theta2 = theta * theta;
    const auto theta4 = theta2 * theta2;
    const auto theta6 = theta4 * theta2;
    const auto theta8 = theta4 * theta4;
    return YL_FLOAT(1.0) + YL_FLOAT(3.0) * k1_ * theta2 + YL_FLOAT(5.0) * k2_ * theta4 + YL_FLOAT(7.0) * k3_ * theta6 +
           YL_FLOAT(9.0) * k4_ * theta8;
}

} // namespace YL_SLAM
