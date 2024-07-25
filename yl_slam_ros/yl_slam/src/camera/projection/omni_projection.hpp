#include "common/logger.h"

namespace YL_SLAM {

template<typename Distortion>
OmniProjection<Distortion>::OmniProjection(const VecXf &intrinsics, const distortion_t &distortion)
    : distortion_(distortion) {
    YL_CHECK(intrinsics.size() == 5, "Intrinsics size should be 5! Order: [xi, fx, fy, cx, cy]");
    xi_           = intrinsics[0];
    fx_           = intrinsics[1];
    fy_           = intrinsics[2];
    cx_           = intrinsics[3];
    cy_           = intrinsics[4];
    xi2_m1_inv_   = 1.0 / (xi_ * xi_ - 1.0);
    fov_param_    = (xi_ <= YL_FLOAT(1.0)) ? xi_ : YL_FLOAT(1.0) / xi_;
    f_matrix_     = Diag2f(fx_, fy_);
    f_inv_matrix_ = Diag2f(YL_FLOAT(1.0) / fx_, YL_FLOAT(1.0) / fy_);
    c_vec_        = Vec2f(cx_, cy_);
}

template<typename Distortion>
bool OmniProjection<Distortion>::project(const Eigen::Ref<const Vec3f> &point, Eigen::Ref<Vec2f> &out_keypoint) const {
    const auto d = point.norm();
    if (point[2] <= -(fov_param_ * d))
        return false;

    const auto z_inv = YL_FLOAT(1.0) / (point[2] + xi_ * d);
    const Vec2f uv   = point.head<2>() * z_inv;
    out_keypoint     = f_matrix_ * distortion_.distort(uv) + c_vec_;

    return true;
}

template<typename Distortion>
bool OmniProjection<Distortion>::project(const Eigen::Ref<const Vec3f> &point, Eigen::Ref<Vec2f> &out_keypoint,
                                         Mat23f &out_jacobian) const {
    const auto d = point.norm();
    if (point[2] <= -(fov_param_ * d))
        return false;

    const auto z_inv = YL_FLOAT(1.0) / (point[2] + xi_ * d);
    const Vec2f uv   = point.head<2>() * z_inv;
    out_keypoint     = f_matrix_ * distortion_.distort(uv) + c_vec_;

    Mat23f duv_dxyz;
    const auto c1           = z_inv * z_inv / d;
    const auto c2           = -c1 * (xi_ * point[2] + d);
    duv_dxyz(0, 0)          = c1 * (d * point[2] + xi_ * (point[1] * point[1] + point[2] * point[2]));
    duv_dxyz(0, 1)          = -c1 * xi_ * point[0] * point[1];
    duv_dxyz(1, 0)          = duv_dxyz(0, 1);
    duv_dxyz(1, 1)          = c1 * (d * point[2] + xi_ * (point[0] * point[0] + point[2] * point[2]));
    duv_dxyz.rightCols<1>() = c2 * point.head<2>();
    out_jacobian            = f_matrix_ * distortion_.jacobian(uv) * duv_dxyz;

    return true;
}

template<typename Distortion>
bool OmniProjection<Distortion>::unproject(const Eigen::Ref<const Vec2f> &keypoint,
                                           Eigen::Ref<Vec3f> &out_point) const {
    Vec2f uv            = f_inv_matrix_ * (keypoint - c_vec_);
    out_point.head<2>() = distortion_.undistort(uv);

    const auto rd2 = out_point.head<2>().squaredNorm();
    if (!isRd2Valid(rd2))
        return false;

    out_point[2] = YL_FLOAT(1.0) -
                   xi_ * (rd2 + YL_FLOAT(1.0)) / (xi_ + std::sqrt(YL_FLOAT(1.0) + (YL_FLOAT(1.0) - xi_ * xi_) * rd2));

    return true;
}

template<typename Distortion>
bool OmniProjection<Distortion>::unproject(const Eigen::Ref<const Vec2f> &keypoint, Eigen::Ref<Vec3f> &out_point,
                                           Mat32f &out_jacobian) const {
    Vec2f uv            = f_inv_matrix_ * (keypoint - c_vec_);
    out_point.head<2>() = distortion_.undistort(uv);

    const auto rd2 = out_point.head<2>().squaredNorm();
    if (!isRd2Valid(rd2))
        return false;

    const auto c1      = std::sqrt(YL_FLOAT(1.0) + (YL_FLOAT(1.0) - xi_ * xi_) * rd2);
    const auto c1_inv  = YL_FLOAT(1.0) / c1;
    const auto c2      = xi_ + c1;
    const auto c2_inv  = YL_FLOAT(1.0) / c2;
    const auto c22_inv = c2_inv * c2_inv;
    const auto c3 =
            -xi_ * (YL_FLOAT(2.0) * c2_inv + c22_inv * (xi_ * xi_ - YL_FLOAT(1.0)) * c1_inv * (rd2 + YL_FLOAT(1.0)));

    out_point[2] = YL_FLOAT(1.0) - xi_ * (rd2 + YL_FLOAT(1.0)) * c2_inv;

    const Mat22f J_dist          = distortion_.jacobian(out_point.head<2>()).inverse();
    out_jacobian.topRows<2>()    = f_inv_matrix_ * J_dist;
    out_jacobian.bottomRows<1>() = c3 * out_point.head<2>().transpose() * J_dist * f_inv_matrix_;

    return true;
}

template<typename Distortion>
void OmniProjection<Distortion>::print(std::ostream &out) const {
    out << "  Projection = Omni" << std::endl;
    out << "  xi = " << YL_FLOAT_FMT(xi_) << std::endl;
    out << "  Focal length = (" << YL_FLOAT_FMT(fx_) << ", " << YL_FLOAT_FMT(fy_) << ")" << std::endl;
    out << "  Principal point = (" << YL_FLOAT_FMT(cx_) << ", " << YL_FLOAT_FMT(cy_) << ")" << std::endl;
    distortion_.print(out);
}

template<typename Distortion>
VecXf OmniProjection<Distortion>::intrinsicParameters() const {
    VecXf intrinsics(5);
    intrinsics[0] = xi_;
    intrinsics[1] = fx_;
    intrinsics[2] = fy_;
    intrinsics[3] = cx_;
    intrinsics[4] = cy_;
    return intrinsics;
}

template<typename Distortion>
VecXf OmniProjection<Distortion>::distortionParameters() const {
    return distortion_.distortionParameters();
}

template<typename Distortion>
const typename OmniProjection<Distortion>::distortion_t &OmniProjection<Distortion>::distortion() const {
    return distortion_;
}

template<typename Distortion>
bool OmniProjection<Distortion>::isRd2Valid(const FloatType &rd2) const {
    return xi_ <= 1.0 || rd2 <= xi2_m1_inv_;
}

} // namespace YL_SLAM
