#include "common/logger.h"

namespace YL_SLAM {

template<typename Distortion>
PinholeProjection<Distortion>::PinholeProjection(const VecXf &intrinsics, const distortion_t &distortion)
    : distortion_(distortion) {
    YL_CHECK(intrinsics.size() == 4, "Intrinsics size should be 4! Order: [fx, fy, cx, cy]");
    fx_           = intrinsics[0];
    fy_           = intrinsics[1];
    cx_           = intrinsics[2];
    cy_           = intrinsics[3];
    f_matrix_     = Diag2f(fx_, fy_);
    f_inv_matrix_ = Diag2f(YL_FLOAT(1.0) / fx_, YL_FLOAT(1.0) / fy_);
    c_vec_        = Vec2f(cx_, cy_);
}

template<typename Distortion>
bool PinholeProjection<Distortion>::project(const Eigen::Ref<const Vec3f> &point,
                                            Eigen::Ref<Vec2f> &out_keypoint) const {
    if (point[2] <= YL_FLOAT(0.0))
        return false;

    const auto z_inv = YL_FLOAT(1.0) / point[2];
    const Vec2f uv   = point.head<2>() * z_inv;
    out_keypoint     = f_matrix_ * distortion_.distort(uv) + c_vec_;

    return true;
}

template<typename Distortion>
bool PinholeProjection<Distortion>::project(const Eigen::Ref<const Vec3f> &point, Eigen::Ref<Vec2f> &out_keypoint,
                                            Mat23f &out_jacobian) const {
    if (point[2] <= YL_FLOAT(0.0))
        return false;

    const auto z_inv = YL_FLOAT(1.0) / point[2];
    const Vec2f uv   = point.head<2>() * z_inv;
    out_keypoint     = f_matrix_ * distortion_.distort(uv) + c_vec_;

    Mat23f duv_dxyz;
    duv_dxyz.leftCols<2>()  = Mat22f::Identity() * z_inv;
    duv_dxyz.rightCols<1>() = -point.head<2>() * z_inv * z_inv;
    out_jacobian            = f_matrix_ * distortion_.jacobian(uv) * duv_dxyz;

    return true;
}

template<typename Distortion>
bool PinholeProjection<Distortion>::unproject(const Eigen::Ref<const Vec2f> &keypoint,
                                              Eigen::Ref<Vec3f> &out_point) const {
    Vec2f uv            = f_inv_matrix_ * (keypoint - c_vec_);
    out_point.head<2>() = distortion_.undistort(uv);
    out_point[2]        = YL_FLOAT(1.0);

    return true;
}

template<typename Distortion>
bool PinholeProjection<Distortion>::unproject(const Eigen::Ref<const Vec2f> &keypoint, Eigen::Ref<Vec3f> &out_point,
                                              Mat32f &out_jacobian) const {
    Vec2f uv            = f_inv_matrix_ * (keypoint - c_vec_);
    out_point.head<2>() = distortion_.undistort(uv);
    out_point[2]        = YL_FLOAT(1.0);

    out_jacobian.setZero();
    out_jacobian.topRows<2>() = f_inv_matrix_;
    out_jacobian *= distortion_.jacobian(out_point.head<2>()).inverse();

    return true;
}

template<typename Distortion>
void PinholeProjection<Distortion>::print(std::ostream &out) const {
    out << "  Projection = Pinhole" << std::endl;
    out << "  Focal length = (" << YL_FLOAT_FMT(fx_) << ", " << YL_FLOAT_FMT(fy_) << ")" << std::endl;
    out << "  Principal point = (" << YL_FLOAT_FMT(cx_) << ", " << YL_FLOAT_FMT(cy_) << ")" << std::endl;
    distortion_.print(out);
}

template<typename Distortion>
VecXf PinholeProjection<Distortion>::intrinsicParameters() const {
    VecXf intrinsics(4);
    intrinsics[0] = fx_;
    intrinsics[1] = fy_;
    intrinsics[2] = cx_;
    intrinsics[3] = cy_;
    return intrinsics;
}

template<typename Distortion>
VecXf PinholeProjection<Distortion>::distortionParameters() const {
    return distortion_.distortionParameters();
}

template<typename Distortion>
const typename PinholeProjection<Distortion>::distortion_t &PinholeProjection<Distortion>::distortion() const {
    return distortion_;
}

} // namespace YL_SLAM
