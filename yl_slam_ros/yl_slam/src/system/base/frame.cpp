#include "system/base/frame.h"
#include "common/logger.h"

#include <opencv2/imgproc.hpp>

namespace YL_SLAM {

static std::atomic<long> frame_counter{0};

Frame::Frame(int64_t timestamp, const CameraGeometryBase::sPtr &camera, const SE3f &T_bc, cv::Mat image,
             size_t pyr_levels)
    : timestamp_(timestamp), id_(frame_counter++), camera_(camera), T_bc_(T_bc), raw_image_(std::move(image)) {
    YL_CHECK(raw_image_.rows == camera_->height() && raw_image_.cols == camera_->width(),
             "Image size should match camera geometry!");

    // 创建图像金字塔
    createImagePyramid(raw_image_, image_pyr_, pyr_levels);
}

int64_t Frame::timestamp() const {
    return timestamp_;
}

long Frame::id() const {
    return id_;
}

long Frame::bundleId() const {
    return bundle_id_;
}

void Frame::setBundleId(long bundle_id) {
    bundle_id_ = bundle_id;
}

const CameraGeometryBase::sConstPtr &Frame::camera() const {
    return camera_;
}

const SE3f &Frame::Twf() const {
    return T_wf_;
}

void Frame::setTwf(const SE3f &T_wf) {
    T_wf_ = T_wf;
}

const SE3f &Frame::Tbc() const {
    return T_bc_;
}

void Frame::setTbc(const SE3f &T_bc) {
    T_bc_ = T_bc;
}

const ImagePyr &Frame::imagePyr() const {
    return image_pyr_;
}

const cv::Mat &Frame::rawImage() const {
    return raw_image_;
}

void Frame::addObservations(const Keypoints &kps, const Bearings &f, const std::vector<Point::sConstPtr> &points) {
    YL_CHECK(kps.cols() == f.cols() && kps.cols() == static_cast<long>(points.size()), "Observation size mismatch!");

    // 申请内存
    const auto prev_size = obs_kp_vec_.cols();
    const auto cur_size  = prev_size + kps.cols();
    obs_kp_vec_.conservativeResize(Eigen::NoChange, cur_size);
    obs_f_vec_.conservativeResize(Eigen::NoChange, cur_size);
    obs_point_vec_.resize(cur_size);

    // 添加观测
    obs_kp_vec_.middleCols(prev_size, kps.cols()) = kps;
    obs_f_vec_.middleCols(prev_size, f.cols())    = f;
    for (size_t i = 0; i < points.size(); ++i) {
        obs_point_vec_[prev_size + i] = points[i];
    }
}

void Frame::removeObservation(size_t idx) {
    YL_CHECK(idx < obs_point_vec_.size(), "Index should be less than the number of observation!");
    obs_point_vec_[idx].reset();
}

size_t Frame::numObservations() const {
    YL_CHECK(obs_kp_vec_.cols() == obs_f_vec_.cols() && obs_kp_vec_.cols() == static_cast<long>(obs_point_vec_.size()),
             "Observation size mismatch!");
    return obs_point_vec_.size();
}

Eigen::Ref<const Keypoint> Frame::obsKeypoint(size_t idx) const {
    YL_CHECK(idx < static_cast<size_t>(obs_kp_vec_.cols()), "Index should be less than the number of observation!");
    return obs_kp_vec_.col(static_cast<long>(idx));
}

Eigen::Ref<const Bearing> Frame::obsBearing(size_t idx) const {
    YL_CHECK(idx < static_cast<size_t>(obs_f_vec_.cols()), "Index should be less than the number of observation!");
    return obs_f_vec_.col(static_cast<long>(idx));
}

Point::sConstPtr Frame::obsPoint(size_t idx) const {
    YL_CHECK(idx < obs_point_vec_.size(), "Index should be less than the number of observation!");
    return obs_point_vec_[idx].lock();
}

void Frame::addSeeds(const Keypoints &kps, const Bearings &f, const std::vector<Point::sPtr> &points,
                     const SeedStates &seed_states) {
    YL_CHECK(kps.cols() == f.cols() && kps.cols() == static_cast<long>(points.size()) &&
                     kps.cols() == seed_states.cols(),
             "Seed size mismatch!");

    // 申请内存
    const auto prev_size = seed_kp_vec_.cols();
    const auto cur_size  = prev_size + kps.cols();
    seed_kp_vec_.conservativeResize(Eigen::NoChange, cur_size);
    seed_f_vec_.conservativeResize(Eigen::NoChange, cur_size);
    seed_point_vec_.resize(cur_size);
    seed_state_vec_.conservativeResize(Eigen::NoChange, cur_size);

    // 添加种子
    seed_kp_vec_.middleCols(prev_size, kps.cols()) = kps;
    seed_f_vec_.middleCols(prev_size, f.cols())    = f;
    for (size_t i = 0; i < points.size(); ++i) {
        seed_point_vec_[prev_size + i] = points[i];
    }
    seed_state_vec_.middleCols(prev_size, seed_states.cols()) = seed_states;
}

void Frame::removeSeed(size_t idx) {
    YL_CHECK(idx < seed_point_vec_.size(), "Index should be less than the number of seeds!");
    seed_point_vec_[idx].reset();
}

size_t Frame::numSeeds() const {
    YL_CHECK(seed_kp_vec_.cols() == seed_f_vec_.cols() &&
                     seed_kp_vec_.cols() == static_cast<long>(seed_point_vec_.size()) &&
                     seed_kp_vec_.cols() == static_cast<long>(seed_state_vec_.size()),
             "Seed size mismatch!");
    return seed_point_vec_.size();
}

Eigen::Ref<const Keypoint> Frame::seedKeypoint(size_t idx) const {
    YL_CHECK(idx < static_cast<size_t>(seed_kp_vec_.cols()), "Index should be less than the number of seeds!");
    return seed_kp_vec_.col(static_cast<long>(idx));
}

Eigen::Ref<const Bearing> Frame::seedBearing(size_t idx) const {
    YL_CHECK(idx < static_cast<size_t>(seed_f_vec_.cols()), "Index should be less than the number of seeds!");
    return seed_f_vec_.col(static_cast<long>(idx));
}

Point::sConstPtr Frame::seedPoint(size_t idx) const {
    YL_CHECK(idx < seed_point_vec_.size(), "Index should be less than the number of seeds!");
    return seed_point_vec_[idx];
}

Point::sPtr &Frame::mutableSeedPoint(size_t idx) {
    YL_CHECK(idx < seed_point_vec_.size(), "Index should be less than the number of seeds!");
    return seed_point_vec_[idx];
}

Eigen::Ref<const SeedState> Frame::seedState(size_t idx) const {
    YL_CHECK(idx < static_cast<size_t>(seed_state_vec_.cols()), "Index should be less than the number of seeds!");
    return seed_state_vec_.col(static_cast<long>(idx));
}

Eigen::Ref<SeedState> Frame::mutableSeedState(size_t idx) {
    YL_CHECK(idx < static_cast<size_t>(seed_state_vec_.cols()), "Index should be less than the number of seeds!");
    return seed_state_vec_.col(static_cast<long>(idx));
}

void Frame::createImagePyramid(const cv::Mat &image, ImagePyr &image_pyr, size_t pyr_levels) {
    // 输入检查
    YL_CHECK(image.type() == CV_8UC1 || image.type() == CV_8UC3, "Image type should be CV_8UC1 or CV_8UC3!");
    YL_CHECK(image.rows > 0 && image.cols > 0, "Image size should be greater than 0!");
    YL_CHECK(pyr_levels > 0, "Pyramid levels should be greater than 0!");

    // 转为灰度图
    image_pyr.resize(pyr_levels);
    if (image.type() == CV_8UC3) {
        cv::cvtColor(image, image_pyr[0], cv::COLOR_BGR2GRAY);
    } else {
        image_pyr[0] = image;
    }

    // 创建图像金字塔
    for (size_t i = 1; i < pyr_levels; ++i) {
        cv::resize(image_pyr[i - 1], image_pyr[i], cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
    }
}

} // namespace YL_SLAM
