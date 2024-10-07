#pragma once

#include "common/nav_state.h"
#include "system/base/frame.h"

namespace YL_SLAM {

/**
 * @brief 帧束类
 * @details 帧束是指同一时刻所有帧的集合（多目），帧束中所有帧的时间戳相同
 */
class FrameBundle : public NonCopyable {
public:
    using sPtr      = std::shared_ptr<FrameBundle>;
    using sConstPtr = std::shared_ptr<const FrameBundle>;

    /**
     * @brief 构造函数
     * @param frames 同一时刻所有帧指针
     */
    explicit FrameBundle(const std::vector<Frame::sPtr> &frames);

    /**
     * @brief 默认析构函数
     */
    ~FrameBundle() = default;

    /**
     * @brief 获取帧束时间戳
     * @return 帧束时间戳
     */
    [[nodiscard]] int64_t timestamp() const;

    /**
     * @brief 获取帧束id
     * @return 帧束id
     */
    [[nodiscard]] long bundleId() const;

    /**
     * @brief 获取世界坐标系下body坐标系（通常是IMU）的导航状态
     * @return 世界坐标系下body坐标系（通常是IMU）的导航状态
     */
    [[nodiscard]] const NavState &state() const;

    /**
     * @brief 设置世界坐标系下body坐标系（通常是IMU）的导航状态
     * @param state 世界坐标系下body坐标系（通常是IMU）的导航状态
     */
    void setState(const NavState &state);

    /**
     * @brief 获取世界坐标系到body坐标系（通常是IMU）的变换
     * @return 世界坐标系到body坐标系（通常是IMU）的变换
     */
    [[nodiscard]] const SE3f &Twb() const;

    /**
     * @brief 设置世界坐标系到body坐标系（通常是IMU）的变换
     * @param T_wb 世界坐标系到body坐标系（通常是IMU）的变换
     */
    void setTwb(const SE3f &T_wb);

    /**
     * @brief 设置帧束中各帧的外参
     * @param T_bs_vec 帧束中各帧的外参
     */
    void setTbs(const std::vector<SE3f> &T_bs_vec);

    /**
     * @brief 获取帧束中帧的数量
     * @return 帧束中帧的数量
     */
    [[nodiscard]] size_t numFrames() const;

    /**
     * @brief 获取指定索引下的帧常量指针
     * @param idx 指定索引
     * @return 指定索引下的帧常量指针
     */
    [[nodiscard]] Frame::sConstPtr frame(size_t idx) const;

    /**
     * @brief 获取指定索引下的帧指针
     * @param idx 指定索引
     * @return 指定索引下的帧指针
     */
    [[nodiscard]] Frame::sPtr &mutableFrame(size_t idx);

private:
    long bundle_id_;                  ///< 帧束id（历史唯一）
    NavState state_;                  ///< 世界坐标系下的导航状态
    std::vector<Frame::sPtr> frames_; ///< 帧束中所有帧的指针（帧的时间戳相同）
};

} // namespace YL_SLAM
