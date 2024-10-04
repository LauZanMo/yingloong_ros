#pragma once

#include "system/base/frame_bundle.h"

#include <deque>
#include <unordered_map>

namespace YL_SLAM {

/**
 * @brief 地图类
 * @details 地图用于管理关键帧束，关键帧束管理关键帧，关键帧管理地图点（即种子点）和观测
 */
class Map : public NonCopyable {
public:
    using uPtr = std::unique_ptr<Map>;

    /**
     * @brief 默认构造函数
     */
    Map() = default;

    /**
     * @brief 默认析构函数
     */
    ~Map() = default;

    /**
     * @brief 重置地图
     */
    void reset();

    /**
     * @brief 添加关键帧束
     * @param bundle 待添加的关键帧束
     */
    void addKeyframeBundle(const FrameBundle::sPtr &bundle);

    /**
     * @brief 移除指定索引下地图中的关键帧束
     * @param idx 指定索引
     */
    void removeKeyframeBundle(size_t idx);

    /**
     * @brief 获取地图中关键帧束的数量
     * @return 地图中关键帧束的数量
     */
    size_t numKeyframeBundles() const;

    /**
     * @brief 获取指定索引下的关键帧束常量指针
     * @param idx 指定索引
     * @return 指定索引下的关键帧束常量指针
     */
    [[nodiscard]] FrameBundle::sConstPtr keyframeBundle(size_t idx) const;

    /**
     * @brief 获取指定索引下的关键帧束指针
     * @param idx 指定索引
     * @return 指定索引下的关键帧束指针
     */
    [[nodiscard]] FrameBundle::sPtr &mutableKeyframeBundle(size_t idx);

private:
    std::deque<FrameBundle::sPtr> keyframe_bundles_;  ///< 关键帧束
    std::unordered_map<long, Frame::sPtr> keyframes_; ///< 地图中的所有关键帧，以帧id为键，用于快速查找关键帧
};

} // namespace YL_SLAM
