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
     * @brief 构造函数
     * @param window_size 关键帧束滑动窗口的大小
     */
    explicit Map(size_t window_size);

    /**
     * @brief 默认析构函数
     */
    ~Map() = default;

    /**
     * @brief 重置地图
     */
    void reset();

    /**
     * @brief 获取关键帧束滑动窗口是否已满
     * @return 关键帧束滑动窗口是否已满
     */
    bool isFull() const;

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

    /**
     * @brief 添加待移除的地图点
     * @param point 待移除的地图点
     * @note 该方法用于遍历判断粗差地图点时
     */
    void addTrashPoint(const Point::sConstPtr &point);

    /**
     * @brief 移除待移除的地图点
     * @note 该方法用于遍历判断粗差地图点后
     */
    void removeTrashPoints();

private:
    size_t window_size_;                             ///< 关键帧束滑动窗口的大小
    std::deque<FrameBundle::sPtr> keyframe_bundles_; ///< 关键帧束滑动窗口
    std::unordered_map<long, Frame::sPtr> keyframes_; ///< 地图中的所有关键帧，以帧id为键，用于快速查找关键帧

    std::vector<Point::sConstPtr> trash_points_; ///< 丢弃的地图点
};

} // namespace YL_SLAM
