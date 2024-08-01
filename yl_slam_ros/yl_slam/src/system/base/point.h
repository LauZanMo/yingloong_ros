#pragma once

#include "common/non_copyable.h"
#include "system/base/system_types.h"

#include <memory>

namespace YL_SLAM {

class Frame;
using FrameSPtr      = std::shared_ptr<Frame>;
using FrameSConstPtr = std::shared_ptr<const Frame>;
using FrameWPtr      = std::weak_ptr<Frame>;
using FrameWConstPtr = std::weak_ptr<const Frame>;

class Point : public NonCopyable {
public:
    using sPtr          = std::shared_ptr<Point>;
    using sConstPtr     = std::shared_ptr<const Point>;
    using wConstPtr     = std::weak_ptr<const Point>;
    using observation_t = std::pair<FrameWConstPtr, size_t>;

    /**
     * @brief 三维点类型枚举
     * @details CORNER_SEED: 角点种子<br/>
     *          EDGELET_SEED: 边缘种子<br/>
     *          CORNER_SEED_CONVERGED: 收敛的角点种子<br/>
     *          EDGELET_SEED_CONVERGED: 收敛的边缘种子<br/>
     *          EDGELET: 边缘<br/>
     *          CORNER: 角点
     */
    enum class Type {
        CORNER_SEED,
        EDGELET_SEED,
        CORNER_SEED_CONVERGED,
        EDGELET_SEED_CONVERGED,
        CORNER,
        EDGELET
    };

    /**
     * @brief 构造函数
     * @param pos 三维点在世界坐标系下的坐标
     * @param seed_frame 三维点的种子帧指针
     * @param seed_idx 三维点在种子帧中的特征索引
     * @param depth 三维点在种子帧中的深度
     * @param type 三维点类型
     */
    Point(Position pos, const FrameSPtr &seed_frame, size_t seed_idx, FloatType depth, Type type);

    /**
     * @brief 默认析构函数
     */
    ~Point() = default;

    /**
     * @brief 获取三维点id
     * @return 三维点id
     */
    [[nodiscard]] long id() const;

    /**
     * @brief 获取三维点在世界坐标系下的坐标
     * @return 三维点在世界坐标系下的坐标
     */
    [[nodiscard]] const Position &pos() const;

    /**
     * @brief 设置三维点在世界坐标系下的坐标
     * @param pos 三维点在世界坐标系下的坐标
     */
    void setPos(const Position &pos);

    /**
     * @brief 获取三维点的种子帧常量指针
     * @return 三维点的种子帧常量指针
     */
    [[nodiscard]] FrameSConstPtr seedFrame() const;

    /**
     * @brief 获取三维点的种子帧指针
     * @return 三维点的种子帧指针
     */
    [[nodiscard]] FrameSPtr mutableSeedFrame();

    /**
     * @brief 获取三维点在种子帧中的特征索引
     * @return 三维点在种子帧中的特征索引
     */
    [[nodiscard]] size_t seedIdx() const;

    /**
     * @brief 获取三维点在种子帧中的深度
     * @return 三维点在种子帧中的深度
     */
    [[nodiscard]] FloatType depth() const;

    /**
     * @brief 设置三维点在种子帧中的深度
     * @param depth 三维点在种子帧中的深度
     */
    void setDepth(FloatType depth);

    /**
     * @brief 获取三维点类型
     * @return 三维点类型
     */
    [[nodiscard]] Type type() const;

    /**
     * @brief 设置三维点类型
     * @param type 三维点类型
     */
    void setType(Type type);

    /**
     * @brief 添加观测到该三维点的地图关键帧指针及对应的特征索引
     * @param frame 观测到该三维点的地图关键帧指针
     * @param idx 对应的的特征索引
     */
    void addObservation(const FrameSPtr &frame, size_t idx);

    /**
     * @brief 获取观测到该三维点的地图关键帧数量
     * @return 观测到该三维点的地图关键帧数量
     * @note 该数量不包含种子帧
     */
    [[nodiscard]] size_t numObservations() const;

    /**
     * @brief 获取指定索引下观测到该三维点的地图关键帧常量指针及对应的特征索引
     * @param idx 指定索引
     * @return 指定索引下观测到该三维点的地图关键帧常量指针及对应的特征索引
     */
    [[nodiscard]] std::pair<FrameSConstPtr, size_t> observation(size_t idx) const;

private:
    long id_;              ///< 三维点id（历史唯一）
    Position pos_;         ///< 三维点在世界坐标系下的坐标
    FrameWPtr seed_frame_; ///< 三维点的种子帧指针
    size_t seed_idx_;      ///< 三维点在种子帧中的特征索引
    FloatType depth_;      ///< 三维点在种子帧中的深度
    Type type_;            ///< 三维点类型
    std::vector<observation_t> observations_; ///< 观测到该三维点的所有地图关键帧指针及对应的特征索引（用于边缘化）
};

} // namespace YL_SLAM
