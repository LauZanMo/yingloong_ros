#pragma once

#include "common/mutex_types.h"
#include "system/base/map.h"

#include <yaml-cpp/yaml.h>

namespace YL_SLAM {

/**
 * @brief 地图服务器类
 * @details 地图服务器类用于管理多线程对地图的读写操作
 */
class MapServer {
public:
    using sPtr    = std::shared_ptr<MapServer>;
    using rtask_t = std::function<void(const Map &)>;
    using wtask_t = std::function<void(Map &)>;

    /**
     * @brief 构造函数
     * @param config 地图服务器配置YAML节点
     */
    explicit MapServer(const YAML::Node &config);

    /**
     * @brief 添加地图读取任务
     * @param task 地图读取任务
     */
    void addReadTask(const rtask_t &task);

    /**
     * @brief 添加地图写入任务
     * @param task 地图写入任务
     */
    void addWriteTask(const wtask_t &task);

private:
    rw_mutex_t mutex_; ///< 地图读写锁
    Map::uPtr map_;    ///< 地图实例指针
};

} // namespace YL_SLAM
