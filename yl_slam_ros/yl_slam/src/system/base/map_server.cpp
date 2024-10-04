#include "system/base/map_server.h"
#include "common/yaml/yaml_serialization.h"

namespace YL_SLAM {

MapServer::MapServer([[maybe_unused]] const YAML::Node &config) {
    map_ = std::make_unique<Map>();
}

void MapServer::addReadTask(const MapServer::rtask_t &task) {
    rlock_t lock(mutex_);
    task(*map_);
}

void MapServer::addWriteTask(const MapServer::wtask_t &task) {
    wlock_t lock(mutex_);
    task(*map_);
}

} // namespace YL_SLAM
