#include "system/base/map_server.h"
#include "common/yaml/yaml_serialization.h"

namespace YL_SLAM {

MapServer::MapServer(const YAML::Node &config) {
    const auto window_size = YAML::get<size_t>(config, "window_size");
    map_                   = std::make_unique<Map>(window_size);
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
