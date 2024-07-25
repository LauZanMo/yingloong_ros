#pragma once

#include <mutex>
#include <shared_mutex>
#include <tbb/spin_mutex.h>
#include <tbb/spin_rw_mutex.h>

namespace YL_SLAM {

using mutex_t    = std::mutex;
using rw_mutex_t = std::shared_mutex;
using ulock_t    = std::unique_lock<mutex_t>;
using rlock_t    = std::shared_lock<rw_mutex_t>;
using wlock_t    = std::unique_lock<rw_mutex_t>;

using spin_mutex_t    = tbb::spin_mutex;
using spin_rw_mutex_t = tbb::spin_rw_mutex;
using spin_ulock_t    = tbb::spin_mutex::scoped_lock;
using spin_rwlock_t   = tbb::spin_rw_mutex::scoped_lock; ///< 初始化第二个参数true为写锁，false为读锁

} // namespace YL_SLAM
