#pragma once

#include <tbb/concurrent_queue.h>
#include <tbb/concurrent_unordered_map.h>
#include <tbb/concurrent_vector.h>

namespace YL_SLAM {

template<typename T, typename Allocator = tbb::cache_aligned_allocator<T>>
using conc_vector = tbb::concurrent_vector<T, Allocator>;

template<typename T, typename Allocator = tbb::cache_aligned_allocator<T>>
using conc_queue = tbb::concurrent_bounded_queue<T, Allocator>;

template<typename KeyT, typename ValueT>
using conc_umap = tbb::concurrent_unordered_map<KeyT, ValueT>;

} // namespace YL_SLAM