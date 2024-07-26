#pragma once

#include <string>

namespace YL_SLAM::path_helper {

/**
 * @brief 补全路径
 * @details 若为相对路径，则补全为绝对路径，绝对路径则直接返回
 * @param path 需要补全的路径
 * @return std::string 绝对路径
 */
std::string completePath(const std::string &path);

} // namespace YL_SLAM::path_helper
