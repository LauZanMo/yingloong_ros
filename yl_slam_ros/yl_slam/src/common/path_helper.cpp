#include "common/path_helper.h"
#include "common/setup.h"

#include <absl/strings/str_cat.h>

namespace YL_SLAM::path_helper {

std::string completePath(const std::string &path) {
    // 相对路径则补全为绝对路径
    if (path.front() != '/') {
        return absl::StrCat(YL_SLAM_DIR, "/", path);
    }

    // 绝对路径直接返回
    return path;
}

} // namespace YL_SLAM::path_helper
