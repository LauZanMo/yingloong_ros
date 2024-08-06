#include "common/path_helper.h"
#include "common/setup.h"

#include <absl/strings/str_cat.h>

namespace YL_SLAM::path_helper {

std::string completePath(const std::string &file) {
    // 相对路径则补全为绝对路径
    if (file.front() != '/') {
        return absl::StrCat(YL_SLAM_DIR, "/", file);
    }

    // 绝对路径直接返回
    return file;
}

std::string getFileName(const std::string &file) {
    const std::string separator = "/";
    const auto last_separator   = file.find_last_of(separator);
    if (last_separator == std::string::npos) {
        return {};
    }
    return file.substr(last_separator + 1);
}

std::string getFilePath(const std::string &file) {
    const std::string separator = "/";
    const auto last_separator   = file.find_last_of(separator);
    if (last_separator == std::string::npos) {
        return {};
    }
    return file.substr(0, last_separator);
}

} // namespace YL_SLAM::path_helper
