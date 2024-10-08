#include "common/logger.h"

#include <absl/strings/str_cat.h>
#include <absl/strings/str_format.h>
#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

namespace YL_SLAM {

void Logger::initialize(bool log_to_screen, const std::string &file_path, const std::string &program_name) {
    // 初始化默认日志器
    const auto file_name = absl::StrCat(file_path, "/", program_name, ".log");
    std::vector<spdlog::sink_ptr> sinks{std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name)};
    if (log_to_screen) { // 若输出到屏幕，则设置控制台输出
        sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
    }

    spdlog::init_thread_pool(8192, 1);
    async_logger =
            std::make_shared<spdlog::async_logger>("async_logger", sinks.begin(), sinks.end(), spdlog::thread_pool());
    sync_logger = std::make_shared<spdlog::logger>("sync_logger", sinks.begin(), sinks.end());
    spdlog::register_logger(async_logger);
    spdlog::register_logger(sync_logger);

    // 设置日志格式: "[时间][线程 代码行][级别] 内容"
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S][thread %t %s:%#][%^%l%$] %v");

    // 设置日志级别
    spdlog::set_level(static_cast<spdlog::level::level_enum>(SPDLOG_ACTIVE_LEVEL));

    // 设置出现critical级别日志时立即刷新缓冲区
    spdlog::flush_on(spdlog::level::critical);
}

void Logger::shutdown() {
    spdlog::drop_all();
}

std::string Logger::format(const double &data) {
    return absl::StrFormat("%0.6lf", data);
}

const Eigen::IOFormat Logger::matrix_fmt{15, 0, ", ", ",\n", "", "", "\n[", "]"};

std::shared_ptr<spdlog::async_logger> Logger::async_logger;
std::shared_ptr<spdlog::logger> Logger::sync_logger;

} // namespace YL_SLAM
