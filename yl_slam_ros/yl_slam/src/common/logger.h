#pragma once

#include "common/setup.h"

#include <Eigen/Core>
#include <spdlog/async_logger.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

#define YL_TRACE(...) SPDLOG_LOGGER_TRACE(YL_SLAM::Logger::async_logger, __VA_ARGS__)
#define YL_DEBUG(...) SPDLOG_LOGGER_DEBUG(YL_SLAM::Logger::async_logger, __VA_ARGS__)
#define YL_INFO(...) SPDLOG_LOGGER_INFO(YL_SLAM::Logger::async_logger, __VA_ARGS__)
#define YL_WARN(...) SPDLOG_LOGGER_WARN(YL_SLAM::Logger::async_logger, __VA_ARGS__)
#define YL_ERROR(...) SPDLOG_LOGGER_ERROR(YL_SLAM::Logger::async_logger, __VA_ARGS__)
#define YL_FATAL(...)                                                                                                  \
    SPDLOG_LOGGER_CRITICAL(YL_SLAM::Logger::sync_logger, __VA_ARGS__);                                                 \
    std::abort()
#if YL_LOG_LEVEL < YL_LOG_LEVEL_INFO
#define YL_CHECK(condition, ...)                                                                                       \
    do {                                                                                                               \
        if (!(condition)) {                                                                                            \
            YL_FATAL("Check failed: " __VA_ARGS__);                                                                    \
        }                                                                                                              \
    } while (0)
#else
#define YL_CHECK(condition, ...) (void) 0
#endif

#define YL_FLOAT_FMT(x) Logger::format(x)
/// @note 若输出矩阵出错，可能时fmt版本过高，请参考: https://github.com/gabime/spdlog/issues/2746
#define YL_MATRIX_FMT(x) x.format(Logger::matrix_fmt)

namespace YL_SLAM {

/**
 * @brief Logger类，用于记录系统运行日志
 * @warning 需要在程序开始和结束时调用initialize和shutdown，这两个函数线程不安全!
 */
class Logger {
public:
    /**
     * @brief 初始化日志系统
     * @param log_to_screen 是否输出到屏幕
     * @param file_path 日志文件路径
     * @param program_name 程序名，日志文件名将以此为前缀
     */
    static void initialize(bool log_to_screen, const std::string &file_path, const std::string &program_name);

    /**
     * @brief 关闭日志系统
     */
    static void shutdown();

    /**
     * @brief 格式化double或float数据
     * @param data 待格式化数据
     * @return 格式化后的字符串，保留小数点后6位
     */
    static std::string format(const double &data);

    /**
     * @brief Eigen矩阵输出格式
     * @note 请使用YL_MATRIX_FMT宏输出矩阵
     */
    static const Eigen::IOFormat matrix_fmt;

    /**
     * @brief 需要使用的logger
     * @details critical信息使用同步logger，其余使用异步logger
     */
    static std::shared_ptr<spdlog::async_logger> async_logger;
    static std::shared_ptr<spdlog::logger> sync_logger;
};

} // namespace YL_SLAM
