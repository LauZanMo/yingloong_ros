#pragma once

#include "common/setup.h"

#include <absl/time/clock.h>

#if YL_LOG_LEVEL < YL_LOG_LEVEL_INFO
#define YL_DECLARE_TIMER(name) YL_SLAM::Timer name
#define YL_START_TIMER(name) name.restart()
#define YL_STOP_TIMER(name) name.costInMsec()
#else
#define YL_DECLARE_TIMER(name)
#define YL_START_TIMER(name) (void) 0
#define YL_STOP_TIMER(name) (void) 0
#endif

namespace YL_SLAM {

/**
 * @brief 计时器类
 * @details 该类用于计时，支持秒/毫秒计时与当前时间获取
 * @warning 该类线程不安全
 */
class Timer {
public:
    /**
     * @brief 构造函数
     * @details 构造后会自动开始计时
     */
    Timer();

    /**
     * @brief 重新开始计时
     */
    void restart();

    /**
     * @brief 停止计时并获取计时时间（秒）
     * @return 计时时间（秒）
     */
    double costInSec();

    /**
     * @brief 停止计时并获取计时时间（毫秒）
     * @return 计时时间（毫秒）
     */
    double costInMsec();

    /**
     * @brief 停止计时并获取计时时间（纳秒）
     * @return 计时时间（纳秒）
     */
    int64_t costInNsec();

    /**
     * @brief 获取当前时间（年-月-日 时-分-秒）
     * @return 当前时间字符串
     */
    static std::string currentTime();

private:
    /**
     * @brief 停止计时
     * @details 该函数会在costInSec和costInMsec中调用
     */
    void stop();

    absl::Time start_;
    absl::Duration duration_;
    bool stop_{false};
};

} // namespace YL_SLAM
