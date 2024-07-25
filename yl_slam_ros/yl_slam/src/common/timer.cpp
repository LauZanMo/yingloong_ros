#include "common/timer.h"

#include <absl/strings/str_format.h>

namespace YL_SLAM {

Timer::Timer() {
    restart();
}

void Timer::restart() {
    start_ = absl::Now();
    stop_  = false;
}

double Timer::costInSec() {
    if (!stop_)
        stop();

    return absl::ToDoubleSeconds(duration_);
}

double Timer::costInMsec() {
    if (!stop_)
        stop();

    return absl::ToDoubleMilliseconds(duration_);
}

int64_t Timer::costInNsec() {
    if (!stop_)
        stop();

    return absl::ToInt64Nanoseconds(duration_);
}

void Timer::stop() {
    duration_ = absl::Now() - start_;
    stop_     = true;
}

std::string Timer::currentTime() {
    const auto cs = absl::ToCivilSecond(absl::Now(), absl::LocalTimeZone());
    return absl::StrFormat("%04d-%02d-%02d %02d-%02d-%02d", cs.year(), cs.month(), cs.day(), cs.hour(), cs.minute(),
                           cs.second());
}

} // namespace YL_SLAM