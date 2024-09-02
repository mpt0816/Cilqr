#pragma once

#include <chrono>

namespace planning {
namespace utils {

// 毫秒 1 s = 1000 ms
typedef std::chrono::time_point<std::chrono::steady_clock,
                                std::chrono::nanoseconds> time;
static inline time Time() {
  return std::chrono::time_point_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now());
}

static inline double Duration(const time& start, const time& end) {
  return static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count()) / 1e6;
}

} // namespace utils
} // namespace planning