#ifndef SCDETECT_APPS_CC_UTIL_MATH_H_
#define SCDETECT_APPS_CC_UTIL_MATH_H_

#include <cstddef>
#include <type_traits>

namespace Seiscomp {
namespace detect {
namespace util {

// Compute the mean value of `samples` based on a cumulative moving average
// algorithm
template <typename T>
double cma(T *samples, std::size_t n) {
  double cma{0};
  // cummulative moving average for samples a_0, ..., a_n:
  //
  // mean_n = mean_{n-1} + (a_n - mean_{n-1}) / n
  //
  for (std::size_t i = 0; i < n; ++i) {
    cma += (samples[i] - cma) / (i + 1);
  }
  return cma;
}

// Compute the square of `n`
template <typename T, typename = typename std::enable_if<
                          std::is_arithmetic<T>::value, T>::type>
inline T square(T n) {
  return n * n;
}

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_UTIL_MATH_H_
