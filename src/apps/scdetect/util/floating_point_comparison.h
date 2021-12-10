#ifndef SCDETECT_APPS_SCDETECT_UTIL_FLOATINGPOINTCOMPARISON_H_
#define SCDETECT_APPS_SCDETECT_UTIL_FLOATINGPOINTCOMPARISON_H_

#include <algorithm>
#include <cmath>

namespace Seiscomp {
namespace detect {
namespace util {

// Returns `true` if the difference between two floating point numbers is
// smaller than epsilon, else `false`
template <typename TFloatingPoint>
bool almostEqual(TFloatingPoint lhs, TFloatingPoint rhs,
                 TFloatingPoint epsilon) {
  // The IEEE standard says that any comparison operation involving
  // a NAN must return false.
  if (std::isnan(lhs) || std::isnan(rhs)) {
    return false;
  }

  // From Knuth - The Art of Computer Programming
  return std::abs(rhs - lhs) <=
         std::max(std::abs(lhs), std::abs(rhs)) * epsilon;
}

// Returns `true` if `lhs` is greater than `rhs` under consideration of an
// accuracy of `epsilon`. If `lhs` is smaller than `rhs`, `false` is returned.
template <typename TFloatingPoint>
bool greaterThan(TFloatingPoint lhs, TFloatingPoint rhs,
                 TFloatingPoint epsilon) {
  // The IEEE standard says that any comparison operation involving
  // a NAN must return false.
  if (std::isnan(lhs) || std::isnan(rhs)) {
    return false;
  }

  // From Knuth - The Art of Computer Programming
  return (lhs - rhs) > std::max(std::abs(lhs), std::abs(rhs)) * epsilon;
}

// Returns `true` if `lhs` is smaller than `rhs` under consideration of an
// accuracy of `epsilon`. If `lhs` is smaller than `rhs`, `false` is returned.
template <typename TFloatingPoint>
bool lessThan(TFloatingPoint lhs, TFloatingPoint rhs, TFloatingPoint epsilon) {
  // The IEEE standard says that any comparison operation involving
  // a NAN must return false.
  if (std::isnan(lhs) || std::isnan(rhs)) {
    return false;
  }

  // From Knuth - The Art of Computer Programming
  return (rhs - lhs) > std::max(std::abs(lhs), std::abs(rhs)) * epsilon;
}

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_UTIL_FLOATINGPOINTCOMPARISON_H_
