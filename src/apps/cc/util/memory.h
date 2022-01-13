#ifndef SCDETECT_APPS_CC_UTIL_MEMORY_H_
#define SCDETECT_APPS_CC_UTIL_MEMORY_H_

#include <seiscomp/core/defs.h>

#include <memory>
#include <utility>

namespace Seiscomp {
namespace detect {
namespace util {

// Provide C++11 make_unique<T>()
template <typename T, typename... Ts>
std::unique_ptr<T> make_unique(Ts &&...params) {
  return std::unique_ptr<T>(new T(std::forward<Ts>(params)...));
}

template <typename T, typename... Ts>
typename Core::SmartPointer<T>::Impl make_smart(Ts &&...params) {
  return
      typename Core::SmartPointer<T>::Impl(new T(std::forward<Ts>(params)...));
}

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_UTIL_MEMORY_H_
