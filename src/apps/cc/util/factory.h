#ifndef SCDETECT_APPS_CC_UTIL_FACTORY_H_
#define SCDETECT_APPS_CC_UTIL_FACTORY_H_

#include <memory>
#include <type_traits>
#include <utility>

#include "memory.h"

namespace Seiscomp {
namespace detect {
namespace util {

// possible to construct
template <typename T, typename... Args>
typename std::enable_if<std::is_constructible<T, Args...>::value,
                        std::unique_ptr<T> >::type
create(Args &&...args) {
  return util::make_unique<T>(std::forward<Args>(args)...);
}

// impossible to construct
template <typename T, typename... Args>
typename std::enable_if<!std::is_constructible<T, Args...>::value,
                        std::unique_ptr<T> >::type
create(Args &&...) {
  return nullptr;
}

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_UTIL_FACTORY_H_
