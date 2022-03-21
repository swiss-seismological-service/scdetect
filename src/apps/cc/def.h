#ifndef SCDETECT_APPS_CC_DEF_H_
#define SCDETECT_APPS_CC_DEF_H_

#include <seiscomp/math/filter.h>

#include "thread_pool.hpp"

namespace Seiscomp {
namespace detect {

template <typename T>
using Filter = Math::Filtering::InPlaceFilter<T>;
using DoubleFilter = Filter<double>;

using Executor = thread_pool;

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_DEF_H_
