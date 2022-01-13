#ifndef SCDETECT_APPS_CC_DEF_H_
#define SCDETECT_APPS_CC_DEF_H_

#include <seiscomp/math/filter.h>

namespace Seiscomp {
namespace detect {

template <typename T>
using Filter = Math::Filtering::InPlaceFilter<T>;
using DoubleFilter = Filter<double>;

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_DEF_H_
