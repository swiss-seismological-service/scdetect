#ifndef SCDETECT_APPS_SCDETECT_UTIL_FILTER_H_
#define SCDETECT_APPS_SCDETECT_UTIL_FILTER_H_

#include <memory>

#include "../waveform.h"

namespace Seiscomp {
namespace detect {
namespace util {

// Resets `filter`
void reset(std::unique_ptr<waveform::DoubleFilter> &filter);

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_UTIL_FILTER_H_
