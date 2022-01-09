#include "filter.h"

namespace Seiscomp {
namespace detect {
namespace util {

void reset(std::unique_ptr<DoubleFilter> &filter) {
  // XXX(damb): currently the only way to achieve this is clone the filter
  if (filter) {
    filter.reset(filter->clone());
  }
}

}  // namespace util
}  // namespace detect
}  // namespace Seiscomp
