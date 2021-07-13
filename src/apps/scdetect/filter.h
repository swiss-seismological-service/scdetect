#ifndef SCDETECT_APPS_SCDETECT_FILTER_H_
#define SCDETECT_APPS_SCDETECT_FILTER_H_

#include "exception.h"

namespace Seiscomp {
namespace detect {
namespace filter {

class BaseException : public Exception {
 public:
  using Exception::Exception;
  BaseException();
};

}  // namespace filter
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_FILTER_H_
