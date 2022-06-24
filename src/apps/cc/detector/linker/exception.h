#ifndef SCDETECT_APPS_CC_DETECTOR_LINKER_EXCEPTION_H_
#define SCDETECT_APPS_CC_DETECTOR_LINKER_EXCEPTION_H_

#include "../../exception.h"

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

class BaseException : public Exception {
 public:
  using Exception::Exception;
  BaseException();
};

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_DETECTOR_LINKER_EXCEPTION_H_
