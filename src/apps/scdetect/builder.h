#ifndef SCDETECT_APPS_SCDETECT_BUILDER_H_
#define SCDETECT_APPS_SCDETECT_BUILDER_H_

#include "exception.h"

namespace Seiscomp {
namespace detect {

template <typename T> class Builder {
public:
  class BaseException : public Exception {
  public:
    using Exception::Exception;
    BaseException();
  };
};

template <typename T>
Builder<T>::BaseException::BaseException()
    : Exception("error while object creation") {}

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_BUILDER_H_
