#ifndef SCDETECT_APPS_SCDETECT_BUILDER_H_
#define SCDETECT_APPS_SCDETECT_BUILDER_H_

#include "exception.h"

namespace Seiscomp {
namespace detect {
namespace builder {

class BaseException : public Exception {
public:
  using Exception::Exception;
  BaseException();
};

class NoWaveformData : public BaseException {
public:
  using BaseException::BaseException;
  NoWaveformData();
};

class NoStream : public BaseException {
public:
  using BaseException::BaseException;
  NoStream();
};

} // namespace builder

template <typename T> class Builder {};

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_BUILDER_H_
