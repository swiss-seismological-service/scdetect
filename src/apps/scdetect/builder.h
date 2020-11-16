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
};

template <typename T>
Builder<T>::BaseException::BaseException()
    : Exception("error while object creation") {}

template <typename T>
Builder<T>::NoWaveformData::NoWaveformData()
    : BaseException{"no waveform data available"} {}

template <typename T>
Builder<T>::NoStream::NoStream() : BaseException{"no stream data available"} {}

} // namespace detect
} // namespace Seiscomp

#endif // SCDETECT_APPS_SCDETECT_BUILDER_H_
