#ifndef SCDETECT_APPS_SCDETECT_BUILDER_H_
#define SCDETECT_APPS_SCDETECT_BUILDER_H_

#include <memory>

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

class NoSensorLocation : public BaseException {
 public:
  using BaseException::BaseException;
  NoSensorLocation();
};

class NoBindings : public BaseException {
 public:
  using BaseException::BaseException;
  NoBindings();
};

class NoPick : public BaseException {
 public:
  using BaseException::BaseException;
  NoPick();
};

}  // namespace builder

template <typename TProduct>
class Builder {
 public:
  Builder() = default;
  Builder(Builder &&other) = default;
  Builder &operator=(Builder &&other) = default;

  virtual std::unique_ptr<TProduct> build();

 protected:
  // finalize the product
  virtual void finalize();

  std::unique_ptr<TProduct> _product{nullptr};
};

#include "builder.ipp"

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_BUILDER_H_
