#ifndef SCDETECT_APPS_CC_CONFIG_EXCEPTION_H_
#define SCDETECT_APPS_CC_CONFIG_EXCEPTION_H_

#include "../exception.h"

namespace Seiscomp {
namespace detect {
namespace config {

class BaseException : public Exception {
 public:
  using Exception::Exception;
  BaseException();
};

class ParserException : public BaseException {
 public:
  using BaseException::BaseException;
  ParserException();
};

class ValidationError : public BaseException {
 public:
  using BaseException::BaseException;
  ValidationError();
};

}  // namespace config
}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_CC_CONFIG_EXCEPTION_H_
