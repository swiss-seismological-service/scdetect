#ifndef SCDETECT_APPS_SCDETECT_EXCEPTION_H_
#define SCDETECT_APPS_SCDETECT_EXCEPTION_H_

#include <exception>
#include <string>

namespace Seiscomp {
namespace detect {

class Exception : public std::exception {
 public:
  Exception();
  Exception(const std::string &msg);

  const char *what() const noexcept override;

 private:
  std::string msg_;
};

class ValueException : public Exception {
 public:
  using Exception::Exception;
  ValueException();
};

}  // namespace detect
}  // namespace Seiscomp

#endif  // SCDETECT_APPS_SCDETECT_EXCEPTION_H_
