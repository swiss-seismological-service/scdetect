#include "exception.h"

namespace Seiscomp {
namespace detect {

Exception::Exception() : _msg{"base module exception"} {}

Exception::Exception(const std::string &msg) : _msg{msg} {}

const char *Exception::what() const noexcept { return _msg.c_str(); }

ValueException::ValueException() : Exception{"value error."} {}

}  // namespace detect
}  // namespace Seiscomp
