#include "exception.h"

namespace Seiscomp {
namespace detect {

Exception::Exception() : msg_{"base module exception."} {}

Exception::Exception(const std::string &msg) : msg_{msg} {}

const char *Exception::what() const noexcept { return msg_.c_str(); }

ValueException::ValueException() : Exception{"value error."} {}

} // namespace detect
} // namespace Seiscomp
