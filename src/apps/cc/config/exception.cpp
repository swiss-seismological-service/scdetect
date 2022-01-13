#include "exception.h"

namespace Seiscomp {
namespace detect {
namespace config {

BaseException::BaseException() : Exception("base config exception") {}

ParserException::ParserException()
    : BaseException{"error while parsing configuration"} {}

ValidationError::ValidationError() : BaseException{"validation error"} {}

}  // namespace config
}  // namespace detect
}  // namespace Seiscomp
