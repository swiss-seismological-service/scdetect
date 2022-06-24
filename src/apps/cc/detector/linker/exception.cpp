#include "exception.h"

namespace Seiscomp {
namespace detect {
namespace detector {
namespace linker {

BaseException::BaseException() : Exception{"base linker exception"} {}

}  // namespace linker
}  // namespace detector
}  // namespace detect
}  // namespace Seiscomp
