#include "processor.h"

namespace Seiscomp {
namespace detect {

Processor::BaseException::BaseException()
    : Exception{"base processor exception"} {}

void Processor::setId(const std::string &id) { _id = id; }

const std::string &Processor::id() const { return _id; }

}  // namespace detect
}  // namespace Seiscomp
