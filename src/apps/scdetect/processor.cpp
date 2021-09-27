#include "processor.h"

namespace Seiscomp {
namespace detect {

Processor::Processor(const std::string &id) : _id{id} {}

Processor::~Processor() {}

Processor::BaseException::BaseException()
    : Exception{"base processor exception"} {}

const std::string &Processor::id() const { return _id; }

}  // namespace detect
}  // namespace Seiscomp
